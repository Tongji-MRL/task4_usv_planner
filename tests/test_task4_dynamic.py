import math
import unittest

from task4_usv_planner import (
    MovingObjectObservation,
    NEDPoint,
    Pose2D,
    Task4DynamicPlanner,
    Task4PlanStatus,
    Task4PlannerConfig,
)


class Task4DynamicPlannerTests(unittest.TestCase):
    def setUp(self):
        self.planner = Task4DynamicPlanner(Task4PlannerConfig(
            min_clearance_m=10.0,
            usv_footprint_radius_m=0.0,
            uncertainty_margin_m=0.0,
            detour_margin_m=1.0,
            prediction_horizon_s=20.0,
            prediction_step_s=0.5,
            max_observation_age_s=5.0,
            nominal_usv_speed_mps=1.0,
        ))

    @staticmethod
    def observation(north, east, heading, speed=0.0, timestamp=100.0):
        return MovingObjectObservation(
            NEDPoint(north, east, 0.0), heading, speed, timestamp,
        )

    def test_prediction_compensates_for_observation_age(self):
        track = self.planner.predict_track(
            self.observation(
                0.0,
                0.0,
                math.pi / 2,
                speed=2.0,
            ),
            age_s=3.0,
        )

        # Heading pi/2 means East in our NED convention.
        # In 3 seconds at 2 m/s, the object has already moved 6 m East.
        self.assertAlmostEqual(track.start_ned.north, 0.0, places=7)
        self.assertAlmostEqual(track.start_ned.east, 6.0, places=7)

        # Prediction horizon is another 20 seconds:
        # 6 m + 2 m/s * 20 s = 46 m East.
        self.assertAlmostEqual(track.end_ned.north, 0.0, places=7)
        self.assertAlmostEqual(track.end_ned.east, 46.0, places=7)

    def test_plan_uses_compensated_object_position(self):
        result = self.planner.plan(
            Pose2D(-30.0, 51.0, 0.0),
            [Pose2D(30.0, 51.0, 0.0)],
            self.observation(
                0.0,
                0.0,
                math.pi / 2,
                speed=2.0,
                timestamp=97.0,
            ),
            now_s=100.0,
        )

        # Without age compensation the predicted corridor ends at E=40 m,
        # so this route would appear safe by 1 m.
        # After compensating the 3-second delay, the corridor reaches E=46 m
        # before predicting forward, so the original route is unsafe.
        self.assertEqual(result.status, Task4PlanStatus.DETOUR)

    def test_prediction_uses_heading_and_speed(self):
        track = self.planner.predict_track(
            self.observation(0.0, 0.0, math.pi / 2, speed=2.0)
        )
        self.assertAlmostEqual(track.end_ned.north, 0.0, places=7)
        self.assertAlmostEqual(track.end_ned.east, 40.0, places=7)
        self.assertAlmostEqual(track.position_at(10.0).east, 20.0, places=7)
        self.assertEqual(track.protected_radius_m, 10.0)

    def test_unaffected_mission_proceeds(self):
        result = self.planner.plan(
            Pose2D(-20.0, -30.0, 0.0),
            [Pose2D(-20.0, 30.0, 0.0)],
            self.observation(20.0, 0.0, 0.0, speed=1.0),
            now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.PROCEED)
        self.assertEqual(result.waypoints[-1], Pose2D(-20.0, 30.0, 0.0))
        self.assertGreaterEqual(result.minimum_clearance_m, 10.0)

    def test_crossing_projected_corridor_gets_detour(self):
        current = Pose2D(-30.0, 0.0, 0.0)
        goal = Pose2D(30.0, 0.0, 0.0)
        result = self.planner.plan(
            current, [goal], self.observation(0.0, -20.0, math.pi / 2, speed=2.0), now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.DETOUR)
        self.assertIn(result.detour_side, ("LEFT", "RIGHT"))
        self.assertGreaterEqual(result.minimum_clearance_m, 10.0)
        self.assertEqual(result.waypoints[0], current)
        self.assertEqual(result.waypoints[-1], goal)

    def test_static_object_is_bypassed(self):
        result = self.planner.plan(
            Pose2D(-30.0, 0.0, 0.0),
            [Pose2D(30.0, 0.0, 0.0)],
            self.observation(0.0, 0.0, 0.0),
            now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.DETOUR)
        self.assertGreaterEqual(result.minimum_clearance_m, 10.0)

    def test_stale_observation_is_rejected(self):
        result = self.planner.plan(
            Pose2D(0.0, 0.0, 0.0),
            [Pose2D(20.0, 0.0, 0.0)],
            self.observation(0.0, 0.0, 0.0, timestamp=90.0),
            now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.INVALID_INPUT)
        self.assertIn("stale", result.reason)

    def test_already_inside_corridor_requires_escape(self):
        result = self.planner.plan(
            Pose2D(0.0, 1.0, 0.0),
            [Pose2D(50.0, 0.0, 0.0)],
            self.observation(0.0, 0.0, 0.0, speed=1.0),
            now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.EMERGENCY_ESCAPE)
        self.assertEqual(result.waypoints[0], Pose2D(0.0, 1.0, 0.0))

    def test_large_detour_margin_still_proves_a_safe_bypass(self):
        planner = Task4DynamicPlanner(Task4PlannerConfig(
            min_clearance_m=10.0,
            usv_footprint_radius_m=0.0,
            uncertainty_margin_m=0.0,
            detour_margin_m=100.0,
            prediction_horizon_s=20.0,
            prediction_step_s=1.0,
            max_observation_age_s=5.0,
            nominal_usv_speed_mps=1.0,
        ))
        current = Pose2D(-30.0, 0.0, 0.0)
        result = planner.plan(
            current,
            [Pose2D(30.0, 0.0, 0.0)],
            self.observation(0.0, -20.0, math.pi / 2, speed=2.0),
            now_s=100.0,
        )
        self.assertEqual(result.status, Task4PlanStatus.DETOUR)
        self.assertGreaterEqual(result.minimum_clearance_m, 10.0)


if __name__ == "__main__":
    unittest.main()
