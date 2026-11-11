import math
import unittest

from task4_usv_ros.task4_controller_node import (
    apply_avoidance_to_thrust,
    differential_thrust,
)


class Task4ControllerTests(unittest.TestCase):
    def pose(self, x, y, theta=0.0):
        class Pose:
            pass

        value = Pose()
        value.x, value.y, value.theta = x, y, theta
        return value

    def test_forward_target_uses_equal_thrust(self):
        left, right, distance, error = differential_thrust(
            self.pose(0.0, 0.0), self.pose(10.0, 0.0)
        )
        self.assertAlmostEqual(left, right)
        self.assertAlmostEqual(distance, 10.0)
        self.assertAlmostEqual(error, 0.0)
        self.assertGreater(left, 0.0)

    def test_positive_heading_error_differentiates_thrust(self):
        left, right, _distance, error = differential_thrust(
            self.pose(0.0, 0.0), self.pose(0.0, 10.0)
        )
        self.assertAlmostEqual(error, math.pi / 2.0)
        self.assertGreater(left, right)

    def test_commands_are_limited(self):
        left, right, _distance, _error = differential_thrust(
            self.pose(0.0, 0.0, math.pi), self.pose(100.0, 0.0), max_thrust=80.0
        )
        self.assertGreaterEqual(left, 0.0)
        self.assertGreaterEqual(right, 0.0)
        self.assertLessEqual(left, 80.0)
        self.assertLessEqual(right, 80.0)

    def test_hold_decision_zeros_thrust(self):
        self.assertEqual(
            apply_avoidance_to_thrust(
                100.0,
                100.0,
                action="HOLD",
                steering_side=None,
                speed_scale=0.0,
                max_thrust=300.0,
                turn_thrust=80.0,
            ),
            (0.0, 0.0),
        )

    def test_replan_steers_to_requested_side_at_reduced_speed(self):
        left, right = apply_avoidance_to_thrust(
            200.0,
            200.0,
            action="REPLAN",
            steering_side="LEFT",
            speed_scale=0.5,
            max_thrust=300.0,
            turn_thrust=80.0,
        )
        self.assertGreater(left, right)
        self.assertLessEqual(left, 300.0)
        self.assertGreaterEqual(right, 0.0)

    def test_invalid_replan_fails_safe(self):
        self.assertEqual(
            apply_avoidance_to_thrust(
                100.0,
                100.0,
                action="REPLAN",
                steering_side=None,
                speed_scale=0.5,
                max_thrust=300.0,
                turn_thrust=80.0,
            ),
            (0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
