import math
import unittest

from task4_usv_ros.task4_safety_monitor_node import (
    MovingObjectState,
    evaluate_safety,
)


class Task4SafetyMonitorTests(unittest.TestCase):
    def evaluate(self, north, east, heading=0.0, speed=0.0, timestamp=100.0, **kwargs):
        return evaluate_safety(
            (0.0, 0.0),
            MovingObjectState(north, east, heading, speed, timestamp),
            now_s=kwargs.pop("now_s", 100.0),
            **kwargs,
        )

    def test_current_distance_above_warning_is_safe(self):
        result = self.evaluate(30.0, 0.0)
        self.assertEqual(result.state, "SAFE")
        self.assertAlmostEqual(result.distance_m, 30.0)

    def test_current_distance_inside_protected_radius_is_emergency(self):
        result = self.evaluate(10.0, 0.0)
        self.assertEqual(result.state, "EMERGENCY")
        self.assertAlmostEqual(result.distance_m, 10.0)

    def test_short_horizon_approach_requires_replan(self):
        result = self.evaluate(20.0, 0.0, heading=math.pi, speed=2.0)
        self.assertEqual(result.state, "REPLAN_REQUIRED")
        self.assertAlmostEqual(result.distance_m, 20.0)
        self.assertAlmostEqual(result.predicted_min_distance_m, 10.0)

    def test_stale_observation_requires_replan(self):
        result = self.evaluate(30.0, 0.0, timestamp=90.0)
        self.assertEqual(result.state, "REPLAN_REQUIRED")
        self.assertIn("stale", result.reason)
        self.assertIsNone(result.distance_m)

    def test_timestamp_and_heading_compensate_current_position(self):
        result = self.evaluate(
            20.0,
            5.0,
            heading=math.pi / 2,
            speed=2.0,
            timestamp=98.0,
        )
        self.assertAlmostEqual(result.distance_m, math.hypot(20.0, 9.0))

    def test_invalid_observation_fails_safe(self):
        result = self.evaluate(math.nan, 0.0)
        self.assertEqual(result.state, "REPLAN_REQUIRED")
        self.assertIn("invalid numeric data", result.reason)


if __name__ == "__main__":
    unittest.main()
