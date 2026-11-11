import math
import unittest

from task4_usv_planner import (
    AvoidanceAction,
    ReactiveAvoidanceConfig,
    ReactiveAvoidanceCore,
    SteeringSide,
)


def scan_with_cluster(angle_deg=0.0, distance=2.0, *, count=181):
    values = [math.inf] * count
    center = int(round(angle_deg + 90.0))
    for index in range(center - 2, center + 3):
        values[index] = distance
    return values


class ReactiveAvoidanceTests(unittest.TestCase):
    def setUp(self):
        self.core = ReactiveAvoidanceCore(ReactiveAvoidanceConfig(
            trigger_frames=2,
            release_frames=2,
        ))
        self.args = dict(
            angle_min=-math.pi / 2,
            angle_increment=math.pi / 180,
            range_min=0.1,
            range_max=12.0,
        )

    def evaluate(self, ranges, stamp):
        return self.core.evaluate(ranges, timestamp=stamp, **self.args)

    def test_stable_obstacle_requests_replan(self):
        scan = scan_with_cluster(distance=2.0)
        self.assertEqual(self.evaluate(scan, 1.0).action, AvoidanceAction.PROCEED)
        result = self.evaluate(scan, 1.1)
        self.assertEqual(result.action, AvoidanceAction.REPLAN)
        self.assertIn(result.steering_side, (SteeringSide.LEFT, SteeringSide.RIGHT))
        self.assertLess(result.speed_scale, 1.0)

    def test_emergency_obstacle_holds(self):
        scan = scan_with_cluster(distance=0.6)
        self.evaluate(scan, 1.0)
        result = self.evaluate(scan, 1.1)
        self.assertEqual(result.action, AvoidanceAction.HOLD)
        self.assertEqual(result.speed_scale, 0.0)

    def test_single_ray_noise_is_ignored(self):
        scan = [math.inf] * 181
        scan[90] = 0.5
        self.assertEqual(self.evaluate(scan, 1.0).action, AvoidanceAction.PROCEED)
        self.assertEqual(self.evaluate(scan, 1.1).action, AvoidanceAction.PROCEED)

    def test_release_hysteresis_clears_latch(self):
        blocked = scan_with_cluster(distance=2.0)
        clear = [math.inf] * 181
        self.evaluate(blocked, 1.0)
        self.assertEqual(self.evaluate(blocked, 1.1).action, AvoidanceAction.REPLAN)
        self.assertNotEqual(self.evaluate(clear, 1.2).action, AvoidanceAction.PROCEED)
        self.assertEqual(self.evaluate(clear, 1.3).action, AvoidanceAction.PROCEED)

    def test_invalid_scan_fails_safe(self):
        result = self.evaluate([], 1.0)
        self.assertEqual(result.action, AvoidanceAction.HOLD)
        self.assertEqual(result.speed_scale, 0.0)


if __name__ == "__main__":
    unittest.main()
