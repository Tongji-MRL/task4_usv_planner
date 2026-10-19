import math
import unittest

from task4_usv_ros.task4_pose_adapter_node import (
    enu_to_ned_pose,
    gps_to_local_ned,
    normalize_angle,
)


class Task4PoseAdapterTests(unittest.TestCase):
    def test_enu_position_is_reordered_and_origin_is_removed(self):
        north, east, down, heading = enu_to_ned_pose(
            x_enu=12.0,
            y_enu=25.0,
            z_enu=3.0,
            yaw_enu_rad=0.0,
            origin_enu=(10.0, 20.0, 5.0),
        )
        self.assertEqual((north, east, down), (5.0, 2.0, 2.0))
        self.assertAlmostEqual(heading, math.pi / 2.0)

    def test_north_and_east_heading_mapping(self):
        self.assertAlmostEqual(enu_to_ned_pose(0, 0, 0, math.pi / 2)[3], 0.0)
        self.assertAlmostEqual(enu_to_ned_pose(0, 0, 0, -math.pi / 2)[3], -math.pi)

    def test_angle_normalization(self):
        self.assertGreaterEqual(normalize_angle(7.0), -math.pi)
        self.assertLess(normalize_angle(7.0), math.pi)
        self.assertAlmostEqual(normalize_angle(-math.pi), -math.pi)

    def test_gps_fallback_uses_local_ned_metres(self):
        north, east, down = gps_to_local_ned(
            30.0001, 120.0001, 98.0, (30.0, 120.0, 100.0)
        )
        self.assertAlmostEqual(north, 11.1319, places=2)
        self.assertAlmostEqual(east, 9.6406, places=2)
        self.assertAlmostEqual(down, 2.0)


if __name__ == "__main__":
    unittest.main()
