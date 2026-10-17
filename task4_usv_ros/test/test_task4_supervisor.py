import unittest

from task4_usv_ros.task4_supervisor_node import resolve_planner_result


class Task4SupervisorDecisionTests(unittest.TestCase):
    def setUp(self):
        self.path = [
            {"north_m": 1.0, "east_m": 2.0, "yaw_rad": 0.5},
            {"north_m": 3.0, "east_m": 4.0, "yaw_rad": 1.0},
        ]

    def test_proceed_uses_planner_path(self):
        decision = resolve_planner_result({
            "status": "PROCEED",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_PROCEED")
        self.assertEqual(len(decision.waypoints), 2)

    def test_detour_uses_planner_path(self):
        decision = resolve_planner_result({
            "status": "DETOUR",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_DETOUR")
        self.assertEqual(decision.waypoints[0], (1.0, 2.0, 0.5))

    def test_hold_clears_previous_detour_path(self):
        decision = resolve_planner_result({
            "status": "HOLD",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_HOLD")
        self.assertEqual(decision.waypoints, ())

    def test_emergency_escape_uses_escape_path(self):
        decision = resolve_planner_result({
            "status": "EMERGENCY_ESCAPE",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_ESCAPE")
        self.assertEqual(len(decision.waypoints), 2)

    def test_abort_clears_active_path(self):
        decision = resolve_planner_result({
            "status": "ABORT",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_ABORT")
        self.assertEqual(decision.waypoints, ())

    def test_invalid_input_fails_safe_to_hold(self):
        decision = resolve_planner_result({
            "status": "INVALID_INPUT",
            "planned_waypoints": self.path,
        })
        self.assertEqual(decision.state, "TASK4_HOLD")
        self.assertEqual(decision.waypoints, ())


if __name__ == "__main__":
    unittest.main()
