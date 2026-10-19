import math
import unittest

from task4_usv_planner import NEDPoint, Pose2D
from task4_usv_planner import GpsReference
from task4_usv_planner.core_state_machine import (
    CoreCheckpoint,
    CorePlannerConfig,
    CoreState,
    CoreTask4Begin,
    Task4CoreStateMachine,
)


class Task4CoreStateMachineTests(unittest.TestCase):
    def setUp(self):
        self.current = Pose2D(0.0, 0.0, 0.0)
        self.command = CoreTask4Begin(100, NEDPoint(30.0, 40.0, 0.0))
        self.checkpoint = CoreCheckpoint("usv", "TASK1", "SEARCH_TARGET")
        self.machine = Task4CoreStateMachine()

    def test_begin_reports_checkpoint_and_ack(self):
        snapshot = self.machine.receive_task4_begin(
            self.command, self.checkpoint, self.current
        )
        self.assertEqual(snapshot.state, CoreState.TASK4_ACK)
        self.assertEqual(snapshot.progress_to_report, "TASK4_ACK")
        self.assertEqual(snapshot.checkpoint, self.checkpoint)
        self.assertEqual(snapshot.active_setpoint.x, 30.0)
        self.assertEqual(snapshot.active_setpoint.y, 40.0)
        self.assertAlmostEqual(snapshot.active_setpoint.yaw, math.atan2(40.0, 30.0))

    def test_transit_reaches_target_and_reports_ready(self):
        self.machine.receive_task4_begin(self.command, self.checkpoint, self.current)
        self.assertEqual(self.machine.tick(self.current).state, CoreState.TRANSIT)
        ready = self.machine.tick(Pose2D(30.0, 40.0, 1.0))
        self.assertEqual(ready.state, CoreState.TASK4_READY)
        self.assertEqual(ready.progress_to_report, "TASK4_READY")

    def test_resume_only_records_main_planner_instruction(self):
        self.machine.receive_task4_begin(self.command, self.checkpoint, self.current)
        self.machine.tick(self.current)
        self.machine.tick(Pose2D(30.0, 40.0, 0.0))
        resumed = self.machine.receive_resume("TASK1", "SEARCH_TARGET")
        self.assertEqual(resumed.state, CoreState.RESUMED)
        self.assertEqual(resumed.resume_task, "TASK1")
        self.assertEqual(resumed.resume_progress, "SEARCH_TARGET")
        self.assertIsNone(resumed.active_setpoint)

    def test_resume_before_ready_is_rejected(self):
        with self.assertRaises(RuntimeError):
            self.machine.receive_resume("TASK1", "SEARCH_TARGET")

    def test_far_target_is_rejected_before_ack(self):
        machine = Task4CoreStateMachine(CorePlannerConfig(max_transit_distance_m=10.0))
        with self.assertRaises(ValueError):
            machine.receive_task4_begin(
                CoreTask4Begin(103, NEDPoint(11.0, 0.0, 0.0)),
                self.checkpoint,
                self.current,
            )

    def test_only_usv_begin_commands_are_accepted(self):
        command = CoreTask4Begin(101, NEDPoint(1.0, 2.0, 0.0), target_vehicle="uav")
        self.assertEqual(command.target_vehicle, "uav")

    def test_gps_begin_is_converted_to_local_ned(self):
        reference = GpsReference(30.0, 120.0, 100.0)
        command = CoreTask4Begin.from_gps(
            102, 30.0001, 120.0001, reference, altitude_m=98.0
        )
        self.assertAlmostEqual(command.target_ned.north, 11.1319, places=2)
        self.assertAlmostEqual(command.target_ned.east, 9.6406, places=2)
        self.assertAlmostEqual(command.target_ned.down, 2.0)


if __name__ == "__main__":
    unittest.main()
