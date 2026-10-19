import json
import unittest

from task4_usv_planner import CommonError, Task4StatusCode, Task4Step, encode_progress
from task4_usv_planner.task4_status_codec import (
    TASK4_ERROR_CATALOG_VERSION,
    TASK4_ERROR_DEFINITIONS,
    PlannerCommand,
)


class Task4StatusCodecTests(unittest.TestCase):
    def test_matches_task3_14_bit_layout_and_round_trips(self):
        status = Task4StatusCode(
            step=Task4Step.GO_TO_TARGET,
            error=CommonError.NONE,
            command=PlannerCommand.NONE,
        )
        self.assertEqual(status.bit_string(), "1 11 0001 0000 000")
        self.assertEqual(Task4StatusCode.unpack(status.pack()), status)

    def test_every_step_has_stable_specific_error_codes(self):
        expected_codes = set(range(0b1000, 0b1_0000))
        self.assertEqual(set(TASK4_ERROR_DEFINITIONS), set(Task4Step))
        for step, definitions in TASK4_ERROR_DEFINITIONS.items():
            with self.subTest(step=step.name):
                self.assertEqual(set(definitions), expected_codes)
                self.assertEqual(len({item.name for item in definitions.values()}), 8)

    def test_normal_progress_keeps_pdf_compatible_fields(self):
        payload = encode_progress("TASK4_ACK", command_seq=12)
        self.assertEqual(payload["vehicle"], "usv")
        self.assertEqual(payload["task"], "TASK4")
        self.assertEqual(payload["progress"], "TASK4_ACK")
        self.assertIsNone(payload["error"])
        self.assertIsNone(payload["command"])
        self.assertEqual(payload["command_seq"], 12)
        self.assertEqual(payload["status_code"], 14336)

    def test_fault_progress_contains_decoder_and_recovery_fields(self):
        payload = encode_progress(
            "TASK4_ERROR",
            command_seq=12,
            step=Task4Step.GO_TO_TARGET,
            error=0b1001,
            detail="target is outside the configured transit range",
            fault_source="task4_core_node",
            detail_code="TARGET_UNREACHABLE",
        )
        self.assertEqual(payload["error"]["name"], "TASK4_TARGET_UNREACHABLE")
        self.assertEqual(payload["error"]["catalog_version"], TASK4_ERROR_CATALOG_VERSION)
        self.assertEqual(payload["error"]["recovery_action"], "REPLAN_FROM_CURRENT_POSE")
        self.assertEqual(json.loads(json.dumps(payload))["status_bits"], "1 11 0001 1001 000")

    def test_unknown_error_requires_detail(self):
        with self.assertRaises(ValueError):
            encode_progress("TASK4_ERROR", error=CommonError.UNKNOWN_ERROR)


if __name__ == "__main__":
    unittest.main()
