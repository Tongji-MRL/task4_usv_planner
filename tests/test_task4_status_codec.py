import json
import unittest

from task4_usv_planner import CommonError, Task4Step, encode_progress
from task4_usv_planner.task4_status_codec import (
    TASK4_ERROR_DEFINITIONS,
    TASK4_ERROR_CATALOG_VERSION,
)


class Task4StatusCodecTests(unittest.TestCase):
    def test_normal_progress_matches_main_fsm_contract(self):
        payload = encode_progress("TASK4_ACK", command_seq=12)
        self.assertEqual(
            payload,
            {
                "vehicle": "usv",
                "task": "TASK4",
                "progress": "TASK4_ACK",
                "error": None,
                "command": None,
                "command_seq": 12,
            },
        )

    def test_error_is_one_flat_string_code(self):
        payload = encode_progress(
            "TASK4_ERROR",
            command_seq=12,
            step=Task4Step.GO_TO_TARGET,
            error=CommonError.TARGET_UNREACHABLE,
            detail="target is outside the configured transit range",
            fault_source="task4_core_node",
            detail_code="TARGET_UNREACHABLE",
        )
        self.assertEqual(payload["error"]["code"], "TARGET_UNREACHABLE")
        self.assertEqual(payload["error"]["step"], "GO_TO_TARGET")
        self.assertEqual(payload["error"]["source"], "task4_core_node")
        self.assertNotIn("status_code", payload)
        self.assertNotIn("status_bits", payload)
        self.assertEqual(TASK4_ERROR_CATALOG_VERSION, "task4-errors-v2")

    def test_error_table_has_no_step_cartesian_product(self):
        self.assertTrue(TASK4_ERROR_DEFINITIONS)
        self.assertTrue(all(isinstance(code, str) for code in TASK4_ERROR_DEFINITIONS))

    def test_json_round_trip_keeps_null_error(self):
        payload = encode_progress("TASK4_READY")
        self.assertIsNone(json.loads(json.dumps(payload))["error"])

    def test_unknown_error_is_rejected(self):
        with self.assertRaises(ValueError):
            encode_progress("TASK4_ERROR", error="NOT_IN_TABLE")


if __name__ == "__main__":
    unittest.main()
