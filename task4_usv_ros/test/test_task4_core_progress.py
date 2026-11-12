import unittest

from task4_usv_ros.task4_core_node import _progress_payload


class Task4CoreProgressTests(unittest.TestCase):
    def test_finish_payload_contains_only_progress_contract(self):
        self.assertEqual(
            _progress_payload("FINISHED"),
            {
                "vehicle": "usv",
                "task": "TASK4",
                "progress": "FINISHED",
                "error": None,
            },
        )

    def test_error_is_plain_text_not_an_encoded_status(self):
        self.assertEqual(
            _progress_payload("TASK4_ERROR", error="target rejected"),
            {
                "vehicle": "usv",
                "task": "TASK4",
                "progress": "TASK4_ERROR",
                "error": "target rejected",
            },
        )

    def test_empty_progress_is_rejected(self):
        with self.assertRaises(ValueError):
            _progress_payload("")


if __name__ == "__main__":
    unittest.main()
