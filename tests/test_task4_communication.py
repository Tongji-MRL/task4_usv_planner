import unittest

from task4_usv_planner.task4_communication import (
    Task4CommunicationSession,
    parse_assistance_request,
    parse_readiness_confirm,
    normalize_ocs_command,
)


class Task4CommunicationTests(unittest.TestCase):
    def setUp(self):
        self.session = Task4CommunicationSession(vehicle_id="USV1", team_id="TONG")
        self.command = {
            "team_id": "TONG",
            "seq": 41,
            "vehicle_id": "USV1",
            "body": {
                "type": "assistance_request",
                "position": {"latitude": 31.2, "longitude": 121.5},
                "vehicle_type": "TYPE_USV",
            },
        }

    def test_official_assistance_request_decodes_and_acknowledges(self):
        request = parse_assistance_request(self.command, expected_vehicle_id="USV1")
        event = self.session.accept_assistance(request)
        self.assertTrue(event.accepted)
        ack = self.session.incident_ack(request.command_seq)
        self.assertEqual(ack["vehicle_id"], "USV1")
        self.assertEqual(ack["seq"], 1)
        self.assertEqual(ack["body"], {"type": "incident_ack", "command_seq": 41})

    def test_readiness_report_uses_per_vehicle_report_sequence(self):
        request = parse_assistance_request(self.command, expected_vehicle_id="USV1")
        self.session.accept_assistance(request)
        self.session.incident_ack(request.command_seq)
        report = self.session.readiness_report(request.command_seq)
        self.assertEqual(report["seq"], 2)
        self.assertEqual(report["body"], {"type": "readiness", "command_seq": 41})
        self.assertEqual(self.session.active_report_seq, 2)

    def test_matching_readiness_confirm_releases_the_vehicle(self):
        request = parse_assistance_request(self.command, expected_vehicle_id="USV1")
        self.session.accept_assistance(request)
        self.session.incident_ack(request.command_seq)
        self.session.readiness_report(request.command_seq)
        confirm = parse_readiness_confirm(
            {
                "seq": 100,
                "body": {
                    "type": "readiness_confirm",
                    "report_seq": 2,
                    "vehicle_id": "USV1",
                },
            },
            expected_vehicle_id="USV1",
        )
        self.assertTrue(self.session.accept_readiness_confirm(confirm).accepted)
        self.assertIsNone(self.session.active_command_seq)

    def test_wrong_vehicle_type_and_stale_commands_are_rejected(self):
        uav = dict(self.command)
        uav["body"] = dict(self.command["body"], vehicle_type="TYPE_UAV")
        self.assertFalse(
            self.session.accept_assistance(
                parse_assistance_request(uav, expected_vehicle_id="USV1")
            ).accepted
        )
        request = parse_assistance_request(self.command, expected_vehicle_id="USV1")
        self.assertTrue(self.session.accept_assistance(request).accepted)
        self.assertFalse(self.session.accept_assistance(request).accepted)

    def test_protobuf_json_camel_case_body_is_supported(self):
        request = parse_assistance_request(
            {
                "seq": 7,
                "body": {
                    "assistanceRequest": {
                        "position": {"latitude": 1.0, "longitude": 2.0},
                        "vehicleType": "TYPE_USV",
                    }
                },
            },
            expected_vehicle_id="USV1",
        )
        self.assertEqual(request.command_seq, 7)
        self.assertEqual(request.vehicle_type, "USV")

    def test_ocs_vehicle_link_envelope_is_supported(self):
        command = normalize_ocs_command(
            {
                "kind": "command",
                "vehicle_id": "USV1",
                "payload": {
                    "command": "task4_assistance",
                    "command_seq": 9,
                    "parameters": {
                        "official_command_seq": 9,
                        "position": {"latitude": 3.0, "longitude": 4.0},
                        "vehicle_type": 1,
                    },
                },
            }
        )
        request = parse_assistance_request(command, expected_vehicle_id="USV1")
        self.assertEqual(request.command_seq, 9)
        self.assertEqual(request.vehicle_type, "USV")


if __name__ == "__main__":
    unittest.main()
