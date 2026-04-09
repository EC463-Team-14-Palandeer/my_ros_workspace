import json

from robo_cayote_control.protocol import (
    normalize_navigation,
    process_incoming_message,
    summarize_estop,
    summarize_navigation,
    validate_estop,
    validate_navigation,
)


def test_estop_message_emits_ack_then_ok_result():
    raw_message = json.dumps(
        {
            "estop": True,
            "source": "frontend",
            "ts": 1710000000,
        }
    )

    ack_payload, result_payload = process_incoming_message(
        raw_message=raw_message,
        topic="estop",
        validator=validate_estop,
        summary_builder=summarize_estop,
    )

    assert ack_payload["stage"] == "ack"
    assert ack_payload["status"] == "received"
    assert ack_payload["topic"] == "estop"
    assert ack_payload["message_id"] == result_payload["message_id"]

    assert result_payload["stage"] == "result"
    assert result_payload["status"] == "ok"
    assert result_payload["estop"] is True
    assert result_payload["source"] == "frontend"


def test_navigation_message_emits_error_result_when_schema_is_wrong():
    raw_message = json.dumps(
        {
            "type": "navigation_update",
            "geofence": [{"lat": 1.0, "lng": 2.0}, {"lat": 3.0, "lng": 4.0}],
            "path": [],
            "repeat": False,
            "timestamp": 1710000000,
        }
    )

    ack_payload, result_payload = process_incoming_message(
        raw_message=raw_message,
        topic="navigation",
        validator=validate_navigation,
        summary_builder=summarize_navigation,
    )

    assert ack_payload["status"] == "received"
    assert result_payload["status"] == "error"
    assert "reason" in result_payload


def test_invalid_json_still_emits_receipt_ack():
    ack_payload, result_payload = process_incoming_message(
        raw_message="{bad json",
        topic="navigation",
        validator=validate_navigation,
        summary_builder=summarize_navigation,
    )

    assert ack_payload["stage"] == "ack"
    assert ack_payload["status"] == "received"
    assert result_payload["stage"] == "result"
    assert result_payload["status"] == "error"
    assert result_payload["reason"] == "JSON parse error"


def test_navigation_allows_missing_return_to_base_and_defaults_it():
    payload = {
        "type": "navigation_update",
        "geofence": [
            {"lat": 1.0, "lng": 2.0},
            {"lat": 3.0, "lng": 4.0},
            {"lat": 5.0, "lng": 6.0},
        ],
        "path": [
            {"lat": 1.5, "lng": 2.5},
            {"lat": 3.5, "lng": 4.5, "returnToBase": True},
        ],
        "repeat": True,
        "timestamp": 1710000000,
    }

    valid, reason = validate_navigation(payload)

    assert valid is True
    assert reason == ""
    assert payload["path"][0]["returnToBase"] is False
    assert payload["path"][1]["returnToBase"] is True


def test_navigation_normalizer_sets_default_return_to_base():
    payload = {"path": [{"lat": 1.0, "lng": 2.0}]}

    normalized = normalize_navigation(payload)

    assert normalized["path"][0]["returnToBase"] is False
