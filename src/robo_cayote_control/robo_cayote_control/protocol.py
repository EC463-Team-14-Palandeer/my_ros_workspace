import json
import time
from typing import Callable


Validator = Callable[[dict], tuple[bool, str]]
SummaryBuilder = Callable[[dict], dict]


def validate_estop(payload: dict) -> tuple[bool, str]:
    """Validate an estop command payload."""
    if not isinstance(payload.get("estop"), bool):
        return False, "'estop' field missing or not a boolean"
    if not isinstance(payload.get("source"), str):
        return False, "'source' field missing or not a string"
    if not isinstance(payload.get("ts"), (int, float)):
        return False, "'ts' field missing or not a number"
    return True, ""


def validate_navigation(payload: dict) -> tuple[bool, str]:
    """Validate a navigation update payload."""
    if payload.get("type") != "navigation_update":
        return False, f"'type' must be 'navigation_update', got '{payload.get('type')}'"

    geofence = payload.get("geofence")
    if not isinstance(geofence, list) or len(geofence) < 3:
        return False, "'geofence' must be a list with at least 3 points"
    for index, point in enumerate(geofence):
        if not isinstance(point.get("lat"), (int, float)) or not isinstance(
            point.get("lng"), (int, float)
        ):
            return False, f"geofence[{index}] missing 'lat' or 'lng'"

    path = payload.get("path")
    if not isinstance(path, list) or len(path) == 0:
        return False, "'path' must be a non-empty list of waypoints"
    for index, waypoint in enumerate(path):
        if not isinstance(waypoint.get("lat"), (int, float)) or not isinstance(
            waypoint.get("lng"), (int, float)
        ):
            return False, f"path[{index}] missing 'lat' or 'lng'"
        if "returnToBase" in waypoint and not isinstance(
            waypoint.get("returnToBase"), bool
        ):
            return False, f"path[{index}] missing 'returnToBase' boolean"

    if not isinstance(payload.get("repeat"), bool):
        return False, "'repeat' field missing or not a boolean"
    if not isinstance(payload.get("timestamp"), (int, float)):
        return False, "'timestamp' field missing or not a number"

    normalize_navigation(payload)
    return True, ""


def normalize_navigation(payload: dict) -> dict:
    """Fill optional navigation fields with defaults expected by the robot."""
    for waypoint in payload.get("path", []):
        waypoint.setdefault("returnToBase", False)
    return payload


def summarize_estop(payload: dict) -> dict:
    return {
        "estop": payload["estop"],
        "source": payload["source"],
    }


def summarize_navigation(payload: dict) -> dict:
    return {
        "waypoint_count": len(payload["path"]),
        "geofence_count": len(payload["geofence"]),
        "repeat": payload["repeat"],
    }


def create_message_id(topic: str) -> str:
    topic_slug = topic.strip("/").replace("/", "_")
    return f"{topic_slug}-{int(time.time() * 1000)}"


def build_ack_payload(topic: str, message_id: str) -> dict:
    return {
        "stage": "ack",
        "status": "received",
        "topic": topic,
        "message_id": message_id,
        "ts": int(time.time() * 1000),
    }


def build_result_payload(
    topic: str,
    message_id: str,
    ok: bool,
    extra: dict | None = None,
) -> dict:
    payload = {
        "stage": "result",
        "status": "ok" if ok else "error",
        "topic": topic,
        "message_id": message_id,
        "ts": int(time.time() * 1000),
    }
    if extra:
        payload.update(extra)
    return payload


def process_incoming_message(
    raw_message: str,
    topic: str,
    validator: Validator,
    summary_builder: SummaryBuilder,
) -> tuple[dict, dict]:
    """
    Produce the two-step handshake for an incoming command.

    1. Immediate ACK proving the robot received the message.
    2. Final validation result that is either OK or ERROR.
    """
    message_id = create_message_id(topic)
    ack_payload = build_ack_payload(topic, message_id)

    try:
        payload = json.loads(raw_message)
    except json.JSONDecodeError:
        result_payload = build_result_payload(
            topic,
            message_id,
            ok=False,
            extra={"reason": "JSON parse error"},
        )
        return ack_payload, result_payload

    valid, reason = validator(payload)
    if not valid:
        result_payload = build_result_payload(
            topic,
            message_id,
            ok=False,
            extra={"reason": reason},
        )
        return ack_payload, result_payload

    result_payload = build_result_payload(
        topic,
        message_id,
        ok=True,
        extra=summary_builder(payload),
    )
    return ack_payload, result_payload
