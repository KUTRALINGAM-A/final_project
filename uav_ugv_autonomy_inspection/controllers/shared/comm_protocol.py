"""File-based communication protocol for UAV–UGV bidirectional messaging.

Because the RosbotXl PROTO exposes only a `lidarSlot` (not a general
`extensionSlot`), we cannot cleanly inject Emitter/Receiver devices into
both robots.  Instead we use a lightweight **file-mailbox** pattern:

    controllers/comm_mailbox/
        uav_inbox.json   ← messages for the UAV
        ugv_inbox.json   ← messages for the UGV

Each robot writes to the *other's* inbox and reads from its own.
Files are written atomically (write-to-temp + rename) so partial reads
never occur.  Duplicate/stale message protection uses request IDs and
timestamps.

Message schema (JSON):
{
  "message_type": str,
  "request_id":   str,
  "sender":       str,        # "crazyflie" | "rosbot_xl"
  "recipient":    str,
  "timestamp":    float,      # simulation seconds
  "position":     [x, y, z],  # optional
  "target":       [x, y, z],  # optional
  "assistance_type": str,     # optional
  "status":       str,        # optional
  "error":        str         # optional
}
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

# ── Message types ─────────────────────────────────────────────────────
MSG_ASSISTANCE_REQUEST  = "ASSISTANCE_REQUEST"
MSG_REQUEST_ACK         = "REQUEST_ACK"
MSG_RENDEZVOUS_TARGET   = "RENDEZVOUS_TARGET"
MSG_RENDEZVOUS_ACCEPTED = "RENDEZVOUS_ACCEPTED"
MSG_ARRIVED             = "ARRIVED"
MSG_INSPECTION_REPORT   = "INSPECTION_REPORT"
MSG_ASSISTANCE_COMPLETE = "ASSISTANCE_COMPLETE"
MSG_CANCEL_REQUEST      = "CANCEL_REQUEST"
MSG_HEARTBEAT           = "HEARTBEAT"
MSG_ERROR               = "ERROR"

# ── Sender / recipient identifiers ───────────────────────────────────
ID_UAV = "crazyflie"
ID_UGV = "rosbot_xl"

# ── Mailbox directory (relative to each controller's working dir) ────
_DEFAULT_MAILBOX_DIR = os.path.join(os.path.dirname(__file__), "..", "comm_mailbox")


class CommMessage:
    """Immutable wrapper around a single protocol message."""

    __slots__ = ("data",)

    def __init__(self, data: Dict[str, Any]):
        self.data = dict(data)

    # -- Convenience accessors ------------------------------------------
    @property
    def message_type(self) -> str:
        return self.data.get("message_type", "")

    @property
    def request_id(self) -> str:
        return self.data.get("request_id", "")

    @property
    def sender(self) -> str:
        return self.data.get("sender", "")

    @property
    def recipient(self) -> str:
        return self.data.get("recipient", "")

    @property
    def timestamp(self) -> float:
        return self.data.get("timestamp", 0.0)

    @property
    def position(self) -> Optional[List[float]]:
        return self.data.get("position")

    @property
    def target(self) -> Optional[List[float]]:
        return self.data.get("target")

    @property
    def assistance_type(self) -> Optional[str]:
        return self.data.get("assistance_type")

    @property
    def status(self) -> Optional[str]:
        return self.data.get("status")

    @property
    def target_name(self) -> Optional[str]:
        return self.data.get("target_name")

    @property
    def findings(self) -> Optional[Dict[str, Any]]:
        return self.data.get("findings")

    @property
    def inspection_position(self) -> Optional[List[float]]:
        return self.data.get("inspection_position")

    @property
    def error(self) -> Optional[str]:
        return self.data.get("error")

    def __repr__(self) -> str:
        return f"CommMessage({self.message_type}, id={self.request_id}, from={self.sender})"


class MailboxComm:
    """Non-blocking file-mailbox communication endpoint.

    Parameters
    ----------
    my_id : str
        Identifier for *this* robot ("crazyflie" or "rosbot_xl").
    mailbox_dir : str | Path | None
        Path to the shared mailbox directory.
    """

    def __init__(self, my_id: str, mailbox_dir: str | Path | None = None, peer_id: Optional[str] = None):
        self.my_id = my_id
        self.peer_id = peer_id if peer_id is not None else (ID_UGV if my_id == ID_UAV else ID_UAV)
        self.mailbox_dir = Path(mailbox_dir or _DEFAULT_MAILBOX_DIR).resolve()
        self.mailbox_dir.mkdir(parents=True, exist_ok=True)

        # Inbox file names
        self._my_inbox = self.mailbox_dir / f"{my_id}_inbox.json"
        self._peer_inbox = self.mailbox_dir / f"{self.peer_id}_inbox.json"

        # Duplicate detection
        self._seen_ids: set[str] = set()
        self._last_read_ts: float = 0.0

        # Initialise empty inboxes if they don't exist
        for inbox in (self._my_inbox, self._peer_inbox):
            if not inbox.exists():
                self._write_json(inbox, [])

    def set_peer(self, peer_id: str) -> None:
        """Dynamically update recipient peer ID (e.g. switching between diadem and rosbot_xl)."""
        self.peer_id = peer_id
        self._peer_inbox = self.mailbox_dir / f"{self.peer_id}_inbox.json"
        if not self._peer_inbox.exists():
            self._write_json(self._peer_inbox, [])

    # ── Public API ────────────────────────────────────────────────────

    def send(
        self,
        message_type: str,
        request_id: str,
        sim_time: float,
        *,
        position: Optional[List[float]] = None,
        target: Optional[List[float]] = None,
        assistance_type: Optional[str] = None,
        status: Optional[str] = None,
        error: Optional[str] = None,
        target_name: Optional[str] = None,
        findings: Optional[Dict[str, Any]] = None,
        inspection_position: Optional[List[float]] = None,
        **extra_kwargs: Any,
    ) -> CommMessage:
        """Enqueue a message into the peer's inbox. Returns the sent message."""
        msg_data: Dict[str, Any] = {
            "message_type": message_type,
            "request_id": request_id,
            "sender": self.my_id,
            "recipient": self.peer_id,
            "timestamp": sim_time,
        }
        if position is not None:
            msg_data["position"] = list(position)
        if target is not None:
            msg_data["target"] = list(target)
        if assistance_type is not None:
            msg_data["assistance_type"] = assistance_type
        if status is not None:
            msg_data["status"] = status
        if error is not None:
            msg_data["error"] = error
        if target_name is not None:
            msg_data["target_name"] = str(target_name)
        if findings is not None:
            msg_data["findings"] = dict(findings)
        if inspection_position is not None:
            msg_data["inspection_position"] = list(inspection_position)
        msg_data.update(extra_kwargs)

        # Append to peer inbox safely
        try:
            existing = self._read_json(self._peer_inbox)
            existing.append(msg_data)
            self._write_json(self._peer_inbox, existing)
        except Exception as exc:
            print(f"[COMM WARN] send({message_type}) failed: {exc}")

        return CommMessage(msg_data)

    def receive(self, sim_time: float, ttl: float = 60.0) -> List[CommMessage]:
        """Read and consume new, non-duplicate, non-expired messages.

        Messages older than *ttl* seconds (simulation time) are discarded.
        Already-seen request_id+type combos are filtered.
        """
        raw = self._read_json(self._my_inbox)
        if not raw:
            return []

        new_messages: List[CommMessage] = []
        retained: List[Dict[str, Any]] = []

        for entry in raw:
            msg = CommMessage(entry)
            uid = f"{msg.request_id}:{msg.message_type}:{msg.timestamp}"
            age = sim_time - msg.timestamp
            if age > ttl:
                continue  # expired
            if uid in self._seen_ids:
                retained.append(entry)  # keep in file for safety; skip delivery
                continue
            self._seen_ids.add(uid)
            new_messages.append(msg)
            retained.append(entry)

        # Only rewrite inbox if messages were actually expired/pruned
        if len(retained) < len(raw):
            try:
                self._write_json(self._my_inbox, retained)
            except Exception as exc:
                print(f"[COMM WARN] receive prune failed: {exc}")

        return new_messages

    def clear_inbox(self) -> None:
        """Remove all messages from this robot's inbox."""
        self._write_json(self._my_inbox, [])
        self._seen_ids.clear()

    # ── Convenience senders ───────────────────────────────────────────

    def send_assistance_request(
        self,
        request_id: str,
        sim_time: float,
        position: List[float],
        assistance_type: str = "general",
        target: Optional[List[float]] = None,
        inspection_position: Optional[List[float]] = None,
        target_name: Optional[str] = None,
        **kwargs: Any,
    ) -> CommMessage:
        payload: Dict[str, Any] = {
            "position": list(position),
            "assistance_type": assistance_type,
        }
        if target is not None:
            payload["target"] = list(target)
        if inspection_position is not None:
            payload["inspection_position"] = list(inspection_position)
        if target_name is not None:
            payload["target_name"] = str(target_name)
        payload.update(kwargs)
        return self.send(
            MSG_ASSISTANCE_REQUEST,
            request_id,
            sim_time,
            **payload,
        )

    def send_ack(self, request_id: str, sim_time: float, position: Optional[List[float]] = None) -> CommMessage:
        return self.send(MSG_REQUEST_ACK, request_id, sim_time, position=position, status="accepted")

    def send_rendezvous_target(
        self, request_id: str, sim_time: float, target: List[float],
    ) -> CommMessage:
        return self.send(MSG_RENDEZVOUS_TARGET, request_id, sim_time, target=target)

    def send_rendezvous_accepted(self, request_id: str, sim_time: float) -> CommMessage:
        return self.send(MSG_RENDEZVOUS_ACCEPTED, request_id, sim_time, status="accepted")

    def send_arrived(self, request_id: str, sim_time: float, position: List[float]) -> CommMessage:
        return self.send(MSG_ARRIVED, request_id, sim_time, position=position, status="arrived")

    def send_inspection_report(
        self,
        request_id: str,
        sim_time: float,
        target_name: str,
        findings: Dict[str, Any],
        uav_position: Optional[List[float]] = None,
    ) -> CommMessage:
        kwargs: Dict[str, Any] = {"target_name": target_name, "findings": findings}
        if uav_position is not None:
            kwargs["position"] = list(uav_position)
        return self.send(MSG_INSPECTION_REPORT, request_id, sim_time, **kwargs)

    def send_complete(self, request_id: str, sim_time: float) -> CommMessage:
        return self.send(MSG_ASSISTANCE_COMPLETE, request_id, sim_time, status="complete")

    def send_cancel(self, request_id: str, sim_time: float, reason: str = "") -> CommMessage:
        return self.send(MSG_CANCEL_REQUEST, request_id, sim_time, status="cancelled", error=reason)

    def send_error(self, request_id: str, sim_time: float, error: str) -> CommMessage:
        return self.send(MSG_ERROR, request_id, sim_time, error=error)

    def send_heartbeat(self, sim_time: float, position: List[float]) -> CommMessage:
        return self.send(MSG_HEARTBEAT, f"hb_{self.my_id}", sim_time, position=position)

    # ── File I/O helpers ──────────────────────────────────────────────

    @staticmethod
    def _read_json(path: Path) -> List[Dict[str, Any]]:
        if not path.exists():
            return []
        for _ in range(5):
            try:
                text = path.read_text(encoding="utf-8")
                if not text.strip():
                    return []
                data = json.loads(text)
                return data if isinstance(data, list) else []
            except (FileNotFoundError, json.JSONDecodeError, OSError, PermissionError):
                time.sleep(0.005)
        return []

    @staticmethod
    def _write_json(path: Path, data: List[Dict[str, Any]]) -> None:
        """Safe write with retry and fallback for Windows file locking."""
        path.parent.mkdir(parents=True, exist_ok=True)
        # Attempt atomic replace with retries
        for attempt in range(10):
            tmp_fd, tmp_path = tempfile.mkstemp(
                dir=str(path.parent), suffix=".tmp", prefix=".msg_"
            )
            try:
                with os.fdopen(tmp_fd, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                # On Windows, os.replace can raise PermissionError if target is locked
                os.replace(tmp_path, str(path))
                return
            except (PermissionError, OSError):
                try:
                    if os.path.exists(tmp_path):
                        os.unlink(tmp_path)
                except OSError:
                    pass
                time.sleep(0.005 * (attempt + 1))

        # Fallback: direct write if os.replace is blocked (e.g. file watcher/editor hold)
        for attempt in range(5):
            try:
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                return
            except (PermissionError, OSError):
                time.sleep(0.01 * (attempt + 1))

        print(f"[COMM WARN] Could not write to {path} after retries")


def make_request_id(prefix: str, counter: int) -> str:
    """Generate a unique request ID."""
    return f"{prefix}_{counter:04d}"
