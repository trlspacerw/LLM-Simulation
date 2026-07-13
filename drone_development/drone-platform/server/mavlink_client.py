"""Thread-safe pymavlink connection manager with automatic reconnect.

This module owns the single MAVLink socket to mavlink-router. One background
thread receives and dispatches every message; a second thread sends a 1 Hz
GCS heartbeat so ArduPilot keeps accepting GUIDED commands. All outgoing
traffic goes through a lock, so any thread (e.g. FastAPI worker threads
handling REST requests) may send safely.

Link supervision: the link is considered up only while autopilot HEARTBEATs
keep arriving. If none is seen for HEARTBEAT_TIMEOUT_S, the socket is torn
down and reopened, which transparently picks up a restarted SITL. Link
up/down transitions are reported to registered link listeners (telemetry
forwards them to the WebSocket clients as "link" events).
"""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Callable, Dict, List, Optional

from pymavlink import mavutil

logger = logging.getLogger(__name__)

# Seconds without an autopilot HEARTBEAT before the link is declared down.
HEARTBEAT_TIMEOUT_S = 5.0
# Delay between reconnect attempts after a failure.
RECONNECT_DELAY_S = 2.0
# Default time to wait for a COMMAND_ACK.
DEFAULT_ACK_TIMEOUT_S = 5.0

# SET_POSITION_TARGET type_mask: use only the position fields, ignore
# velocity, acceleration, yaw and yaw rate (bits 3..8, 10, 11).
POSITION_TARGET_TYPEMASK_POSITION_ONLY = 0x0DF8

MessageListener = Callable[[object], None]  # receives a MAVLink_message
LinkListener = Callable[[bool], None]       # receives connected: bool


class LinkDownError(RuntimeError):
    """Raised when an operation needs the MAVLink link but it is down."""


class AckTimeoutError(TimeoutError):
    """Raised when no COMMAND_ACK arrives within the timeout."""


def mav_result_name(result: int) -> str:
    """Human-readable name for a MAV_RESULT value."""
    entry = mavutil.mavlink.enums["MAV_RESULT"].get(result)
    return entry.name if entry is not None else f"MAV_RESULT_{result}"


class CommandRejectedError(RuntimeError):
    """Raised when the autopilot ACKs a command with a non-ACCEPTED result."""

    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result
        self.result_name = mav_result_name(result)
        super().__init__(f"command {command} rejected: {self.result_name}")


class MavlinkClient:
    """Manages one pymavlink connection shared by the whole gateway."""

    def __init__(self, endpoint: str = "udpin:0.0.0.0:14551",
                 source_system: int = 245) -> None:
        self._endpoint = endpoint
        self._source_system = source_system
        self._conn: Optional[mavutil.mavfile] = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._connected = False
        self._last_heartbeat = 0.0  # time.monotonic() of last FCU heartbeat
        self._ack_lock = threading.Lock()
        self._ack_waiters: Dict[int, "queue.Queue[object]"] = {}
        self._message_listeners: List[MessageListener] = []
        self._link_listeners: List[LinkListener] = []
        self._rx_thread: Optional[threading.Thread] = None
        self._hb_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ API

    @property
    def connected(self) -> bool:
        return self._connected

    def add_message_listener(self, listener: MessageListener) -> None:
        """Register a callback invoked (from the RX thread) for every message."""
        self._message_listeners.append(listener)

    def add_link_listener(self, listener: LinkListener) -> None:
        """Register a callback invoked on link up/down transitions."""
        self._link_listeners.append(listener)

    def start(self) -> None:
        """Spawn the receive and heartbeat threads."""
        self._stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="mavlink-rx", daemon=True)
        self._hb_thread = threading.Thread(
            target=self._gcs_heartbeat_loop, name="mavlink-hb", daemon=True)
        self._rx_thread.start()
        self._hb_thread.start()
        logger.info("MavlinkClient started on %s", self._endpoint)

    def stop(self) -> None:
        """Stop the background threads and close the connection."""
        self._stop.set()
        for thread in (self._rx_thread, self._hb_thread):
            if thread is not None:
                thread.join(timeout=3.0)
        self._teardown()

    def mode_mapping(self) -> Dict[str, int]:
        """Mode name -> custom_mode id for the connected vehicle type.

        Only available once a heartbeat has been received (pymavlink derives
        the table from the vehicle type in HEARTBEAT).
        """
        conn = self._require_link()
        mapping = conn.mode_mapping()
        if not mapping:
            raise LinkDownError("mode mapping unavailable (no heartbeat yet)")
        return dict(mapping)

    def command_long(self, command: int,
                     p1: float = 0.0, p2: float = 0.0, p3: float = 0.0,
                     p4: float = 0.0, p5: float = 0.0, p6: float = 0.0,
                     p7: float = 0.0,
                     ack_timeout: float = DEFAULT_ACK_TIMEOUT_S) -> int:
        """Send a COMMAND_LONG and block until its COMMAND_ACK.

        Returns the MAV_RESULT on success (always MAV_RESULT_ACCEPTED).
        Raises CommandRejectedError, AckTimeoutError or LinkDownError.
        """
        conn = self._require_link()
        waiter: "queue.Queue[object]" = queue.Queue(maxsize=4)
        with self._ack_lock:
            self._ack_waiters[command] = waiter
        try:
            msg = conn.mav.command_long_encode(
                conn.target_system, conn.target_component,
                command, 0, p1, p2, p3, p4, p5, p6, p7)
            self.send_message(msg)
            deadline = time.monotonic() + ack_timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AckTimeoutError(
                        f"no COMMAND_ACK for command {command} "
                        f"within {ack_timeout:.1f}s")
                try:
                    ack = waiter.get(timeout=remaining)
                except queue.Empty:
                    raise AckTimeoutError(
                        f"no COMMAND_ACK for command {command} "
                        f"within {ack_timeout:.1f}s") from None
                # Keep waiting through interim IN_PROGRESS acks.
                if ack.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                    break
        finally:
            with self._ack_lock:
                self._ack_waiters.pop(command, None)
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise CommandRejectedError(command, ack.result)
        return int(ack.result)

    def send_position_target_global(self, lat: float, lon: float,
                                    relative_alt: float) -> None:
        """Send a position-only SET_POSITION_TARGET_GLOBAL_INT (GUIDED goto).

        Altitude is relative to home (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT).
        Note: this message has no COMMAND_ACK in the MAVLink protocol.
        """
        conn = self._require_link()
        msg = conn.mav.set_position_target_global_int_encode(
            0,  # time_boot_ms (ignored by ArduPilot)
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_TARGET_TYPEMASK_POSITION_ONLY,
            int(lat * 1e7), int(lon * 1e7), float(relative_alt),
            0, 0, 0,   # vx, vy, vz
            0, 0, 0,   # ax, ay, az
            0, 0)      # yaw, yaw_rate
        self.send_message(msg)

    def send_message(self, msg: object) -> None:
        """Thread-safe raw message send."""
        conn = self._conn
        if conn is None or not self._connected:
            raise LinkDownError("MAVLink link is down")
        with self._send_lock:
            conn.mav.send(msg)

    # ------------------------------------------------------------ internals

    def _require_link(self) -> mavutil.mavfile:
        conn = self._conn
        if conn is None or not self._connected:
            raise LinkDownError("MAVLink link is down")
        return conn

    def _open(self) -> None:
        try:
            logger.info("opening MAVLink connection %s", self._endpoint)
            self._conn = mavutil.mavlink_connection(
                self._endpoint,
                source_system=self._source_system,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
        except Exception:
            logger.exception("failed to open %s", self._endpoint)
            self._conn = None

    def _teardown(self) -> None:
        conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass

    def _set_connected(self, connected: bool) -> None:
        if connected == self._connected:
            return
        self._connected = connected
        logger.warning("MAVLink link %s", "UP" if connected else "DOWN")
        for listener in self._link_listeners:
            try:
                listener(connected)
            except Exception:
                logger.exception("link listener failed")

    def _rx_loop(self) -> None:
        """Single receive thread: read, dispatch, watch the heartbeat."""
        while not self._stop.is_set():
            if self._conn is None:
                self._open()
                if self._conn is None:
                    self._stop.wait(RECONNECT_DELAY_S)
                    continue
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.5)
            except Exception:
                logger.exception("recv failed, reopening connection")
                self._set_connected(False)
                self._teardown()
                self._stop.wait(RECONNECT_DELAY_S)
                continue

            now = time.monotonic()
            if msg is not None and msg.get_type() != "BAD_DATA":
                self._handle_message(msg, now)

            # Heartbeat watchdog: declare the link down and rebuild the
            # socket so a restarted SITL is picked up cleanly.
            if self._connected and now - self._last_heartbeat > HEARTBEAT_TIMEOUT_S:
                logger.warning("heartbeat lost (> %.1fs), reconnecting",
                               HEARTBEAT_TIMEOUT_S)
                self._set_connected(False)
                self._teardown()

    def _handle_message(self, msg: object, now: float) -> None:
        mtype = msg.get_type()
        if mtype == "HEARTBEAT":
            # Only autopilot heartbeats keep the link alive; other GCS
            # heartbeats routed our way must not mask a dead SITL.
            if msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self._last_heartbeat = now
                self._set_connected(True)
        elif mtype == "COMMAND_ACK":
            with self._ack_lock:
                waiter = self._ack_waiters.get(msg.command)
            if waiter is not None:
                try:
                    waiter.put_nowait(msg)
                except queue.Full:
                    pass
        for listener in self._message_listeners:
            try:
                listener(msg)
            except Exception:
                logger.exception("message listener failed")

    def _gcs_heartbeat_loop(self) -> None:
        """Send a 1 Hz GCS heartbeat; ArduPilot needs it to trust the GCS."""
        while not self._stop.wait(1.0):
            conn = self._conn
            if conn is None:
                continue
            try:
                with self._send_lock:
                    conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                logger.debug("heartbeat send failed (link down?)")
