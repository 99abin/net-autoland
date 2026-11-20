"""Opt-in local timing traces. Never participates in flight authorization.

Set AUTOLAND_TIMING_DIRECTORY to enable. Callbacks enqueue bounded records;
one background thread writes/flushes them and samples clock/process progress.
Monotonic clocks may be compared only for records with the same host/boot ID.
"""
from collections import deque
from functools import wraps
import hashlib
import itertools
import json
import os
from pathlib import Path
import queue
import socket
import threading
import time
import uuid


def command_fields(command):
    fields = {name: getattr(command, name) for name in (
        "plan_id", "valid", "target_held", "target_age_s", "terminal_ready",
        "terminal_mode", "target_passed", "forward_range_m", "time_to_target_s",
        "crossing_error_y_m", "crossing_error_z_m", "position_std_y_m",
        "position_std_z_m", "roll_rad", "pitch_rad", "yaw_enu_rad", "thrust", "reason")}
    fields.update(stamp_ns=command.header.stamp.sec * 10**9 + command.header.stamp.nanosec,
                  frame_id=command.header.frame_id)
    return fields


def command_key(command):
    """Match unchanged wire messages without adding fields to ROS interfaces."""
    return hashlib.sha256(json.dumps(command_fields(command), sort_keys=True,
                                    separators=(",", ":")).encode()).hexdigest()


class TimingRecorder:
    def __init__(self, node, directory=None):
        self.node = node
        directory = os.environ.get("AUTOLAND_TIMING_DIRECTORY") if directory is None else directory
        self.enabled = bool(directory)
        self.dropped = 0
        self.error = None
        self.ids = itertools.count(1)
        self.callbacks = deque(maxlen=32)
        self.active_callback = None
        self.stop = threading.Event()
        self.thread = None
        if not self.enabled:
            return
        self.queue = queue.Queue(maxsize=20000)
        name = getattr(node, "get_name", lambda: type(node).__name__)()
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"{name}-{os.getpid()}-{uuid.uuid4().hex[:8]}.jsonl"
        self.stream = self.path.open("x", buffering=65536)
        boot = Path('/proc/sys/kernel/random/boot_id')
        self.record("session", node=name, pid=os.getpid(), host=socket.gethostname(),
                    boot_id=boot.read_text().strip() if boot.exists() else None,
                    scheduler_fields="main-thread run_ns, runqueue_wait_ns, timeslices",
                    dds_timestamp_note="Installed Humble executor passes messages only; unavailable timestamps stay null",
                    matching="command SHA256; repeated identical keys can be ambiguous")
        self.thread = threading.Thread(target=self._write, name="timing-writer", daemon=True)
        self.thread.start()

    def record(self, event, **fields):
        if not self.enabled or self.stop.is_set():
            return None
        identifier = next(self.ids)
        entry = dict(event=event, event_id=identifier, monotonic_ns=time.monotonic_ns(),
                     system_ns=time.time_ns(), process_cpu_ns=time.process_time_ns(),
                     thread_cpu_ns=time.thread_time_ns(),
                     ros_ns=self.node.get_clock().now().nanoseconds, **fields)
        try:
            self.queue.put_nowait(entry)
        except queue.Full:
            self.dropped += 1
        return identifier

    def command(self, event, command, **fields):
        if self.enabled:
            self.record(event, command_key=command_key(command),
                        command=command_fields(command), **fields)

    def wrap(self, name, callback):
        if not self.enabled:
            return callback

        @wraps(callback)
        def invoke(*args, **kwargs):
            started = time.monotonic_ns()
            cpu = time.thread_time_ns()
            identity = self.record("callback_enter", callback=name)
            self.active_callback = dict(callback=name, started_ns=started, event_id=identity)
            try:
                return callback(*args, **kwargs)
            finally:
                detail = dict(callback=name, enter_event_id=identity,
                              duration_ns=time.monotonic_ns() - started,
                              callback_cpu_ns=time.thread_time_ns() - cpu)
                self.callbacks.append(detail)
                self.record("callback_exit", **detail)
                self.active_callback = None
        return invoke

    def context(self):
        return dict(enabled=self.enabled, dropped=self.dropped, error=self.error,
                    active_callback=self.active_callback, recent_callbacks=list(self.callbacks))

    def _write(self):
        heartbeat = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    entry = self.queue.get(timeout=0.05)
                    self.stream.write(json.dumps(entry, separators=(",", ":")) + "\n")
                except queue.Empty:
                    pass
                now = time.monotonic()
                if now >= heartbeat and not self.stop.is_set():
                    sched = Path('/proc/self/schedstat')
                    self.record("heartbeat", active_callback=self.active_callback,
                                dropped=self.dropped, load_average=os.getloadavg(),
                                main_thread_scheduler=(sched.read_text().strip() if sched.exists() else None))
                    self.stream.flush()
                    heartbeat = now + 0.1
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            self.enabled = False
        finally:
            self.stream.close()

    def close(self):
        if self.thread is not None:
            self.record("shutdown", dropped=self.dropped)
            self.stop.set()
            self.thread.join(timeout=2)
