"""Non-blocking JSONL telemetry for the real-time control path."""

from __future__ import annotations

import json
import queue
import threading
from pathlib import Path
from typing import Any

import numpy as np


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


class JsonlTelemetry:
    def __init__(self, path: str | Path | None, queue_size: int = 512):
        self.path = None if path in (None, "") else Path(path).expanduser().resolve()
        self.dropped_records = 0
        self._queue: queue.Queue[dict[str, Any] | None] | None = None
        self._thread: threading.Thread | None = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._queue = queue.Queue(maxsize=max(int(queue_size), 1))
            self._thread = threading.Thread(
                target=self._writer_loop,
                name="mpcc-jsonl-writer",
                daemon=True,
            )
            self._thread.start()

    def _writer_loop(self) -> None:
        assert self.path is not None and self._queue is not None
        with self.path.open("a", encoding="utf-8", buffering=64 * 1024) as stream:
            while True:
                record = self._queue.get()
                if record is None:
                    self._queue.task_done()
                    break
                stream.write(json.dumps(_plain(record), sort_keys=True, allow_nan=False))
                stream.write("\n")
                self._queue.task_done()

    def write(self, record: dict[str, Any]) -> bool:
        """Queue one record without ever waiting in the control callback."""
        if self._queue is None:
            return True
        try:
            self._queue.put_nowait(record)
            return True
        except queue.Full:
            self.dropped_records += 1
            return False

    def close(self, timeout_s: float = 2.0) -> None:
        if self._queue is None or self._thread is None:
            return
        try:
            self._queue.put(None, timeout=max(float(timeout_s), 0.0))
        except queue.Full:
            return
        self._thread.join(timeout=max(float(timeout_s), 0.0))
