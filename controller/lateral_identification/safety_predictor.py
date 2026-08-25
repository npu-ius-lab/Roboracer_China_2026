"""Asynchronous robust track-margin prediction for identification runs.

The rollout is intentionally isolated in a spawned process.  On the vehicle's
CPU, spline projection can take longer than one 20 ms command period; keeping
it outside the ROS process prevents odometry callbacks and command publication
from being starved by Python's interpreter lock.
"""

from __future__ import annotations

import multiprocessing as mp
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

from f1tenth_dynamic_mpcc.track_model import PeriodicTrack

try:
    from .track_guided_core import GuidedProfile, TrackGuidedController
except ImportError:  # Direct execution through run_experiment.py.
    from track_guided_core import GuidedProfile, TrackGuidedController


@dataclass(frozen=True)
class SafetyPrediction:
    sequence: int
    submitted_monotonic: float
    baseline_margin: float
    excitation_margin: float
    evaluated_excitation_rad: float
    compute_seconds: float
    error: str = ""


def _worker(
    connection, track_path: str, profile_values: dict, role: str
) -> None:
    try:
        try:
            track = PeriodicTrack(Path(track_path))
            profile = GuidedProfile(**profile_values)
            controller = TrackGuidedController(track, profile)
            connection.send({"ready": True, "error": ""})
        except Exception as error:
            connection.send({
                "ready": False,
                "error": f"{type(error).__name__}: {error}",
            })
            return
        while True:
            request = connection.recv()
            if request is None:
                break
            started = time.perf_counter()
            try:
                arguments = (
                    request["x"], request["y"], request["yaw"], request["vx"],
                    request["elapsed"], request["s_guess"], request["steering"],
                )
                excitation_amplitude = abs(request["future_excitation"])
                if role == "baseline":
                    baseline = controller.predicted_minimum_margin(
                        *arguments, future_excitation=0.0
                    )
                    excitation = baseline
                elif excitation_amplitude > 1.0e-9:
                    positive = controller.predicted_minimum_margin(
                        *arguments, future_excitation=excitation_amplitude
                    )
                    negative = controller.predicted_minimum_margin(
                        *arguments, future_excitation=-excitation_amplitude
                    )
                    excitation = min(positive, negative)
                    baseline = excitation
                else:
                    baseline = controller.predicted_minimum_margin(
                        *arguments, future_excitation=0.0
                    )
                    excitation = baseline
                response = SafetyPrediction(
                    sequence=request["sequence"],
                    submitted_monotonic=request["submitted_monotonic"],
                    baseline_margin=baseline,
                    excitation_margin=excitation,
                    evaluated_excitation_rad=excitation_amplitude,
                    compute_seconds=time.perf_counter() - started,
                )
            except Exception as error:
                response = SafetyPrediction(
                    sequence=request["sequence"],
                    submitted_monotonic=request["submitted_monotonic"],
                    baseline_margin=float("-inf"),
                    excitation_margin=float("-inf"),
                    evaluated_excitation_rad=abs(request["future_excitation"]),
                    compute_seconds=time.perf_counter() - started,
                    error=f"{type(error).__name__}: {error}",
                )
            connection.send(asdict(response))
    except (EOFError, BrokenPipeError):
        pass
    finally:
        connection.close()


class AsyncSafetyPredictor:
    """Two-process, single-flight safety predictor with non-blocking polling.

    One process evaluates exact baseline tracking while the second evaluates
    both excitation signs. The wall time is therefore two rollouts rather than
    three, without assuming that the baseline trajectory is bracketed by the
    two excitation trajectories on a curved track.
    """

    def __init__(self, track_path: Path, profile: GuidedProfile):
        context = mp.get_context("spawn")
        self._connections = []
        self._processes = []
        resolved_track = str(Path(track_path).resolve())
        for role in ("baseline", "excitation_envelope"):
            parent, child = context.Pipe(duplex=True)
            process = context.Process(
                target=_worker,
                args=(child, resolved_track, asdict(profile), role),
                name=f"identification_safety_{role}",
                daemon=True,
            )
            process.start()
            child.close()
            self._connections.append(parent)
            self._processes.append(process)
        self._in_flight = False
        self._sequence = 0
        for connection in self._connections:
            if not connection.poll(10.0):
                self.close()
                raise RuntimeError("safety predictor process startup timed out")
            ready = connection.recv()
            if not ready.get("ready", False):
                self.close()
                raise RuntimeError(
                    "safety predictor process startup failed: "
                    f"{ready.get('error', 'unknown')}"
                )

    @property
    def in_flight(self) -> bool:
        return self._in_flight

    def submit(
        self,
        x: float,
        y: float,
        yaw: float,
        vx: float,
        elapsed: float,
        s_guess: float,
        steering: float,
        future_excitation: float,
    ) -> bool:
        if self._in_flight:
            return False
        if not all(process.is_alive() for process in self._processes):
            raise RuntimeError("a safety predictor process is not alive")
        self._sequence += 1
        request = {
            "sequence": self._sequence,
            "submitted_monotonic": time.monotonic(),
            "x": float(x), "y": float(y), "yaw": float(yaw),
            "vx": float(vx), "elapsed": float(elapsed),
            "s_guess": float(s_guess), "steering": float(steering),
            "future_excitation": float(future_excitation),
        }
        for connection in self._connections:
            connection.send(request)
        self._in_flight = True
        return True

    def poll(self) -> Optional[SafetyPrediction]:
        if not all(connection.poll() for connection in self._connections):
            if self._in_flight and not all(
                process.is_alive() for process in self._processes
            ):
                raise RuntimeError("a safety predictor process exited unexpectedly")
            return None
        baseline = SafetyPrediction(**self._connections[0].recv())
        envelope = SafetyPrediction(**self._connections[1].recv())
        if baseline.sequence != envelope.sequence:
            raise RuntimeError("safety predictor branch sequence mismatch")
        errors = "; ".join(item for item in (baseline.error, envelope.error) if item)
        result = SafetyPrediction(
            sequence=baseline.sequence,
            submitted_monotonic=baseline.submitted_monotonic,
            baseline_margin=baseline.baseline_margin,
            excitation_margin=envelope.excitation_margin,
            evaluated_excitation_rad=envelope.evaluated_excitation_rad,
            compute_seconds=max(baseline.compute_seconds, envelope.compute_seconds),
            error=errors,
        )
        self._in_flight = False
        return result

    def wait(self, timeout: float) -> Optional[SafetyPrediction]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = self.poll()
            if result is not None:
                return result
            time.sleep(0.005)
        return None

    def close(self) -> None:
        try:
            for connection, process in zip(self._connections, self._processes):
                if process.is_alive():
                    connection.send(None)
            for process in self._processes:
                process.join(timeout=1.0)
            for process in self._processes:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1.0)
        except (BrokenPipeError, EOFError, OSError):
            pass
        finally:
            for connection in self._connections:
                connection.close()
