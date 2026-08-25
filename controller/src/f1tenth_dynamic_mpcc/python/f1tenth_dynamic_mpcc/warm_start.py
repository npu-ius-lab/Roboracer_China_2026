"""Warm-start trajectory management for receding-horizon control."""

from __future__ import annotations

import numpy as np


class WarmStart:
    def __init__(self, horizon_steps: int, nx: int = 9, nu: int = 3):
        if horizon_steps < 2:
            raise ValueError("horizon_steps must be at least two")
        self.N = int(horizon_steps)
        self.nx = int(nx)
        self.nu = int(nu)
        self.states = np.zeros((self.N + 1, self.nx), dtype=float)
        self.controls = np.zeros((self.N, self.nu), dtype=float)
        self.valid = False

    def reset(self, initial_state: np.ndarray, control_seed: np.ndarray | None = None) -> None:
        initial_state = np.asarray(initial_state, dtype=float)
        if initial_state.shape != (self.nx,):
            raise ValueError("invalid initial state shape")
        self.states[:] = initial_state
        if control_seed is None:
            self.controls[:] = 0.0
        else:
            control_seed = np.asarray(control_seed, dtype=float)
            if control_seed.shape != (self.nu,):
                raise ValueError("invalid control seed shape")
            self.controls[:] = control_seed
        self.valid = True

    def shift(self, states: np.ndarray, controls: np.ndarray) -> None:
        states = np.asarray(states, dtype=float)
        controls = np.asarray(controls, dtype=float)
        if states.shape != self.states.shape or controls.shape != self.controls.shape:
            raise ValueError("solver trajectory shape mismatch")
        self.states[:-1] = states[1:]
        self.states[-1] = states[-1]
        self.controls[:-1] = controls[1:]
        self.controls[-1] = controls[-1]
        self.valid = True

    def theta_sequence(self) -> np.ndarray:
        return self.states[:, -1].copy()
