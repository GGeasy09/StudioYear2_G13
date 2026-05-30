"""
kalman.py — Kalman Filter Library for FRA263 / FRA264 Base System
=================================================================
Provides a 1-D kinematic Kalman filter with state vector:
    x = [position, velocity, acceleration]

Raw Modbus values from registers 0x28 / 0x29 / 0x30 are signed int16
scaled by ×10 on the robot side.  Call `decode_raw(raw)` first, or use
the convenience helpers `update_from_registers()` and
`calculate_acceleration()`.

Usage example
-------------
    from kalman import KalmanFilter1D, decode_raw

    kf = KalmanFilter1D(dt=0.02)            # 50 Hz loop → dt = 0.02 s

    # Inside your Modbus read loop:
    raw_pos  = read_register(0x28)           # signed int16
    raw_vel  = read_register(0x29)
    raw_accel = read_register(0x30)

    pos  = decode_raw(raw_pos)              # real units (÷10)
    vel  = decode_raw(raw_vel)
    accel_measured = decode_raw(raw_accel)

    kf.update(pos, vel, accel_measured)
    smooth_accel = kf.calculate_acceleration()

    print(f"Filtered accel: {smooth_accel:.2f}")
"""

import numpy as np


# ---------------------------------------------------------------------------
# Helper: two's-complement decode + ×10 scale
# ---------------------------------------------------------------------------

def decode_raw(raw: int) -> float:
    """
    Convert a raw 16-bit Modbus register value to real engineering units.

    The robot firmware scales position / velocity / acceleration by ×10
    before writing to registers 0x28, 0x29, 0x30.  Negative values are
    stored as 16-bit two's complement (0 … 65535 range on the wire).

    Parameters
    ----------
    raw : int
        Unsigned 16-bit integer read from the Modbus register (0–65535).

    Returns
    -------
    float
        Real value in engineering units (degrees, deg/s, deg/s²).

    Examples
    --------
    >>> decode_raw(1234)
    123.4
    >>> decode_raw(65413)   # two's complement of -123
    -12.3
    """
    # Two's complement: values ≥ 32768 are negative
    if raw >= 32768:
        raw -= 65536
    return raw / 10.0


# ---------------------------------------------------------------------------
# Core Kalman filter
# ---------------------------------------------------------------------------

class KalmanFilter1D:
    """
    Discrete-time Kalman filter for 1-D kinematic motion.

    State vector  x = [position, velocity, acceleration]ᵀ

    The constant-acceleration model is used for prediction:
        pos(k+1)   = pos(k) + vel(k)·dt + ½·accel(k)·dt²
        vel(k+1)   = vel(k) + accel(k)·dt
        accel(k+1) = accel(k)           (assumed constant between steps)

    Parameters
    ----------
    dt : float
        Time step in seconds (e.g. 0.02 for 50 Hz).
    process_noise_std : float
        Standard deviation of the process noise (model uncertainty).
        Increase if the robot's acceleration changes rapidly.
    pos_noise_std : float
        Measurement noise std for position (register 0x28).
    vel_noise_std : float
        Measurement noise std for velocity (register 0x29).
    accel_noise_std : float
        Measurement noise std for acceleration (register 0x30).
    """

    def __init__(
        self,
        dt: float = 0.02,
        process_noise_std: float = 1.0,
        pos_noise_std: float = 0.5,
        vel_noise_std: float = 1.0,
        accel_noise_std: float = 2.0,
    ):
        self.dt = dt

        # ── State transition matrix F ────────────────────────────────────
        # x(k+1) = F · x(k)
        self.F = np.array([
            [1.0,  dt,  0.5 * dt**2],
            [0.0, 1.0,           dt],
            [0.0, 0.0,          1.0],
        ])

        # ── Measurement matrix H ─────────────────────────────────────────
        # We observe all three states directly from Modbus registers
        self.H = np.eye(3)

        # ── Process noise covariance Q ───────────────────────────────────
        q = process_noise_std ** 2
        # Higher-order terms have more uncertainty
        self.Q = q * np.array([
            [dt**4 / 4, dt**3 / 2, dt**2 / 2],
            [dt**3 / 2,    dt**2,         dt],
            [dt**2 / 2,       dt,        1.0],
        ])

        # ── Measurement noise covariance R ───────────────────────────────
        self.R = np.diag([
            pos_noise_std   ** 2,
            vel_noise_std   ** 2,
            accel_noise_std ** 2,
        ])

        # ── Initial state and covariance ─────────────────────────────────
        self.x = np.zeros(3)          # [pos, vel, accel]
        self.P = np.eye(3) * 1000.0   # large initial uncertainty
        self._initialized = False

    # ------------------------------------------------------------------ #
    #  Public API                                                          #
    # ------------------------------------------------------------------ #

    def update(self, pos: float, vel: float, accel: float) -> np.ndarray:
        """
        Run one predict → update cycle with a full measurement triplet.

        Call this once per Modbus read cycle with the decoded (÷10) values
        from registers 0x28, 0x29, and 0x30.

        Parameters
        ----------
        pos   : float  — decoded position   (register 0x28 ÷ 10)
        vel   : float  — decoded velocity   (register 0x29 ÷ 10)
        accel : float  — decoded accel      (register 0x30 ÷ 10)

        Returns
        -------
        np.ndarray, shape (3,)
            Filtered state [position, velocity, acceleration].
        """
        z = np.array([pos, vel, accel])

        # Seed the filter on the first real measurement
        if not self._initialized:
            self.x = z.copy()
            self._initialized = True
            return self.x.copy()

        # Predict step
        x_pred = self.F @ self.x
        P_pred = self.F @ self.P @ self.F.T + self.Q

        # Update step (Kalman gain)
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        innovation = z - self.H @ x_pred
        self.x = x_pred + K @ innovation
        self.P = (np.eye(3) - K @ self.H) @ P_pred

        return self.x.copy()

    def update_from_registers(
        self,
        raw_pos: int,
        raw_vel: int,
        raw_accel: int,
    ) -> np.ndarray:
        """
        Convenience wrapper: decode raw Modbus register values then filter.

        Parameters
        ----------
        raw_pos   : int  — raw uint16 from register 0x28
        raw_vel   : int  — raw uint16 from register 0x29
        raw_accel : int  — raw uint16 from register 0x30

        Returns
        -------
        np.ndarray, shape (3,)
            Filtered state [position, velocity, acceleration].
        """
        pos   = decode_raw(raw_pos)
        vel   = decode_raw(raw_vel)
        accel = decode_raw(raw_accel)
        return self.update(pos, vel, accel)

    def calculate_acceleration(self) -> float:
        """
        Return the current filtered acceleration estimate.

        Must be called after at least one `update()` or
        `update_from_registers()` call.

        Returns
        -------
        float
            Kalman-filtered acceleration in engineering units (deg/s²).

        Raises
        ------
        RuntimeError
            If called before any measurement has been provided.

        Example
        -------
            kf = KalmanFilter1D(dt=0.02)
            kf.update(pos=12.3, vel=1.1, accel=0.5)
            a = kf.calculate_acceleration()
        """
        if not self._initialized:
            raise RuntimeError(
                "KalmanFilter1D: call update() at least once before "
                "calculate_acceleration()."
            )
        return float(self.x[2])

    def calculate_acceleration_from_registers(
        self,
        raw_pos: int,
        raw_vel: int,
        raw_accel: int,
    ) -> float:
        """
        One-shot helper: decode registers, filter, return filtered accel.

        Parameters
        ----------
        raw_pos   : int  — raw uint16 from register 0x28
        raw_vel   : int  — raw uint16 from register 0x29
        raw_accel : int  — raw uint16 from register 0x30

        Returns
        -------
        float
            Kalman-filtered acceleration in engineering units (deg/s²).
        """
        self.update_from_registers(raw_pos, raw_vel, raw_accel)
        return self.calculate_acceleration()

    # ── Read-only properties for the full state ──────────────────────── #

    @property
    def position(self) -> float:
        """Filtered position (deg)."""
        return float(self.x[0])

    @property
    def velocity(self) -> float:
        """Filtered velocity (deg/s)."""
        return float(self.x[1])

    @property
    def acceleration(self) -> float:
        """Filtered acceleration (deg/s²) — same as calculate_acceleration()."""
        return float(self.x[2])

    @property
    def state(self) -> dict:
        """Full filtered state as a plain dict."""
        return {
            "position":     self.position,
            "velocity":     self.velocity,
            "acceleration": self.acceleration,
        }

    def reset(self):
        """Reset the filter to its initial (uninitialized) state."""
        self.x = np.zeros(3)
        self.P = np.eye(3) * 1000.0
        self._initialized = False


# ---------------------------------------------------------------------------
# Acceleration-only variant (lighter weight)
# ---------------------------------------------------------------------------

class AccelerationEstimator:
    """
    Lightweight scalar Kalman filter that tracks **only acceleration**.

    Use this when you only need a smoothed acceleration value and don't
    need the full kinematic state.  It models acceleration as a random
    walk with Gaussian process noise.

    Parameters
    ----------
    process_noise_std : float
        How quickly you expect acceleration to change between steps.
    measure_noise_std : float
        Noise level of the raw acceleration reading from register 0x30.
    """

    def __init__(
        self,
        process_noise_std: float = 2.0,
        measure_noise_std: float = 3.0,
    ):
        self._x = 0.0           # state estimate
        self._p = 1000.0        # error covariance
        self._q = process_noise_std ** 2
        self._r = measure_noise_std ** 2
        self._initialized = False

    def update(self, accel_measured: float) -> float:
        """
        Filter a single decoded acceleration measurement.

        Parameters
        ----------
        accel_measured : float
            Decoded acceleration value (raw ÷ 10) from register 0x30.

        Returns
        -------
        float
            Filtered acceleration estimate.
        """
        if not self._initialized:
            self._x = accel_measured
            self._initialized = True
            return self._x

        # Predict (random-walk model: x stays the same, P grows)
        p_pred = self._p + self._q

        # Update
        k = p_pred / (p_pred + self._r)          # Kalman gain
        self._x = self._x + k * (accel_measured - self._x)
        self._p = (1.0 - k) * p_pred

        return self._x

    def update_from_register(self, raw_accel: int) -> float:
        """
        Decode raw register 0x30 and return filtered acceleration.

        Parameters
        ----------
        raw_accel : int
            Raw uint16 from Modbus register 0x30.

        Returns
        -------
        float
            Filtered acceleration (deg/s²).
        """
        return self.update(decode_raw(raw_accel))

    def calculate_acceleration(self) -> float:
        """
        Return the current filtered acceleration estimate.

        Returns
        -------
        float
            Filtered acceleration in engineering units (deg/s²).
        """
        if not self._initialized:
            raise RuntimeError(
                "AccelerationEstimator: call update() at least once first."
            )
        return self._x

    def reset(self):
        """Reset filter state."""
        self._x = 0.0
        self._p = 1000.0
        self._initialized = False
