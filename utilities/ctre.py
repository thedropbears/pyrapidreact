from typing import Optional

import ctre

# Counts per revolution for the Falcon 500 integrated sensor.
FALCON_CPR = 2048
# Freespeed in rev/s
FALCON_FREE_RPS = 100

VERSA_ENCODER_CPR = 4096


class TalonEncoder:
    """
    Wrapper around a Talon selected feedback sensor, mimicking the Spark Max API.

    Note that this does not change the behaviour of closed-loop set calls.
    """

    __slots__ = (
        "talon",
        "pid_idx",
        "counts_per_rev",
        "pos_factor",
        "vel_factor",
    )

    def __init__(
        self,
        talon: ctre.BaseTalon,
        countsPerRev: Optional[int] = None,
        *,
        unitsPerRev: float = 1,
        pidIdx: int = 0,
    ) -> None:
        """
        Args:
            talon: The motor controller the encoder is attached to.
            countsPerRev: Assumes integrated sensor for Talon FX, otherwise 4096.
            pidIdx: Which PID loop to use. 0 for primary PID loop, 1 for auxilliary PID loop.
        """
        if countsPerRev is None:
            countsPerRev = FALCON_CPR if isinstance(talon, ctre.TalonFX) else 4096

        self.talon = talon
        self.pid_idx = pidIdx
        self.counts_per_rev = countsPerRev
        self.setPositionConversionFactor(unitsPerRev)

    def setPositionConversionFactor(self, units_per_rev: float) -> None:
        """
        Set the conversion factor from rotations to desired units.

        Note: Differs from Spark Max as this affects getVelocity too.
        """
        self.pos_factor = units_per_rev / self.counts_per_rev
        self.vel_factor = self.pos_factor * 10

    def getPosition(self) -> float:
        return self.talon.getSelectedSensorPosition(self.pid_idx) * self.pos_factor

    def getVelocity(self) -> float:
        """Get the velocity in units per second."""
        return self.talon.getSelectedSensorVelocity(self.pid_idx) * self.vel_factor

    def setPosition(self, position: float) -> ctre.ErrorCode:
        return self.talon.setSelectedSensorPosition(
            position / self.pos_factor,
            self.pid_idx,
        )

    def getCountsPerRevolution(self) -> int:
        return self.counts_per_rev
