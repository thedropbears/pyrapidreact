import math

from pyfrc.physics.core import PhysicsInterface  # type: ignore

import ctre
import navx
from utilities.ctre import FALCON_CPR, FALCON_FREE_RPS

from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState

from components.chassis import SwerveModule
from components.turret import Turret

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        self.turret: ctre.WPI_TalonSRX = robot.turret_motor

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: list[SwerveModule] = robot.chassis.modules

        self.imu: navx.AHRS = robot.imu

    def update_sim(self, now: float, tm_diff: float) -> None:
        # turret
        sign = 1
        if self.turret.getInverted():
            sign = -1
        turret_speed = sign * int(
            0.5 * self.turret.getMotorOutputPercent() * (1023.0 / Turret.pidF)
        )
        self.turret.getSimCollection().setQuadratureVelocity(turret_speed)
        self.turret.getSimCollection().addQuadraturePosition(
            int(turret_speed * 10 * tm_diff)
        )

        for module in self.swerve_modules:
            # Drive
            motor = module.drive
            drive_max_speed_counts_per_100ms = (
                SwerveModule.METRES_TO_DRIVE_COUNTS * 4.0 / 10
            )
            sign = 1
            if motor.getInverted():
                sign = -1
            velocity = sign * int(
                drive_max_speed_counts_per_100ms * motor.getMotorOutputPercent()
            )  # counts per 100ms
            motor.getSimCollection().setIntegratedSensorVelocity(velocity)
            motor.getSimCollection().addIntegratedSensorPosition(
                int(velocity * 10 * tm_diff)
            )

            # Steer
            motor = module.steer
            load_factor = 0.20  # cannot run at free speed
            sign = 1
            if motor.getInverted():
                sign = -1
            velocity = sign * int(
                load_factor
                * FALCON_CPR
                * motor.getMotorOutputPercent()
                * FALCON_FREE_RPS
                / 10
            )  # counts per 100ms
            motor.getSimCollection().setIntegratedSensorVelocity(velocity)
            motor.getSimCollection().addIntegratedSensorPosition(
                int(velocity * 10 * tm_diff)
            )

        states = typing.cast(
            typing.Tuple[
                SwerveModuleState,
                SwerveModuleState,
                SwerveModuleState,
                SwerveModuleState,
            ],
            tuple([module.get() for module in self.swerve_modules]),
        )
        speeds = self.kinematics.toChassisSpeeds(states)

        self.imu.setAngleAdjustment(
            self.imu.getAngle() - math.degrees(speeds.omega * tm_diff)
        )

        self.physics_controller.drive(speeds, tm_diff)
