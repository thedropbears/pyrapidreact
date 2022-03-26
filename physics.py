import math

from pyfrc.physics.core import PhysicsInterface  # type: ignore

import ctre
from utilities.ctre import FALCON_CPR, VERSA_ENCODER_CPR

import wpilib.simulation
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState

from components.chassis import SwerveModule
from components.turret import Turret

import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class SimpleTalonFXMotorSim:
    def __init__(self, motor: ctre.TalonFX, kV: float, rev_per_unit: float) -> None:
        self.sim_collection = motor.getSimCollection()
        self.kV = kV  # volt seconds per unit
        self.rev_per_unit = rev_per_unit

    def update(self, dt: float) -> None:
        voltage = self.sim_collection.getMotorOutputLeadVoltage()
        velocity = voltage / self.kV  # units per second
        self.sim_collection.setIntegratedSensorVelocity(
            int(velocity * self.rev_per_unit * FALCON_CPR / 10)
        )
        self.sim_collection.addIntegratedSensorPosition(
            int(velocity * self.rev_per_unit * FALCON_CPR * dt)
        )


class SimpleTalonSRXMotorSim:
    def __init__(self, motor: ctre.TalonSRX, kV: float, rev_per_unit: float) -> None:
        self.sim_collection = motor.getSimCollection()
        self.kV = kV  # volt seconds per unit
        self.rev_per_unit = rev_per_unit

    def update(self, dt: float) -> None:
        voltage = self.sim_collection.getMotorOutputLeadVoltage()
        velocity = voltage / self.kV  # units per second
        self.sim_collection.setQuadratureVelocity(
            int(velocity * self.rev_per_unit * VERSA_ENCODER_CPR / 10)
        )
        self.sim_collection.addQuadraturePosition(
            int(velocity * self.rev_per_unit * VERSA_ENCODER_CPR * dt)
        )


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        turret_kv = (
            100000
            * 1.0
            / (
                1023
                * 12.0
                * Turret.COUNTS_PER_MOTOR_REV
                / Turret.COUNTS_PER_TURRET_RADIAN
                / Turret.pidF
            )
        )
        self.turret = SimpleTalonSRXMotorSim(
            robot.turret_motor, turret_kv, 1.0 / math.tau
        )

        self.kinematics: SwerveDrive4Kinematics = robot.chassis.kinematics
        self.swerve_modules: list[SwerveModule] = robot.chassis.modules

        # Motors
        self.wheels = [
            SimpleTalonFXMotorSim(
                module.drive, module.drive_ff.kV, 1 / module.DRIVE_MOTOR_REV_TO_METRES
            )
            for module in robot.chassis.modules
        ]
        self.steer = [
            SimpleTalonFXMotorSim(
                module.steer,
                kV=1,  # TODO: get from sysid logs
                rev_per_unit=1 / module.STEER_MOTOR_REV_TO_RAD,
            )
            for module in robot.chassis.modules
        ]

        self.imu = wpilib.simulation.SimDeviceSim("navX-Sensor", 4)
        self.imu_yaw = self.imu.getDouble("Yaw")

    def update_sim(self, now: float, tm_diff: float) -> None:
        # turret
        self.turret.update(tm_diff)

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

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

        self.imu_yaw.set(self.imu_yaw.get() - math.degrees(speeds.omega * tm_diff))

        self.physics_controller.drive(speeds, tm_diff)
