#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import typing

import ctre

from pyfrc.physics.core import PhysicsInterface  # type: ignore

from utilities.ctre import FALCON_CPR

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


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot objet
        """

        self.physics_controller = physics_controller

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

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain

        for wheel in self.wheels:
            wheel.update(tm_diff)
        for steer in self.steer:
            steer.update(tm_diff)

        # transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        # pose = self.physics_controller.move_robot(transform)
