import wpilib
import commands2
from commands2 import WaitCommand, cmd
from commands2.button import CommandXboxController
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_CANdleLight import SS_CANdleLight


def SEQ_shoot(shooter: SS_Kraken, feeder: SS_Kraken):
    return commands2.SequentialCommandGroup(
        shooter.spin_up_and_wait_command(),
        cmd.runOnce(shooter.run_velocity_at_setpoint),  # Ensure shooter keeps spinning at setpoint
        cmd.startEnd(feeder.run_velocity_at_setpoint, feeder.stop_motor, feeder).withTimeout(3.0),
        cmd.runOnce(shooter.stop_motor),
    )

def SEQ_extend_intake(extender: SS_Kraken):
    return commands2.SequentialCommandGroup(
        cmd.runOnce(lambda: extender.set_position(1.5)).withTimeout(2.0),
        cmd.runOnce(extender.stop_motor)
    )

def SEQ_auto_shake_intake(swerve: SS_SwerveDrive):
    return commands2.SequentialCommandGroup(
        swerve.padlocked_drive_request_command(vx_requested=0.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(2.0),
        swerve.padlocked_drive_request_command(vx_requested=-1.5, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(1.0),
        swerve.padlocked_drive_request_command(vx_requested=2.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.2),
        swerve.padlocked_drive_request_command(vx_requested=-0.0, vy_requested=0.0, x_vector=1.0, y_vector=0.0).withTimeout(0.5),
    )

def CMD_deploy_intake(extender: SS_Kraken, shooter: SS_Kraken):
    return commands2.ParallelCommandGroup(
        cmd.runOnce(lambda: extender.set_position(3)).withTimeout(2.0),
        cmd.runOnce(shooter.run_voltage_percent_reverse).withTimeout(2.0)
    )

class CMD_ComboShoot(commands2.Command):
    def __init__(self, ss_shooter: SS_Kraken, ss_feeder: SS_Kraken, joystick: CommandXboxController):
        super().__init__()
        self.ss_shooter = ss_shooter
        self.ss_feeder = ss_feeder
        self.addRequirements(self.ss_shooter, self.ss_feeder) # Ensure no other command these subsystems while this command is running
        self.timer = wpilib.Timer()
        self.velocity_tolerance = 500 # RPM tolerance for considering the shooter "up to speed"
        self._joystick = joystick

    def initialize(self):
        self.timer.restart()
        self.ss_shooter.run_velocity_at_setpoint() # Spin up shooter

    def execute(self):
            if abs(self.ss_shooter.velocity_actual - self.ss_shooter.velocity_setpoint) < self.velocity_tolerance:
                self.ss_feeder.run_velocity_at_setpoint() # Spin feeder
            else:
                self.ss_feeder.stop_motor() # Stop feeder if shooter is no longer at speed

    def isFinished(self):
        return not self._joystick.rightBumper().getAsBoolean() # Run until the right bumper is released

    def end(self, interrupted):
        self.ss_shooter.stop_motor()
        self.ss_feeder.stop_motor()
