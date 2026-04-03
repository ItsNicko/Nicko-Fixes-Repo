import wpilib
import commands2
from commands2 import cmd
from wpilib import Color8Bit, SmartDashboard, Timer, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6 import HootAutoReplay
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathConstraints
from commands2.button import CommandXboxController
from generated.tuner_constants_2026_GF import TunerConstants
from subsystems.SS_SwerveDrive import SS_SwerveDrive
from subsystems.SS_Kraken import SS_Kraken
from subsystems.SS_CANdleLight import SS_CANdleLight
from subsystems.SS_CameraPose import SS_CameraPose
from commands.complex_and_sequences import CMD_ComboShoot, SEQ_shoot, SEQ_extend_intake

class RobotContainer:
    def __init__(self) -> None:
        self.gamepad = None or CommandXboxController(0)
        self.operator = None or CommandXboxController(1)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.canbus = TunerConstants.canbus
        self.ss_shooter = None or SS_Kraken(3, self.canbus, "Shooter", inverted=True, max_rps=100, velocity_setpoint=40, kp=0.08, ki=0.0, kd=0.0, kv=0.012, ks=0.0)
        self.ss_feeder = None or SS_Kraken(1, self.canbus, "Feeder", kp=1.0, velocity_setpoint=40, percent_power_setpoint=0.5)
        self.ss_intake = None or SS_Kraken(4, self.canbus, "Intake", max_rps=120, percent_power_setpoint=0.5)
        self.ss_extend = None or SS_Kraken(6, self.canbus, "Extension", inverted=True, brake_mode=True, kp=5, ki=0.5, Vmax=.5, Amax=.5, Jerk=2.5)
        self.ss_candle_light_left = None or SS_CANdleLight(2, self.canbus, "Left")
        self.ss_candle_light_right = None or SS_CANdleLight(5, self.canbus, "Right")
        self.ss_swerve_drive = None or SS_SwerveDrive(self.gamepad)
        self.ss_camera_pose = None# or SS_CameraPose(self.ss_swerve_drive)

        self._build_complex_commands_and_autochooser()
        self._setup_simulated_mechanism2d()

        if self.gamepad: self.configure_gamepad_bindings()

    def configure_gamepad_bindings(self):
        if self.ss_shooter and self.ss_feeder:
            self.gamepad.rightBumper().whileTrue(CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.gamepad))
        elif self.ss_shooter:
            self.gamepad.rightBumper().whileTrue(cmd.startEnd(self.ss_shooter.run_velocity_at_setpoint, self.ss_shooter.stop_motor, self.ss_shooter))
        if self.ss_feeder:
            self.gamepad.leftBumper().whileTrue(cmd.startEnd(self.ss_feeder.run_velocity_at_setpoint, self.ss_feeder.stop_motor, self.ss_feeder))
        if self.ss_intake:
            self.gamepad.leftTrigger(threshold=.2).whileTrue(commands2.cmd.startEnd(self.ss_intake.run_voltage_percent_reverse, self.ss_intake.stop_motor, self.ss_intake))
            self.gamepad.rightTrigger(threshold=.2).whileTrue(commands2.cmd.startEnd(self.ss_intake.run_voltage_percent_forward, self.ss_intake.stop_motor, self.ss_intake))
            self.gamepad.y().whileTrue(cmd.startEnd(lambda: self.ss_extend.set_position(3), self.ss_extend.stop_motor, self.ss_intake))
        if self.ss_shooter and self.ss_feeder:
            self.gamepad.x().onTrue(SEQ_shoot(self.ss_shooter, self.ss_feeder))

        if self.ss_swerve_drive:
            ## Set starting drive mode to field-centered, and allow toggling to padlocked with the A button
            self.ss_swerve_drive.drive_mode_field_centered()
            self.gamepad.a().onTrue(cmd.runOnce(self.ss_swerve_drive.drive_mode_padlocked))
            self.gamepad.a().onFalse(cmd.runOnce(self.ss_swerve_drive.drive_mode_field_centered))
            # self.gamepad.a().and_(self.gamepad.back()).onFalse(cmd.runOnce(self.ss_swerve_drive.change_target))
            # self.gamepad.pov(45).whileTrue(self.ss_swerve_drive.robot_pov_drive_request_command(1, 0))
            # self.gamepad.pov(135).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.robot_pov_drive_request_command(-1, 0), lambda: self.ss_swerve_drive.robot_pov_drive_request_command(0, 0)) )
            # self.gamepad.pov(225).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.robot_pov_drive_request_command(0, 1), lambda: self.ss_swerve_drive.robot_pov_drive_request_command(0, 0)) )
            # self.gamepad.pov(315).whileTrue(cmd.startEnd(lambda: self.ss_swerve_drive.robot_pov_drive_request_command(0, -1), lambda: self.ss_swerve_drive.robot_pov_drive_request_command(0, 0)) )
            self.gamepad.back().and_(self.gamepad.b()).whileTrue(cmd.runOnce(self.ss_swerve_drive.brake))
            self.gamepad.back().and_(self.gamepad.start()).onTrue(cmd.runOnce(self.ss_swerve_drive.reset_field_oriented_perspective))

    def _build_complex_commands_and_autochooser(self):
        self.cmd_combo_shoot = CMD_ComboShoot(self.ss_shooter, self.ss_feeder, self.gamepad)
        NamedCommands.registerCommand("Combo Shoot", self.cmd_combo_shoot)
        self.seq_shoot = SEQ_shoot(self.ss_shooter, self.ss_feeder)
        NamedCommands.registerCommand("SEQ Shoot", self.seq_shoot)
        self.deploy_intake = cmd.runOnce(lambda: self.ss_extend.set_position(1.5), self.ss_extend)
        NamedCommands.registerCommand("Deploy_Intake", self.deploy_intake)
        self.run_intake = cmd.startEnd(self.ss_intake.run_voltage_percent_forward, self.ss_intake.stop_motor, self.ss_intake)
        NamedCommands.registerCommand("Run_Intake", self.run_intake)
        self.auto_chooser = AutoBuilder.buildAutoChooser("None") # must be defined after SS's and all registered commands
        SmartDashboard.putData("Auto Chooser", self.auto_chooser)

    def _setup_simulated_mechanism2d(self):
        self.mech2d = wpilib.Mechanism2d(10, 10)  # Width, Height
        self.root2d = self.mech2d.getRoot("root", 5, 2)
        if self.ss_intake:
            self.intake2d = self.root2d.appendLigament("intake", 4, 135, 6, Color8Bit(0, 0, 255))
        if self.ss_feeder:
            self.feeder2d = self.root2d.appendLigament("feeder", 4, 90, 6, Color8Bit(0, 255, 0))
        if self.ss_shooter:
            self.shooter2d = self.root2d.appendLigament("shooter", 4, -135, 6, Color8Bit(255, 0, 0))
        if self.ss_extend:
            self.extend2d = self.root2d.appendLigament("extend", 2, 90, 3, Color8Bit(255, 255, 255))
        wpilib.SmartDashboard.putData("Mechanism", self.mech2d)

    def getAutonomousCommand(self) -> commands2.Command:
        return self.auto_chooser.getSelected()


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.

        Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        autonomous chooser on the dashboard.
        """
        self.autonomousCommand = None
        self.container = RobotContainer()
        self.localMatchTimer = Timer()
        # self._time_and_driver_replay = (HootAutoReplay().with_timestamp_replay().with_driver_replay() )
        if self.container.ss_candle_light_right:
            self.container.ss_candle_light_right.set_all_leds_RGBW(0, 255, 0) # Set front CANdle to green
        if self.container.ss_candle_light_left:
            self.container.ss_candle_light_left.set_all_leds_RGBW(255, 165, 0) # Set rear CANdle to orange

    def robotPeriodic(self) -> None:
        """
        This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating.

        Starts by running the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        commands, running already-scheduled commands, removing finished or interrupted commands,
        and running subsystem periodic() methods.  This must be called from the robot's periodic
        block in order for anything in the Command-based framework to work.
        """
        # self._time_and_driver_replay.update() # using HootAutoReplay to log and replay timestamp and driver data
        match_time_from_driver_station = Timer.getMatchTime()
        SmartDashboard.putNumber("Match Time", self.localMatchTimer.get() if match_time_from_driver_station < 0 else match_time_from_driver_station)
        if self.container.ss_intake:
            self.container.intake2d.setLength(self.container.ss_intake.velocity_actual/10)
        if self.container.ss_feeder:
            self.container.feeder2d.setLength(self.container.ss_feeder.velocity_actual/10)
        if self.container.ss_shooter:
            self.container.shooter2d.setLength(self.container.ss_shooter.velocity_actual/10)
        if self.container.ss_extend:
            self.container.extend2d.setAngle(90 - self.container.ss_extend.position_actual*90/2)

    def teleopInit(self) -> None:
        """
        This makes sure that the autonomous stops running when
        teleop starts running. If you want the autonomous to
        continue until interrupted by another command, remove
        this line or comment it out. 
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            if self.container.ss_candle_light_right:
                self.container.ss_candle_light_right.set_all_leds_RGBW(255, 0, 0)
            if self.container.ss_candle_light_left:
                self.container.ss_candle_light_left.set_all_leds_RGBW(255, 0, 0)
        elif DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            if self.container.ss_candle_light_right:
                self.container.ss_candle_light_right.set_all_leds_RGBW(0, 0, 255)
            if self.container.ss_candle_light_left:
                self.container.ss_candle_light_left.set_all_leds_RGBW(0, 0, 255)

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        pass


    def autonomousInit(self) -> None:
        """
        This function is run once when the robot enters autonomous mode.
        Gets the selected autonomous command from the chooser and schedules it.
        """
        self.localMatchTimer.reset()
        self.localMatchTimer.start()
        self.autonomousCommand = self.container.getAutonomousCommand()
        
        if not self.autonomousCommand:
            print("No autonomous command selected.")
            return

        self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        pass


    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        pass

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        pass


    def testInit(self) -> None:
        """Cancels all running commands at the start of test mode"""
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.ss_swerve_drive.drive_mode_padlocked()
        ''' # Example test code for manually testing subsystems during test mode, using the mechanism2d for visualization. Uncomment and modify as needed for testing.
        if self.container.ss_shooter:
            self.container.ss_shooter.velocity_setpoint = 50
            self.container.ss_shooter.run_velocity_at_setpoint()
        if self.container.ss_feeder:
            self.container.ss_feeder.percent_power_setpoint = .5
            self.container.ss_feeder.run_voltage_percent_forward()
        if self.container.ss_intake:
            self.container.ss_intake.percent_power_setpoint = .5
            self.container.ss_intake.run_voltage_percent_forward()'''

    def testPeriodic(self) -> None:
        """This function is called periodically during test mode"""
        pass

    def testExit(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
