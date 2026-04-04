import commands2
import wpilib
from wpimath.units import rotationsToRadians
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from phoenix6 import swerve, SignalLogger
from wpimath.kinematics import ChassisSpeeds
from telemetry import Telemetry
from generated.tuner_constants_2026_GF import TunerConstants
# from generated.tuner_constants_2025_old import TunerConstants
from wpilib import DriverStation, Timer, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import DriverStation

class SS_SwerveDrive(commands2.Subsystem):
    def __init__(self, joystick) -> None:
        self._joystick = joystick
        self._max_angular_rate = rotationsToRadians(0.75)
        self._base_speed_factor = 1.00
        self._boost_speed_factor = 1.00
        self._max_speed_factor = self._base_speed_factor
        self._full_throttle_hold_seconds = 0.0
        self._full_throttle_threshold = 0.75
        self._full_throttle_ramp_rate = 1.20
        self._last_periodic_timestamp = Timer.getFPGATimestamp()
        self._left_x_limiter = SlewRateLimiter(3.5)
        self._left_y_limiter = SlewRateLimiter(3.5)
        self._right_x_limiter = SlewRateLimiter(6.0)
        self._right_y_limiter = SlewRateLimiter(6.0)
        self._last_heading = Rotation2d()
        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts
        self._drive_deadband = 0.03 * TunerConstants.speed_at_12_volts
        self._rot_deadband = 0.03 * self._max_angular_rate
        wpilib.SmartDashboard.putNumber("Swerve/Swerve Max Speed Factor", self._max_speed_factor)
        self._pov_speed = 0.2
        self._latest_pose = Pose2d()
        self.drivetrain = TunerConstants.create_drivetrain() # does this need to after swerve configs?
        # self._logger = Telemetry(self._max_speed)
        # self.drivetrain.register_telemetry( lambda state: self._logger.telemeterize(state) )
        self.x_vector_to_target = 0.0
        self.y_vector_to_target = 0.0
        self.range_to_target = 0.0
        self.target_x = 0.0
        self.target_y = 0.0

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        self.PIDF_sysID_tuning_bindings()
        idle = swerve.requests.Idle() # Determine behavior when no other commands are running. 
        Trigger(DriverStation.isDisabled).whileTrue( # This is important to prevent unexpected robot movement when commands end.
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True) )

        # Initialize swerve drive configurations
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._drive_deadband)
            .with_rotational_deadband(self._rot_deadband)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) )
        self._drive_facing_direction = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._drive_deadband)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) )

        self._setup_padlock_target_chooser()
        self._setup_pathplanner_auto_builder()
        
    def periodic(self) -> None:
        pose = self.drivetrain.sample_pose_at(Timer.getFPGATimestamp())
        if pose is not None:
            self._latest_pose = pose
            self.target_x, self.target_y = self._determine_padlock_target(pose)
            self.x_vector_to_target = self.target_x - self._latest_pose.translation().X()
            self.y_vector_to_target = self.target_y - self._latest_pose.translation().Y()
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                self.x_vector_to_target = -self.x_vector_to_target
                self.y_vector_to_target = -self.y_vector_to_target
            self.range_to_target = (self.x_vector_to_target**2 + self.y_vector_to_target**2)**0.5

        now = Timer.getFPGATimestamp()
        dt = max(0.0, now - self._last_periodic_timestamp)
        self._last_periodic_timestamp = now
        left_x = applyDeadband(self._joystick.getLeftX(), 0.08)
        left_y = applyDeadband(self._joystick.getLeftY(), 0.08)
        left_mag = (left_x * left_x + left_y * left_y) ** 0.5
        if left_mag >= self._full_throttle_threshold:
            self._full_throttle_hold_seconds += dt
        else:
            self._full_throttle_hold_seconds = max(0.0, self._full_throttle_hold_seconds - (2.0 * dt))

        target_speed_factor = min(
            self._boost_speed_factor,
            self._base_speed_factor + (self._full_throttle_hold_seconds * self._full_throttle_ramp_rate),
        )
        dashboard_max_speed = wpilib.SmartDashboard.getNumber("Swerve/Swerve Max Speed Factor", target_speed_factor)
        self._max_speed_factor = max(min(dashboard_max_speed, 1.0), 0.0) if dashboard_max_speed != target_speed_factor else target_speed_factor
        wpilib.SmartDashboard.putNumber("Swerve/Target X Vector", self.x_vector_to_target)
        wpilib.SmartDashboard.putNumber("Swerve/Target Y Vector", self.y_vector_to_target)
        wpilib.SmartDashboard.putNumber("Swerve/Target X", self.target_x)
        wpilib.SmartDashboard.putNumber("Swerve/Target Y", self.target_y)
        wpilib.SmartDashboard.putNumber("Swerve/Full Throttle Hold (s)", self._full_throttle_hold_seconds)
        wpilib.SmartDashboard.putNumber("Swerve/Swerve Max Speed Factor", self._max_speed_factor)
        wpilib.SmartDashboard.putNumber("Swerve/Full Throttle Hold (s)", self._full_throttle_hold_seconds)
        wpilib.SmartDashboard.putNumber("Swerve/Swerve Max Speed Factor", self._max_speed_factor)

        self._max_speed = self._max_speed_factor * TunerConstants.speed_at_12_volts

        # Dashboard pose output
        pose_translation = self._latest_pose.translation()
        pose_rotation = self._latest_pose.rotation()
        SmartDashboard.putNumber("Swerve/Swerve Pose X (meters)", pose_translation.X())
        SmartDashboard.putNumber("Swerve/Swerve Pose Y (meters)", pose_translation.Y())
        SmartDashboard.putNumber("Swerve/Swerve Rotation (deg)", pose_rotation.degrees())
        self.field.setRobotPose(self._latest_pose)

    def _determine_padlock_target(self, pose: Pose2d) -> tuple:
        selected_target = self._padlock_target_chooser.getSelected()
        if selected_target == (-1.0, -1.0) or selected_target is None: 
            # If "Auto Targetting" is selected, choose target based on alliance and position
            if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
                if pose.translation().X() < 4.6:
                    selected_target = (4.6, 4.0)
                elif pose.translation().Y() > 4.0:
                    selected_target = (4.0, 6.0)
                else:
                    selected_target = (4.0, 2.0)
            elif DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                if pose.translation().X() < 12.0:
                    selected_target = (12.0, 4.0)
                elif pose.translation().Y() > 4.0:
                    selected_target = (12.0, 6.0)
                else:
                    selected_target = (12.0, 2.0)
        return selected_target

    # -------------------------
    # Drive mode switching for joystick/gamepad control
    # -------------------------
    def drive_mode_field_centered(self) -> None:
        # Field-centric translation; use the right-stick vector (axes 2 & 3)
        # to set a target heading. _heading_from_right_stick() preserves the
        # last heading when the stick is near-center, so the robot won't
        # constantly re-orient when the driver releases the stick.
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_facing_direction
                    .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=False) * self._max_speed)
                    .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=False) * self._max_speed)
                    .with_target_direction(self._heading_from_right_stick())
                    .with_heading_pid(7, 0, 0)) ))

    def drive_mode_padlocked(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_facing_direction
                    .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=False) * self._max_speed)
                    .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=False) * self._max_speed)
                    .with_target_direction(Rotation2d(self.x_vector_to_target, self.y_vector_to_target)) # Desired Heading (e.g., (0,1) = 90 deg)
                    .with_heading_pid(20, 0, 0) ))  ) # PID for heading control
    
    def drive_mode_robot_centered(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(lambda: (
                self._drive_robot_centered
                    .with_velocity_x(-self._smoothed_axis(self._joystick.getLeftY(), self._left_y_limiter, square_input=False) * self._max_speed)
                    .with_velocity_y(-self._smoothed_axis(self._joystick.getLeftX(), self._left_x_limiter, square_input=False) * self._max_speed)
                    # Use raw axis indices 2 and 3 for the right stick so the
                    # controller mapping matches what the driver expects (axes
                    # 2/3 instead of 4/5). _joystick_axis will fallback to
                    # getRightX()/getRightY() if getRawAxis is not available.
                    .with_rotational_rate(-self._smoothed_axis(self._joystick_axis(2), self._right_x_limiter) * self._max_angular_rate)) ))

    # -------------------------
    # Drive requests for automated movement
    # -------------------------
    def free_rotate_drive_request_command(self, vx_requested, vy_requested, rotational_rate) -> commands2.Command:
        return self.drivetrain.apply_request(lambda: (
            self._drive_field_centered
                .with_velocity_x(vx_requested * self._max_speed)
                .with_velocity_y(vy_requested * self._max_speed)
                .with_rotational_rate(rotational_rate * self._max_angular_rate) ))

    def padlocked_drive_request_command(self, vx_requested, vy_requested, x_vector=0.0, y_vector=0.0) -> commands2.Command:
        if x_vector == 0 and y_vector == 0:
            x_vector = self.x_vector_to_target
            y_vector = self.y_vector_to_target
        return self.drivetrain.apply_request(lambda: (
            self._drive_facing_direction
                .with_velocity_x(vx_requested * self._max_speed)
                .with_velocity_y(vy_requested * self._max_speed)
                .with_target_direction(Rotation2d(x_vector, y_vector))
                .with_heading_pid(5, 0, 0) ))

    def robot_pov_drive_request_command(self, direction_x, direction_y) -> commands2.Command:
        return self.drivetrain.apply_request(lambda: (
            self._drive_robot_centered
                .with_velocity_x(direction_x * self._pov_speed)
                .with_velocity_y(direction_y * self._pov_speed)
                .with_rotational_rate(0) ) )

    def brake(self) -> None:
        self.drivetrain.apply_request(lambda: swerve.requests.SwerveDriveBrake())

    # -------------------------
    # Utility functions
    # -------------------------
    def PIDF_sysID_tuning_bindings(self) -> None:
        # Previously these bindings required pressing the Start button as a
        # modifier (e.g. Start + LeftBumper). That makes the face buttons and
        # bumpers appear to not work if you press them by themselves. Bind the
        # bumpers and face buttons directly so pressing them alone runs the
        # intended actions.
        # Start/Stop SignalLogger (bumpers alone)
        self._joystick.leftBumper().onTrue(SignalLogger.start)
        self._joystick.rightBumper().onTrue(SignalLogger.stop)

        # SysID routines: use face buttons directly (no Start modifier required)
        self._joystick.a().whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.b().whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        self._joystick.y().whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.x().whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

    def reset_field_oriented_perspective(self) -> None:
        # Resets the rotation of the robot pose to 0 from the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective. 
        # This makes the current orientation of the robot X forward for field-centric maneuvers.
        return self.drivetrain.seed_field_centric()


    # -------------------------
    # Pathplannerlib setup and helpers
    # -------------------------
    def _setup_pathplanner_auto_builder(self) -> None:
        AutoBuilder.configure(
            pose_supplier=self.get_pose,
            reset_pose=self.reset_pose,
            robot_relative_speeds_supplier=self.get_robot_relative_speeds,
            output=self.drive_robot_relative,
            controller=PPHolonomicDriveController(
                PIDConstants(2.0, 0.0, 0.0),  # Translation PID (tune these values)
                PIDConstants(2.0, 0.0, 0.0),  # Rotation PID (tune these values)
            ),
            robot_config=RobotConfig.fromGUISettings(),
            should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            drive_subsystem=self
        )


    def get_pose(self) -> Pose2d:
        return self._latest_pose

    def reset_pose(self, pose: Pose2d) -> None:
        self.drivetrain.reset_pose(pose)
        self._latest_pose = pose

    def get_robot_relative_speeds(self) -> ChassisSpeeds:
        """Get the current robot-relative chassis speeds.
        The underlying drivetrain may not yet have a valid state (especially
        during early init or in simulation). Return a zero ChassisSpeeds if the
        state is unavailable to avoid AttributeError.
        """
        state = None
        try:
            state = self.drivetrain.get_state()
        except Exception:
            # Be defensive: if getting state raises, treat as no motion.
            state = None

        if state is None:
            return ChassisSpeeds(0.0, 0.0, 0.0)

        # Some drivetrain implementations may not include 'speeds' attribute
        # (unlikely), so be defensive.
        return getattr(state, "speeds", ChassisSpeeds(0.0, 0.0, 0.0))

    def drive_robot_relative(self, robot_relative_speeds: ChassisSpeeds, drive_feedforwards=None) -> None:
        self.drivetrain.set_control(
            swerve.requests.ApplyRobotSpeeds().with_speeds(robot_relative_speeds)
        )

    def _setup_padlock_target_chooser(self):
        # This is an example of how you might set up a dashboard chooser to select between different padlock targets (e.g., different scoring locations)
        self._padlock_target_chooser = wpilib.SendableChooser()
        self._padlock_target_chooser.setDefaultOption("Auto Targetting", (-1.0, -1.0)) # Choose a target based on alliance and position
        self._padlock_target_chooser.addOption("Blue Target", (4.6, 4.0)) # Blue alliance target
        self._padlock_target_chooser.addOption("Blue Top Zone", (4.0, 6.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Blue Bottom Zone", (4.0, 2.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Target", (12.0, 4.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Top Zone", (12.6, 6.0)) # Red alliance target
        self._padlock_target_chooser.addOption("Red Bottom Zone", (12.6, 2.0)) # Red alliance target
        wpilib.SmartDashboard.putData("Swerve/Padlock Target Chooser", self._padlock_target_chooser)

    def _smoothed_axis(self, raw_axis: float, limiter: SlewRateLimiter, deadband: float = 0.08, square_input: bool = True) -> float:
        axis = applyDeadband(raw_axis, deadband)
        if square_input:
            axis = axis * abs(axis)
        return limiter.calculate(axis)

    def _heading_from_right_stick(self) -> Rotation2d:
        # Read the controller raw axes 2 and 3 for right-stick X/Y. Some
        # gamepads (or custom mappings) put the right-stick on axes 2/3 rather
        # than 4/5; this ensures we use the driver's preferred mapping. Use
        # _joystick_axis which gracefully falls back if getRawAxis isn't
        # available.
        right_x = self._smoothed_axis(self._joystick_axis(2), self._right_x_limiter)
        right_y = self._smoothed_axis(self._joystick_axis(3), self._right_y_limiter)
        right_mag = (right_x * right_x + right_y * right_y) ** 0.5
        if right_mag > 0.20:
            self._last_heading = Rotation2d(-right_y, -right_x)
        return self._last_heading

    def _joystick_axis(self, axis_index: int) -> float:
        """Return the raw axis value for the given index.

        Tries CommandGenericHID.getRawAxis first (used by CommandXboxController
        and friends). If that isn't available, falls back to getRightX/getRightY
        for backwards compatibility when axis_index matches those semantics.
        """
        try:
            # CommandGenericHID exposes getRawAxis(axis)
            return self._joystick.getRawAxis(axis_index)
        except Exception:
            # Best-effort fallback: map common right-stick indices to helper
            # methods if present.
            if axis_index == 2:
                try:
                    return self._joystick.getRightX()
                except Exception:
                    return 0.0
            if axis_index == 3:
                try:
                    return self._joystick.getRightY()
                except Exception:
                    return 0.0
            return 0.0
