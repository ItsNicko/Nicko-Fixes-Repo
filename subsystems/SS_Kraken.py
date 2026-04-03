import wpilib
import phoenix6
import commands2
from phoenix6 import CANBus
from wpilib import SmartDashboard
from pathplannerlib.auto import NamedCommands

class SS_Kraken(commands2.Subsystem):
    def __init__(self, device_id: int, canbus: CANBus, dashboard_name: str, 
                 inverted: bool=False, brake_mode: bool=False,
                 max_rps: int=100, velocity_setpoint: float=0.0, percent_power_setpoint: float=0.0,
                 kp: float=0.0, ki: float=0.0, kd: float=0.0, kv: float=0.0, ks: float=0.0,
                 ka: float=0.0, kg: float=0.0, Vmax: float=0.0, Amax: float=0.0, Jerk: float=0.0) -> None:
        self.motor = phoenix6.hardware.TalonFX(device_id, canbus)

        self.dashboard_name = dashboard_name
        self.max_rps = max_rps
        self.percent_power_setpoint = percent_power_setpoint
        self.velocity_setpoint = velocity_setpoint
        self.velocity_actual = 0.0
        self._setup_hardware_configuration(inverted, brake_mode)
        self._apply_pidf_to_config(kp, ki, kd, kv, ks, ka, kg, Vmax, Amax, Jerk)
        self.requested_position = 0.0
        self.position_actual = 0.0
        self.motor.set_position(self.requested_position) # pidf reset encoder position to 0 on startup

        self._put_telemetry_on_dashboard()
        self._register_pathplanner_commands()

    def _setup_hardware_configuration(self, inverted: bool=False, brake_mode: bool=False):
        self._config = phoenix6.configs.TalonFXConfiguration()
        if brake_mode:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.BRAKE
        else:
            self._config.motor_output.neutral_mode = phoenix6.signals.NeutralModeValue.COAST
        if inverted:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            self._config.motor_output.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE

        self._config.current_limits.stator_current_limit_enable = True
        self._config.current_limits.stator_current_limit = 100.0
        self._config.current_limits.supply_current_limit_enable = True
        self._config.current_limits.supply_current_limit = 40.0

        self.velocity_request = phoenix6.controls.VelocityDutyCycle(0.0)
        self.velocity_FOC_request = phoenix6.controls.VelocityTorqueCurrentFOC(0.0) # advanced control unused
        self.position_request_with_trapezoid = phoenix6.controls.MotionMagicDutyCycle(0.0)

    def _apply_pidf_to_config(self, new_p, new_i, new_d, new_v, new_s, 
                              new_a, new_g, new_Vmax, new_Amax, new_Jerk):
        wpilib.reportWarning(f"Applying PIDF config to {self.dashboard_name}: kP={new_p}, kI={new_i}, kD={new_d}, kV={new_v}, kS={new_s}, kA={new_a}, kG={new_g}, Vmax={new_Vmax}, Amax={new_Amax}, Jerk={new_Jerk}", printTrace=False)
        self._config.slot0.k_p = self.kP = new_p
        self._config.slot0.k_i = self.kI = new_i
        self._config.slot0.k_d = self.kD = new_d
        self._config.slot0.k_v = self.kV = new_v # optional velocity feedforward for running at certain speeds
        self._config.slot0.k_s = self.kS = new_s # optional static feedforward for overcoming static friction
        self._config.slot0.k_a = self.kA = new_a # optional acceleration feedforward for compensating inertia
        self._config.slot0.k_g = self.kG = new_g # optional gravity feedforward for compensating gravity effects
        self.Vmax = new_Vmax
        self.Amax = new_Amax
        self.Jerk = new_Jerk
        if self.Vmax or self.Amax or self.Jerk:
            self._config.motion_magic.motion_magic_cruise_velocity = self.Vmax
            self._config.motion_magic.motion_magic_acceleration = self.Amax
            self._config.motion_magic.motion_magic_jerk = self.Jerk
        #self._config.feedback.sensor_to_mechanism_ratio = 100.0

        self.status = self.motor.configurator.apply(self._config)
        if not self.status.is_ok():
            wpilib.reportError(f"Kraken PID update failed: {self.status}", False)
        SmartDashboard.putBoolean(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Config Success", self.status.is_ok())

    def _put_telemetry_on_dashboard(self):
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Actual", self.velocity_actual)
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Position Actual", self.position_actual)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kP", self.kP)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kI", self.kI)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kD", self.kD)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kV", self.kV)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kS", self.kS)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} ka", self.kA)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kg", self.kG)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Vmax", self.Vmax)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Amax", self.Amax)
        SmartDashboard.putNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Jerk", self.Jerk)

    def _register_pathplanner_commands(self):
        # NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Spin-up to Setpoint", self.spin_up_and_wait_command())
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Power Percent Forward", commands2.cmd.runOnce(self.run_voltage_percent_forward))
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Power Percent Reverse", commands2.cmd.runOnce(self.run_voltage_percent_reverse))
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", commands2.cmd.runOnce(self.run_velocity_at_setpoint))
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Stop", commands2.cmd.runOnce(self.stop_motor))
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Point Ahead", commands2.cmd.runOnce(lambda: self.set_position(0)))
        NamedCommands.registerCommand(f"{self.dashboard_name}/{self.dashboard_name} Point Right", commands2.cmd.runOnce(lambda: self.set_position(-2.56)))

    # -------------------------
    # Periodic tasks - dashboard updates and config changes
    # -------------------------
    def periodic(self):
        self.position_actual = self.motor.get_position().value
        self.velocity_actual = self.motor.get_velocity().value
        SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Actual", self.velocity_actual)
        dashboard_velocity_setpoint = SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        if dashboard_velocity_setpoint != self.velocity_setpoint:
            self.velocity_setpoint = min(dashboard_velocity_setpoint, self.max_rps)
            SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Velocity Setpoint", self.velocity_setpoint)
        dashboard_power_percent_setpoint = SmartDashboard.getNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)
        if dashboard_power_percent_setpoint != self.percent_power_setpoint:
            self.percent_power_setpoint = max(min(dashboard_power_percent_setpoint, 1.0), -1.0)
            SmartDashboard.putNumber(f"SS_Telemetry/{self.dashboard_name}/{self.dashboard_name} Power Percent Setpoint", self.percent_power_setpoint)

        dashboard_p = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Position Actual", self.position_actual)
        dashboard_p = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kP", self.kP)
        dashboard_i = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kI", self.kI)
        dashboard_d = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kD", self.kD)
        dashboard_v = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kV", self.kV)
        dashboard_s = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kS", self.kS)
        dashboard_a = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kA", self.kA)
        dashboard_g = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} kG", self.kG)
        dashboard_Vmax = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Vmax", self.Vmax)
        dashboard_Amax = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Amax", self.Amax)
        dashboard_Jerk = SmartDashboard.getNumber(f"PIDF/{self.dashboard_name}/{self.dashboard_name} Jerk", self.Jerk)
        if (dashboard_p != self.kP or dashboard_i != self.kI or dashboard_d != self.kD or 
            dashboard_v != self.kV or dashboard_s != self.kS or dashboard_a != self.kA or dashboard_g != self.kG or
            dashboard_Vmax != self.Vmax or dashboard_Amax != self.Amax or dashboard_Jerk != self.Jerk):
            self._apply_pidf_to_config(dashboard_p, dashboard_i, dashboard_d, dashboard_v, dashboard_s, 
                                       dashboard_a, dashboard_g, dashboard_Vmax, dashboard_Amax, dashboard_Jerk)
        self._put_telemetry_on_dashboard()


    # -------------------------
    # Motor movement functions
    # -------------------------
    def run_velocity_at_setpoint(self, setpoint:float = None) -> None:
        if setpoint is None:
            setpoint = self.velocity_setpoint
        """Needs PIDF settings and max_rps to be configured appropriately for good performance."""
        self.motor.set_control(self.velocity_request.with_velocity(setpoint))

    def run_voltage_percent_forward(self, setpoint:float = None) -> None:
        if setpoint is None:
            setpoint = self.percent_power_setpoint
        self.motor.set(setpoint)

    def run_voltage_percent_reverse(self, setpoint:float = None) -> None:
        if setpoint is None:
            setpoint = self.percent_power_setpoint
        self.motor.set(-setpoint)

    def set_position(self, target_rotations: float = None) -> None:
        if target_rotations is None:
            target_rotations = self.requested_position
        self.motor.set_control(self.position_request_with_trapezoid.with_position(float(target_rotations)))
        self.requested_position = target_rotations

    def stop_motor(self):
        self.motor.set(0.0)
        # self.motor.stopMotor()

    # -------------------------
    # Commands
    # -------------------------
    def spin_up_and_wait_command(self):
        class SpinUpAndWait_CommDef(commands2.Command):
            def __init__(self, ss_kraken: SS_Kraken):
                super().__init__()
                self.ss_kraken = ss_kraken
                self.addRequirements(ss_kraken) # Ensure no other command uses subsystems
                self.timer = wpilib.Timer()
                self.velocity_tolerance = 10 # RPS tolerance for considering the kraken "up to speed"

            def initialize(self):
                self.timer.restart()
                self.ss_kraken.run_velocity_at_setpoint() # Spin up

            def execute(self):
                pass # Command is running

            def isFinished(self):
                return abs(self.ss_kraken.velocity_actual - self.ss_kraken.velocity_setpoint) < self.velocity_tolerance

            def end(self, interrupted):
                # Keep spinning even if interrupted, since this command is just for waiting until up to speed.
                if interrupted:
                    wpilib.reportWarning("SpinUpAndWait_Command was interrupted before reaching target velocity!", printTrace=False)
                else:
                    wpilib.reportWarning(f"SpinUpAndWait_Command reached target velocity: {self.timer.get():.1f} seconds.", printTrace=False)
                
        spin_up_and_wait_command = SpinUpAndWait_CommDef(self)
        return spin_up_and_wait_command
