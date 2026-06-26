package first.robot.subsystems;

import static com.ctre.phoenix6.signals.NeutralModeValue.Coast;
import static first.robot.Constants.CANIVORE_BUS;
import static first.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_0;
import static first.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_1;
import static first.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_FOLLOWER_2;
import static first.robot.Constants.ShooterConstants.DEVICE_ID_FLYWHEEL_LEADER;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_EJECT_VELOCITY;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_SLOT_CONFIGS;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
import static first.robot.Constants.ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

/** Shooter subsystem: turret yaw + pitch + flywheel. */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class ShooterSubsystem extends Mechanism {
  private final TalonFX flywheelLeaderMotor = new TalonFX(DEVICE_ID_FLYWHEEL_LEADER, CANIVORE_BUS);
  private final TalonFX flywheelFollower0Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_0, CANIVORE_BUS);
  private final TalonFX flywheelFollower1Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_1, CANIVORE_BUS);
  private final TalonFX flywheelFollower2Motor = new TalonFX(DEVICE_ID_FLYWHEEL_FOLLOWER_2, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelLeaderMotor.getVelocity(false);
  private final StatusSignal<AngularAcceleration> flywheelAcceleration = flywheelLeaderMotor.getAcceleration(false);

  /**
   * Creates a new shooter subsystem
   */
  public ShooterSubsystem() {
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(Coast).withInverted(InvertedValue.CounterClockwise_Positive))

        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(FLYWHEEL_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(FLYWHEEL_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(FLYWHEEL_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true))
        .withSlot0(Slot0Configs.from(FLYWHEEL_SLOT_CONFIGS));

    flywheelLeaderMotor.getConfigurator().apply(flywheelConfig);
    flywheelFollower0Motor.getConfigurator().apply(flywheelConfig);
    flywheelFollower1Motor.getConfigurator().apply(flywheelConfig);
    flywheelFollower2Motor.getConfigurator().apply(flywheelConfig);

    // Increase the leader update frequency so followers can respond quickly
    flywheelLeaderMotor.getTorqueCurrent(false).setUpdateFrequency(Hertz.of(200));
    // Keep higher update frequency for used and important signals for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(100),
          flywheelVelocity,
          flywheelAcceleration,
          flywheelLeaderMotor.getAcceleration(false));
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50),
          flywheelLeaderMotor.getStatorCurrent(false),
          flywheelLeaderMotor.getSupplyCurrent(false),
          flywheelFollower0Motor.getStatorCurrent(false),
          flywheelFollower0Motor.getSupplyCurrent(false),
          flywheelFollower1Motor.getStatorCurrent(false),
          flywheelFollower1Motor.getSupplyCurrent(false),
          flywheelFollower2Motor.getStatorCurrent(false),
          flywheelFollower2Motor.getSupplyCurrent(false));
    // Turn unused and follower signals down, but not off, for logging
    ParentDevice.optimizeBusUtilizationForAll(
        flywheelLeaderMotor,
          flywheelFollower0Motor,
          flywheelFollower1Motor,
          flywheelFollower2Motor);

    flywheelFollower0Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    flywheelFollower1Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelFollower2Motor.setControl(new Follower(flywheelLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /**
   * Creates a new command that runs the flywheel at a velocity. The command runs until canceled.
   * 
   * @param flywheelVelocity
   * @return new command
   */
  public Command runFlywheel(AngularVelocity flywheelVelocity) {
    return run(coroutine -> {
      flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(flywheelVelocity));
      coroutine.park();
    }).named("Run at " + flywheelVelocity);
  }

  /**
   * Creates a new command that runs the flywheel at a velocity provided by a supplier. The command runs until canceled.
   * 
   * @param flywheelVelocity velocity supplier
   * @return new command
   */
  public Command runFlywheel(Supplier<AngularVelocity> flywheelVelocity) {
    return run(coroutine -> {
      flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(flywheelVelocity.get()));
      coroutine.park();
    }).named("Run at supplied velocity");
  }

  /**
   * Creates a new command that spins the flywheel in reverse to eject balls or unjam fuel
   * 
   * @return new command
   */
  public Command eject() {
    return run(coroutine -> {
      flywheelLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(FLYWHEEL_EJECT_VELOCITY));
      coroutine.park();
    }).named("Eject");
  }

  /**
   * Creates a new command that stops the shooter
   * 
   * @return new command
   */
  public Command stop() {
    return run(coroutine -> {
      flywheelLeaderMotor.stopMotor();
      coroutine.park();
    }).named("Stop");
  }

  /**
   * Returns whether flywheel velocity is within tolerance of the active request
   * 
   * @return true if flywheel is at the target velocity, otherwise false
   */
  @Logged
  public boolean isFlywheelAtVelocity() {
    if (RobotBase.isSimulation()) {
      return true;
    } else {
      BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAcceleration);
      AngularVelocity currentSpeed = BaseStatusSignal
          .getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
      return MathUtil.isNear(
          flywheelVelocityRequest.Velocity,
            currentSpeed.in(RotationsPerSecond),
            FLYWHEEL_VELOCITY_TOLERANCE.in(RotationsPerSecond));
    }
  }

}
