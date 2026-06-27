package first.robot.mechanisms;

import static first.robot.Constants.CANIVORE_BUS;
import static first.robot.Constants.FeederConstants.DEVICE_ID_FEEDER_FOLLOWER;
import static first.robot.Constants.FeederConstants.DEVICE_ID_FEEDER_LEADER;
import static first.robot.Constants.FeederConstants.FEEDER_EJECT_VELOCITY;
import static first.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static first.robot.Constants.FeederConstants.FEEDER_PEAK_TORQUE_CURRENT_FORWARD;
import static first.robot.Constants.FeederConstants.FEEDER_PEAK_TORQUE_CURRENT_REVERSE;
import static first.robot.Constants.FeederConstants.FEEDER_SLOT_CONFIGS;
import static first.robot.Constants.FeederConstants.FEEDER_STATOR_CURRENT_LIMIT;
import static first.robot.Constants.FeederConstants.FEEDER_SUPPLY_CURRENT_LIMIT;
import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Mechanism for the Feeder.
 */
public class FeederMechanism extends Mechanism {
  private final TalonFX feederLeaderMotor = new TalonFX(DEVICE_ID_FEEDER_LEADER, CANIVORE_BUS);
  private final TalonFX feederFollowerMotor = new TalonFX(DEVICE_ID_FEEDER_FOLLOWER, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC feederVelocityTorque = new VelocityTorqueCurrentFOC(0.0);

  public FeederMechanism() {
    var feederTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(FEEDER_SLOT_CONFIGS))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(FEEDER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(FEEDER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(FEEDER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(FEEDER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));
    feederTalonconfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feederLeaderMotor.getConfigurator().apply(feederTalonconfig);
    feederFollowerMotor.getConfigurator().apply(feederTalonconfig);

    // Increase update frequency for leader for fast following
    feederLeaderMotor.getTorqueCurrent(false).setUpdateFrequency(Hertz.of(200));
    // Keep higher update frequency for logging important signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50),
          feederLeaderMotor.getVelocity(false),
          feederLeaderMotor.getStatorCurrent(false),
          feederLeaderMotor.getSupplyCurrent(false),
          feederFollowerMotor.getTorqueCurrent(false),
          feederFollowerMotor.getStatorCurrent(false),
          feederFollowerMotor.getSupplyCurrent(false));
    // Turn unused and follower signals down, but not off, for logging
    ParentDevice.optimizeBusUtilizationForAll(feederLeaderMotor, feederFollowerMotor);

    feederFollowerMotor.setControl(new Follower(feederLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /**
   * Creates a command to spin the feeder to feed the shooter
   * 
   * @return new command
   */
  public Command feedShooter() {
    return run(coroutine -> {
      feederLeaderMotor.setControl(feederVelocityTorque.withVelocity(FEEDER_FEED_VELOCITY));
      coroutine.park();
    }).named("Feed Shooter");
  }

  /**
   * Returns a command to run the feeder at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the feeder
   * 
   * @returns new command
   */
  public Command runFeeder(AngularVelocity velocity) {
    return run(coroutine -> {
      feederLeaderMotor.setControl(feederVelocityTorque.withVelocity(velocity));
      coroutine.park();
    }).named("Run feeder at " + velocity);
  }

  /**
   * Returns a command that spins the feeder backward to eject or unjam fuel
   * 
   * @return new command
   */
  public Command eject() {
    return run(coroutine -> {
      feederLeaderMotor.setControl(feederVelocityTorque.withVelocity(FEEDER_EJECT_VELOCITY));
      coroutine.park();
    }).named("Eject");
  }

  /**
   * Returns a command to stop the feeder motor
   */
  public Command stop() {
    return run(coroutine -> {
      feederLeaderMotor.stopMotor();
      coroutine.park();
    }).named("Stop");
  }

}
