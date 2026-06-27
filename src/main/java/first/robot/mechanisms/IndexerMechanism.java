package first.robot.mechanisms;

import static first.robot.Constants.CANIVORE_BUS;
import static first.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_MOTOR_FOLLOWER;
import static first.robot.Constants.IndexerConstants.DEVICE_ID_INDEXER_MOTOR_LEADER;
import static first.robot.Constants.IndexerConstants.INDEXER_EJECT_VELOCITY;
import static first.robot.Constants.IndexerConstants.INDEXER_FEED_VELOCITY;
import static first.robot.Constants.IndexerConstants.INDEXER_PEAK_TORQUE_CURRENT_FORWARD;
import static first.robot.Constants.IndexerConstants.INDEXER_PEAK_TORQUE_CURRENT_REVERSE;
import static first.robot.Constants.IndexerConstants.INDEXER_SLOT_CONFIGS;
import static first.robot.Constants.IndexerConstants.INDEXER_STATOR_CURRENT_LIMIT;
import static first.robot.Constants.IndexerConstants.INDEXER_SUPPLY_CURRENT_LIMIT;
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
 * Mechanism for the Indexer.
 */
public class IndexerMechanism extends Mechanism {

  private final TalonFX indexerLeaderMotor = new TalonFX(DEVICE_ID_INDEXER_MOTOR_LEADER, CANIVORE_BUS);
  private final TalonFX indexerFollower = new TalonFX(DEVICE_ID_INDEXER_MOTOR_FOLLOWER, CANIVORE_BUS);

  private final VelocityTorqueCurrentFOC indexerVelocityTorque = new VelocityTorqueCurrentFOC(0.0);

  /**
   * Creates a new Mechanism for the Indexer
   */
  public IndexerMechanism() {
    var indexerTalonconfig = new TalonFXConfiguration().withSlot0(Slot0Configs.from(INDEXER_SLOT_CONFIGS))
        .withMotorOutput(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(INDEXER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(INDEXER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(INDEXER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(INDEXER_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));

    indexerLeaderMotor.getConfigurator().apply(indexerTalonconfig);
    indexerFollower.getConfigurator().apply(indexerTalonconfig);

    // Increase update frequency for leader for fast following
    indexerLeaderMotor.getTorqueCurrent(false).setUpdateFrequency(Hertz.of(200));
    // Keep higher update frequency for important signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(50),
          indexerLeaderMotor.getVelocity(false),
          indexerLeaderMotor.getStatorCurrent(false),
          indexerLeaderMotor.getSupplyCurrent(false),
          indexerFollower.getStatorCurrent(false),
          indexerFollower.getTorqueCurrent(false),
          indexerFollower.getSupplyCurrent(false));
    // Turn unused and follower signals down, but not off, for logging
    ParentDevice.optimizeBusUtilizationForAll(indexerLeaderMotor, indexerFollower);

    indexerFollower.setControl(new Follower(indexerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  /**
   * Creates a command that spins the indexer forward to feed the shooter
   * 
   * @return new command
   */
  public Command feedShooter() {
    return run(coroutine -> {
      indexerLeaderMotor.setControl(indexerVelocityTorque.withVelocity(INDEXER_FEED_VELOCITY));
      coroutine.park();
    }).named("Feed Shooter");
  }

  /**
   * Returns a command that spins the indexer in reverse to eject or unjam fuel
   * 
   * @return new command
   */
  public Command eject() {
    return run(coroutine -> {
      indexerLeaderMotor.setControl(indexerVelocityTorque.withVelocity(INDEXER_EJECT_VELOCITY));
      coroutine.park();
    }).named("Eject");
  }

  /**
   * Returns a command to run the indexer at the set velocity. Used for tuning, should not be used for normal operation.
   * 
   * @param velocity the velocity to run the indexer
   * 
   * @return new command
   */
  public Command runIndexer(AngularVelocity velocity) {
    return run(coroutine -> {
      indexerLeaderMotor.setControl(indexerVelocityTorque.withVelocity(velocity));
      coroutine.park();
    }).named("Run Indexer at " + velocity);
  }

  /**
   * Returns a command that stops the indexer
   * 
   * @return new command
   */
  public Command stop() {
    return run(coroutine -> {
      indexerLeaderMotor.stopMotor();
      coroutine.park();
    }).named("Stop");
  }
}