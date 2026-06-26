//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package first.robot.subsystems;

import static first.robot.Constants.CANIVORE_BUS;
import static first.robot.Constants.IntakeConstants.CHANNEL_ID_DEPLOY_POTENTIOMETER;
import static first.robot.Constants.IntakeConstants.DEPLOYED_POSITION;
import static first.robot.Constants.IntakeConstants.DEPLOY_FORWARD_LIMIT;
import static first.robot.Constants.IntakeConstants.DEPLOY_MOTION_MAGIC_CONFIGS;
import static first.robot.Constants.IntakeConstants.DEPLOY_PEAK_CURRENT_FORWARD;
import static first.robot.Constants.IntakeConstants.DEPLOY_PEAK_CURRENT_REVERSE;
import static first.robot.Constants.IntakeConstants.DEPLOY_REVERSE_LIMIT;
import static first.robot.Constants.IntakeConstants.DEPLOY_SHOOTING_CURRENT;
import static first.robot.Constants.IntakeConstants.DEPLOY_SLOT_CONFIGS;
import static first.robot.Constants.IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
import static first.robot.Constants.IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
import static first.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static first.robot.Constants.IntakeConstants.DEVICE_ID_DEPLOY_MOTOR;
import static first.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER_FOLLOWER;
import static first.robot.Constants.IntakeConstants.DEVICE_ID_ROLLER_MOTOR;
import static first.robot.Constants.IntakeConstants.POSE_DEPLOYED;
import static first.robot.Constants.IntakeConstants.POSE_RETRACTED;
import static first.robot.Constants.IntakeConstants.POTENTIOMETER_FULL_RANGE;
import static first.robot.Constants.IntakeConstants.POTENTIOMETER_OFFSET;
import static first.robot.Constants.IntakeConstants.RETRACTED_POSITION;
import static first.robot.Constants.IntakeConstants.ROLLER_EJECT_VELOCITY;
import static first.robot.Constants.IntakeConstants.ROLLER_INTAKE_SHOOTING_VELOCITY;
import static first.robot.Constants.IntakeConstants.ROLLER_INTAKE_VELOCITY;
import static first.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_FORWARD;
import static first.robot.Constants.IntakeConstants.ROLLER_PEAK_TORQUE_CURRENT_REVERSE;
import static first.robot.Constants.IntakeConstants.ROLLER_SLOT_CONFIGS;
import static first.robot.Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
import static first.robot.Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
import static first.robot.Constants.ShootingConstants.JAM_DEBOUNCE_TIME;
import static first.robot.Constants.ShootingConstants.JAM_THRESHOLD;
import static first.robot.Constants.ShootingConstants.RETRACTED_THRESHOLD;
import static first.robot.Constants.ShootingConstants.RETRACT_INTAKE_DELAY;
import static first.robot.Constants.ShootingConstants.UNJAM_DURATION;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.epilogue.Logged;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.rotation.AnalogPotentiometer;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;

/**
 * Subsytem for the intake.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeSubsytem extends Mechanism {

  private final TalonFX rollerLeaderMotor = new TalonFX(DEVICE_ID_ROLLER_MOTOR, CANIVORE_BUS);
  private final TalonFX rollerFollowerMotor = new TalonFX(DEVICE_ID_ROLLER_FOLLOWER, CANIVORE_BUS);
  private final TalonFX deployMotor = new TalonFX(DEVICE_ID_DEPLOY_MOTOR, CANIVORE_BUS);

  // Motor request objects
  private final MotionMagicVoltage deployProfiledControl = new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC deployTorqueControl = new TorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC rollerControl = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> deployPositionSignal = deployMotor.getPosition(false);
  private final StatusSignal<AngularVelocity> deployVelocitySignal = deployMotor.getVelocity(false);

  @Logged(name = "Potentiometer")
  private final AnalogPotentiometer deploySensor = new AnalogPotentiometer(
      CHANNEL_ID_DEPLOY_POTENTIOMETER,
      POTENTIOMETER_FULL_RANGE.in(Rotations),
      POTENTIOMETER_OFFSET.in(Rotations));

  private Pose3d currentPose3d = POSE_RETRACTED;

  /**
   * Creates a new substyem for the intake
   */
  public IntakeSubsytem() {
    // Configure the roller motor
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withSlot0(Slot0Configs.from(ROLLER_SLOT_CONFIGS))
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(ROLLER_PEAK_TORQUE_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(ROLLER_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true));
    rollerLeaderMotor.getConfigurator().apply(rollerConfig);
    rollerFollowerMotor.getConfigurator().apply(rollerConfig);

    // Max update frequency for leader for fast following
    rollerLeaderMotor.getTorqueCurrent(false).setUpdateFrequency(Hertz.of(1000));
    // Keep default update frequency for current signals for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(100),
          rollerLeaderMotor.getVelocity(false),
          rollerLeaderMotor.getStatorCurrent(false),
          rollerLeaderMotor.getSupplyCurrent(false),
          rollerFollowerMotor.getStatorCurrent(false),
          rollerFollowerMotor.getSupplyCurrent(false));
    // Turn unused and follower signals down, but not off, for logging
    ParentDevice.optimizeBusUtilizationForAll(rollerLeaderMotor, rollerFollowerMotor);

    rollerFollowerMotor.setControl(new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    // Configure the deploy motor
    var deployConfig = new TalonFXConfiguration();
    deployConfig
        .withMotorOutput(
            new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive))
        .withSlot0(Slot0Configs.from(DEPLOY_SLOT_CONFIGS))
        .withMotionMagic(DEPLOY_MOTION_MAGIC_CONFIGS)
        .withTorqueCurrent(
            new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(DEPLOY_PEAK_CURRENT_FORWARD)
                .withPeakReverseTorqueCurrent(DEPLOY_PEAK_CURRENT_REVERSE))
        .withCurrentLimits(
            new CurrentLimitsConfigs().withSupplyCurrentLimit(DEPLOY_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(DEPLOY_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true))
        .withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(DEPLOY_FORWARD_LIMIT)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(DEPLOY_REVERSE_LIMIT));
    deployMotor.getConfigurator().apply(deployConfig);
    deployMotor.setPosition(getPotentiometerValue());

    // Keep default update frequency for important signals for logging
    BaseStatusSignal.setUpdateFrequencyForAll(
        Hertz.of(100),
          deployPositionSignal,
          deployVelocitySignal,
          deployMotor.getMotorVoltage(false),
          deployMotor.getStatorCurrent(false),
          deployMotor.getSupplyCurrent(false));
    // Turn unused signals down, but not off, for logging
    deployMotor.optimizeBusUtilization();
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  /**
   * Returns a new command to deploy the intake and run it to intake fuel
   * 
   * @return new command
   */
  public Command intake() {
    return run(coroutine -> {
      doDeploy();
      rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_VELOCITY));
      while (!isDeployed()) {
        coroutine.yield();
      }
      deployMotor.stopMotor();
      coroutine.park();
    }).whenCanceled(this::doStop).named("Intake");
  }

  /**
   * Returns a new command to deploy the intake and run it backwards to intake fuel
   * 
   * @return new command
   */
  public Command eject() {
    return run(coroutine -> {
      doDeploy();
      rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_EJECT_VELOCITY));
      while (!isDeployed()) {
        coroutine.yield();
      }
      deployMotor.stopMotor();
      coroutine.park();
    }).whenCanceled(this::doStop).named("Eject");
  }

  /**
   * Returns a new command to stop all intake motion
   * 
   * @return new command
   */
  public Command stop() {
    return run(coroutine -> {
      doStop();
      coroutine.park();
    }).named("Stop");
  }

  /**
   * Creates a command to deploy the intake. This command will turn off the intake and exit once the intake is deployed.
   * 
   * @return new command
   */
  public Command deploy() {
    return run(coroutine -> {
      doDeploy();
      while (!isDeployed()) {
        coroutine.yield();
      }
      doStop();
    }).whenCanceled(this::doStop).named("Deploy Intake");
  }

  /**
   * Creates a new command to retract the intake and turn it off once retracted. This command will run until
   * interrupted.
   * 
   * @return new command
   */
  public Command retract() {
    return run(coroutine -> {
      deployMotor.setControl(
          deployProfiledControl.withPosition(RETRACTED_POSITION)
              .withLimitReverseMotion(getPotentiometerValue() <= RETRACTED_POSITION.in(Rotations)));
      if (RobotBase.isSimulation()) {
        deployMotor.setPosition(RETRACTED_POSITION);
      }
      while (!isRetracted()) {
        coroutine.yield();
      }
      doStop();
      coroutine.park();
    }).whenCanceled(this::doStop).named("Retract");
  }

  /**
   * Returns a new command to run the shooting sequence with tunable unjam parameters
   * 
   * @param retractDelay the time to wait before retracting the intake
   * @param retractCurrent the current to use when retracting the intake
   * @param jamThreshold the velocity below which we consider the intake to be jammed
   * @param unjamDuration the duration for which to unjamming
   * @param retractedThreshold the deploy position below which we consider the intake retracted and can stop
   * 
   * @return new command
   */
  public Command shootingSequence(
      Time debounceTime,
      Time retractDelay,
      Current retractCurrent,
      AngularVelocity jamThreshold,
      Time unjamDuration,
      Angle retractedThreshold) {

    return run(coroutine -> {
      Debouncer jamDebouncer = new Debouncer(debounceTime.in(Seconds));
      runIntakeForShooting();
      coroutine.wait(retractDelay);
      retractForShooting(retractCurrent);
      while (true) {
        // Check if the intake is in
        if (getDeployPosition().lte(retractedThreshold)) {
          deployMotor.stopMotor();
          coroutine.park(); // All done, but leave the rollers running until canceled
        }

        // Check if the intake is jammed
        boolean isJammed = deployVelocitySignal.refresh().getValue().gt(jamThreshold);
        if (jamDebouncer.calculate(isJammed)) {
          // Intake is jammed, jiggle it for unjamDuration
          doDeploy();
          coroutine.wait(unjamDuration);

          jamDebouncer.calculate(false);
          retractForShooting(retractCurrent);
        }

        coroutine.yield();
      }
    }).whenCanceled(this::doStop).named("Shooting Sequence");

  }

  /**
   * Returns a new command to run the shooting sequence with comp parameters
   * 
   * @return new command
   */
  public Command shootingSequence() {
    return shootingSequence(
        JAM_DEBOUNCE_TIME,
          RETRACT_INTAKE_DELAY,
          DEPLOY_SHOOTING_CURRENT,
          JAM_THRESHOLD,
          UNJAM_DURATION,
          RETRACTED_THRESHOLD);
  }

  /**
   * Checks if the intake is deployed
   * 
   * @return true if the intake is deployed, false otherwise
   */
  @Logged
  public boolean isDeployed() {
    return (getDeployPosition().gte(DEPLOYED_POSITION.minus(DEPLOY_TOLERANCE))
        || getPotentiometerValue() >= DEPLOYED_POSITION.minus(DEPLOY_TOLERANCE).in(Rotations));
  }

  /**
   * Gets the current deploy position
   * 
   * @return current deploy position
   */
  private Angle getDeployPosition() {
    // Signals refreshed in periodic
    return BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
  }

  /**
   * Checks if the intake is retracted
   * 
   * @return true if the intake is retracted, false otherwise
   */
  @Logged
  public boolean isRetracted() {
    // Signals refreshed in periodic
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    return (deployPosition.lte(RETRACTED_POSITION.plus(DEPLOY_TOLERANCE))
        || getPotentiometerValue() <= RETRACTED_POSITION.plus(DEPLOY_TOLERANCE).in(Rotations));
  }

  /**
   * Gets the pose of the intake for AdvantageScope.
   * <p>
   * <strong>This is not intended for use in robot code.</strong>
   * </p>
   * 
   * @return intake pose for AdvantageScope
   */
  @Logged
  public Pose3d getPose() {
    return currentPose3d;
  }

  private void periodic() {
    BaseStatusSignal.refreshAll(deployPositionSignal, deployVelocitySignal);
    // Update the intake pose for AdvantageScope
    Angle deployPosition = BaseStatusSignal.getLatencyCompensatedValue(deployPositionSignal, deployVelocitySignal);
    currentPose3d = POSE_RETRACTED.interpolate(POSE_DEPLOYED, deployPosition.div(DEPLOY_FORWARD_LIMIT).in(Value));
  }

  /**
   * Gets the position of the intake from the potentiometer.
   * 
   * @return the position reading from the potentiometer
   */
  private double getPotentiometerValue() {
    return deploySensor.get();
  }

  /**
   * Deploys the intake
   */
  private void doDeploy() {
    deployMotor.setControl(
        deployProfiledControl.withPosition(DEPLOYED_POSITION)
            .withLimitReverseMotion(getPotentiometerValue() >= DEPLOYED_POSITION.in(Rotations)));
    if (RobotBase.isSimulation()) {
      deployMotor.setPosition(DEPLOYED_POSITION);
    }
  }

  /**
   * Retracts the intake with a set current to help feed fuel into the feeder while shooting
   * 
   * @param current The current to apply to the deploy motor
   */
  private void retractForShooting(Current current) {
    deployMotor.setControl(deployTorqueControl.withOutput(current));
    if (RobotBase.isSimulation()) {
      deployMotor.setPosition(RETRACTED_POSITION);
    }
  }

  /**
   * Spins the intake rollers at a velocity to help feed fuel into the feeder while shooting
   */
  private void runIntakeForShooting() {
    rollerLeaderMotor.setControl(rollerControl.withVelocity(ROLLER_INTAKE_SHOOTING_VELOCITY));
  }

  /**
   * Stops all intake motion
   */
  private void doStop() {
    rollerLeaderMotor.stopMotor();
    deployMotor.stopMotor();
  }

}
