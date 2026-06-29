//Copyright (c) FIRST and other WPILib contributors
//Open Source Software; can modify and/or share it under the terms of
//the WPILib BSD licensefile in the root directory of this project.

package first.robot.mechanisms;

import static first.robot.Constants.CANIVORE_BUS;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;

/**
 * Subsytem for the intake.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class IntakeMechanism extends Mechanism {
  private static final int DEVICE_ID_DEPLOY_MOTOR = 10;
  private static final int DEVICE_ID_ROLLER_MOTOR = 11; // left
  private static final int DEVICE_ID_ROLLER_FOLLOWER = 12; // right
  private static final int CHANNEL_ID_DEPLOY_POTENTIOMETER = 3;

  private static final Current ROLLER_PEAK_TORQUE_CURRENT_FORWARD = Amps.of(170);
  private static final Current ROLLER_PEAK_TORQUE_CURRENT_REVERSE = ROLLER_PEAK_TORQUE_CURRENT_FORWARD.unaryMinus();
  private static final Current ROLLER_STATOR_CURRENT_LIMIT = Amps.of(190);
  private static final Current ROLLER_SUPPLY_CURRENT_LIMIT = Amps.of(80);
  private static final SlotConfigs ROLLER_SLOT_CONFIGS = new SlotConfigs().withKP(12).withKS(5.1);

  private static final AngularVelocity ROLLER_INTAKE_VELOCITY = RotationsPerSecond.of(80.0);
  private static final AngularVelocity ROLLER_INTAKE_SHOOTING_VELOCITY = RotationsPerSecond.of(40.0);
  private static final AngularVelocity ROLLER_EJECT_VELOCITY = RotationsPerSecond.of(-30.0);

  private static final Current DEPLOY_STATOR_CURRENT_LIMIT = Amps.of(40);
  private static final Current DEPLOY_SUPPLY_CURRENT_LIMIT = Amps.of(30);
  private static final Current DEPLOY_PEAK_CURRENT_FORWARD = Amps.of(50);
  private static final Current DEPLOY_PEAK_CURRENT_REVERSE = DEPLOY_PEAK_CURRENT_FORWARD.unaryMinus();

  private static final Angle DEPLOY_REVERSE_LIMIT = Rotations.of(0); // Retracted
  private static final Angle DEPLOY_FORWARD_LIMIT = Rotations.of(11.10); // Deployed
  private static final double POTENTIOMETER_REVERSE_LIMIT = 0.544;
  private static final double POTENTIOMETER_FORWARD_LIMIT = 0.913;
  private static final Angle POTENTIOMETER_FULL_RANGE = DEPLOY_FORWARD_LIMIT.minus(DEPLOY_REVERSE_LIMIT)
      .div(POTENTIOMETER_FORWARD_LIMIT - POTENTIOMETER_REVERSE_LIMIT);
  private static final Angle POTENTIOMETER_OFFSET = Rotations.of(-16.31);

  private static final SlotConfigs DEPLOY_SLOT_CONFIGS = new SlotConfigs().withGravityType(GravityTypeValue.Arm_Cosine)
      .withKP(5.0)
      .withKS(0.0)
      .withKV(0.0);
  private static final MotionMagicConfigs DEPLOY_MOTION_MAGIC_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicAcceleration(120.0)
      .withMotionMagicCruiseVelocity(180.0);

  private static final Angle DEPLOYED_POSITION = DEPLOY_FORWARD_LIMIT.minus(Rotations.of(0.25));
  public static final Angle RETRACTED_POSITION = DEPLOY_REVERSE_LIMIT.plus(Rotations.of(0.25));

  private static final Pose3d POSE_DEPLOYED = new Pose3d(0.267, 0.0, -0.043, Rotation3d.kZero);
  private static final Pose3d POSE_RETRACTED = new Pose3d(0.0, 0.0, 0.0, Rotation3d.kZero);

  private static final Angle DEPLOY_TOLERANCE = Rotations.of(0.2);
  public static final Current DEPLOY_SHOOTING_CURRENT = Amps.of(-29.0);

  public static final Time RETRACT_INTAKE_DELAY = Seconds.of(0.18);
  public static final Time JAM_DEBOUNCE_TIME = Seconds.of(0.5);
  public static final AngularVelocity JAM_THRESHOLD = RotationsPerSecond.of(-1.0);
  public static final Time UNJAM_DURATION = Seconds.of(0.25);
  public static final Angle RETRACTED_THRESHOLD = RETRACTED_POSITION;

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
  public IntakeMechanism() {
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
    }).named("Intake");
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
    }).named("Eject");
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
    }).named("Deploy Intake");
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
    }).named("Retract");
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
    }).named("Shooting Sequence");

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
