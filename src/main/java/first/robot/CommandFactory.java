package first.robot;

import static first.robot.Constants.FieldConstants.FIELD_WIDTH;
import static first.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static first.robot.Constants.ShootingConstants.HUB_BLUE;
import static first.robot.Constants.ShootingConstants.HUB_RED;
import static first.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;
import static first.robot.Constants.ShootingConstants.RETRACTED_THRESHOLD;
import static first.robot.Constants.ShootingConstants.SHUTTLE_BLUE_HIGH;
import static first.robot.Constants.ShootingConstants.SHUTTLE_BLUE_LOW;
import static first.robot.Constants.ShootingConstants.SHUTTLE_OFFSET_DISTANCE;
import static first.robot.Constants.ShootingConstants.SHUTTLE_RED_HIGH;
import static first.robot.Constants.ShootingConstants.SHUTTLE_RED_LOW;
import static first.robot.Constants.TeleopDriveConstants.CENTER_OF_ROTATION;
import static first.robot.mechanisms.FeederMechanism.FEEDER_FEED_VELOCITY;
import static first.robot.mechanisms.IndexerMechanism.INDEXER_FEED_VELOCITY;
import static first.robot.mechanisms.IntakeMechanism.DEPLOY_SHOOTING_CURRENT;
import static first.robot.mechanisms.IntakeMechanism.JAM_DEBOUNCE_TIME;
import static first.robot.mechanisms.IntakeMechanism.JAM_THRESHOLD;
import static first.robot.mechanisms.IntakeMechanism.RETRACT_INTAKE_DELAY;
import static first.robot.mechanisms.IntakeMechanism.UNJAM_DURATION;
import static first.robot.mechanisms.ShooterMechanism.SHOOTER_OFFSET_ANGLE;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import first.robot.mechanisms.CommandSwerveDrivetrain;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeMechanism;
import first.robot.mechanisms.LEDMechanism;
import first.robot.mechanisms.ShooterMechanism;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.util.Color;

/**
 * Factory class for creating commands with the necessary mechanism dependencies.
 */
public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrainMechanism;
  private final ShooterMechanism shooterMechanism;
  private final IndexerMechanism indexerMechanism;
  private final FeederMechanism feederMechanism;
  private final IntakeMechanism intakeMechanism;
  private final LEDMechanism ledMechanism;

  /**
   * Constructor for Commandfactory, takes in all mechanisms as parameters to use in command creation methods.
   * 
   * @param drivetrainMechanism drivetrain mechanism
   * @param shooterMechanism shooter mechanism
   * @param indexerMechanism indexer mechanism
   * @param feederMechanism feeder mechanism
   * @param intakeMechanism intake mechanism
   * @param ledMechanism led mechanism
   */
  public CommandFactory(
      CommandSwerveDrivetrain drivetrainMechanism,
      ShooterMechanism shooterMechanism,
      IndexerMechanism indexerMechanism,
      FeederMechanism feederMechanism,
      IntakeMechanism intakeMechanism,
      LEDMechanism ledMechanism) {
    this.drivetrainMechanism = drivetrainMechanism;
    this.shooterMechanism = shooterMechanism;
    this.indexerMechanism = indexerMechanism;
    this.feederMechanism = feederMechanism;
    this.intakeMechanism = intakeMechanism;
    this.ledMechanism = ledMechanism;
  }

  /**
   * Creates a command to shoot at the hub
   * 
   * @return a new command to shoot at the hub
   */
  public Command shootAtHub() {
    return shootAtTarget(
        () -> drivetrainMechanism.getState().Pose,
          t -> MatchState.getAlliance().orElse(BLUE) == BLUE ? HUB_BLUE : HUB_RED);
  }

  public Command demoToss() {
    return justShootCommand(Meters.of(0.5));
  }

  /**
   * Creates a command to shuttle fuel from any location to the nearest alliance-specific corner.
   * 
   * @return a new command to shuttle fuel to the corner
   */
  public Command shuttleToCorner() {
    return shootAtTarget(() -> drivetrainMechanism.getState().Pose, targetTranslation -> {
      Translation2d target;
      if (MatchState.getAlliance().orElse(BLUE) == BLUE) {
        target = targetTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_BLUE_HIGH : SHUTTLE_BLUE_LOW;
      } else {
        target = targetTranslation.getY() > FIELD_WIDTH.in(Meters) / 2.0 ? SHUTTLE_RED_HIGH : SHUTTLE_RED_LOW;
      }
      // Adjust the target to be "offset distance" short of target along the vector between the robot and the target
      Translation2d vectorToTarget = target.minus(targetTranslation);
      double distanceToTarget = vectorToTarget.getNorm();
      Translation2d adjustedTarget = targetTranslation
          .plus(vectorToTarget.times((distanceToTarget - SHUTTLE_OFFSET_DISTANCE.in(Meters)) / distanceToTarget));
      return adjustedTarget;
    });
  }

  /**
   * Creates a new Command to control the drivetrain field-oriented using the provided translation and rotation
   * suppliers.
   *
   * @param translationXSupplier supplier for the robot's x translation velocity
   * @param translationYSupplier supplier for the robot's y translation velocity
   * @param omegaSupplier supplier for the robot's rotational velocity
   * @return a new Command to control the drivetrain
   */
  public Command drive(
      Supplier<LinearVelocity> translationXSupplier,
      Supplier<LinearVelocity> translationYSupplier,
      Supplier<AngularVelocity> omegaSupplier,
      LinearVelocity maxVelocity,
      AngularVelocity maxAngularVelocity) {
    /* Setting up bindings for necessary control of the swerve drive platform */
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withCenterOfRotation(CENTER_OF_ROTATION)
        .withDeadband(maxVelocity.times(0.01))
        .withRotationalDeadband(maxAngularVelocity.times(0.01))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    return drivetrainMechanism.applyRequest(
        () -> drive.withVelocityX(translationXSupplier.get())
            .withVelocityY(translationYSupplier.get())
            .withRotationalRate(omegaSupplier.get()));
  }

  public Command ejectCommand() {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(
          intakeMechanism.eject(),
            indexerMechanism.eject(),
            feederMechanism.eject(),
            shooterMechanism.eject(),
            ledMechanism
                .runPattern(LEDPattern.rainbow(255, 255).scrollAtRelativeVelocity(Percent.per(Second).of(200))));
    }).named("Eject");
  }

  public Command intakeWithLEDsCommand() {
    final LEDPattern patternOne = LEDPattern.gradient(GradientType.DISCONTINUOUS, Color.BLACK, Color.ORANGE)
        .scrollAtRelativeVelocity(Percent.per(Second).of(200));
    final LEDPattern patternTwo = patternOne.reversed();
    return Command.requiring(intakeMechanism).executing(coroutine -> {
      coroutine.fork(intakeMechanism.intake(), ledMechanism.runPatternOnHalves(patternOne, patternTwo));
      while (RobotState.isEnabled()) {
        coroutine.yield();
      }
    }).named("Intake Fuel");
  }

  public Command tuneShootCommand(Supplier<Pose2d> poseSupplier) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Tune Shoot");
    DoublePublisher distancePublisher = table.getDoubleTopic("_Hub Distance").publish();
    DoubleEntry flywheelSubscriber = table.getDoubleTopic("Flywheel Velocity (RPS)").getEntry(0.0);
    flywheelSubscriber.set(HUB_SETPOINTS_BY_DISTANCE_METERS.get(1.0));
    DoubleEntry feederVelocitySubscriber = table.getDoubleTopic("Feeder Velocity (RPS)")
        .getEntry(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    feederVelocitySubscriber.set(FEEDER_FEED_VELOCITY.in(RotationsPerSecond));
    DoubleEntry indexerVelocitySubscriber = table.getDoubleTopic("Indexer Velocity (RPS)")
        .getEntry(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    indexerVelocitySubscriber.set(INDEXER_FEED_VELOCITY.in(RotationsPerSecond));
    DoubleEntry retractDelaySubscriber = table.getDoubleTopic("Retract Delay (s)").getEntry(0.0);
    retractDelaySubscriber.set(RETRACT_INTAKE_DELAY.in(Seconds));
    DoubleEntry retractCurrentSubscriber = table.getDoubleTopic("Retract Current (A)").getEntry(0.0);
    retractCurrentSubscriber.set(DEPLOY_SHOOTING_CURRENT.in(Amps));
    DoubleEntry jamDebounceSubscriber = table.getDoubleTopic("Jam Debounce Time (s)").getEntry(0.0);
    jamDebounceSubscriber.set(JAM_DEBOUNCE_TIME.in(Seconds));
    DoubleEntry unjamDurationSubscriber = table.getDoubleTopic("Unjam Duration (s)").getEntry(0.0);
    unjamDurationSubscriber.set(UNJAM_DURATION.in(Seconds));
    DoubleEntry jamThresholdSubscriber = table.getDoubleTopic("Jam Threshold (RPS)").getEntry(0.0);
    jamThresholdSubscriber.set(JAM_THRESHOLD.in(RotationsPerSecond));
    DoubleEntry retractedThresholdSubscriber = table.getDoubleTopic("Retracted Threshold (R)").getEntry(0.0);
    retractedThresholdSubscriber.set(RETRACTED_THRESHOLD.in(Rotations));

    return Command.requiring(intakeMechanism, shooterMechanism, feederMechanism, indexerMechanism)
        .executing(coroutine -> {
          Translation2d hubTranslation;
          var alliance = MatchState.getAlliance();
          hubTranslation = (alliance.isEmpty() || alliance.get() == BLUE) ? HUB_BLUE : HUB_RED;

          var distanceToHub = poseSupplier.get().getTranslation().getDistance(hubTranslation);
          distancePublisher.accept(distanceToHub);

          var flywheelVelocity = RotationsPerSecond.of(flywheelSubscriber.get());
          var retractDelay = Seconds.of(retractDelaySubscriber.get());
          var retractCurrent = Amps.of(retractCurrentSubscriber.get());
          var jamThreshold = RotationsPerSecond.of(jamThresholdSubscriber.get());
          var unjamDuration = Seconds.of(unjamDurationSubscriber.get());
          var retractedThreshold = Rotations.of(retractedThresholdSubscriber.get());
          var feederVelocity = RotationsPerSecond.of(feederVelocitySubscriber.get());
          var indexerVelocity = RotationsPerSecond.of(indexerVelocitySubscriber.get());

          // Get ready to shoot
          coroutine.fork(
              intakeMechanism.stop(),
                feederMechanism.stop(),
                indexerMechanism.stop(),
                shooterMechanism.runFlywheel(flywheelVelocity));

          // Wait until the flywheel is up to speed
          coroutine.waitUntil(shooterMechanism::isFlywheelAtVelocity);
          coroutine.wait(Milliseconds.of(6.0));

          // Shoot
          coroutine.fork(feederMechanism.runFeeder(feederVelocity));
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerMechanism.runIndexer(indexerVelocity));
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(
              intakeMechanism.shootingSequence(
                  unjamDuration,
                    retractDelay,
                    retractCurrent,
                    jamThreshold,
                    unjamDuration,
                    retractedThreshold));
          coroutine.park();
        })
        .named("Tune Shoot");
  }

  public Command justShootCommand(Distance targetDistance) {
    final SwerveRequest.SwerveDriveBrake swerveBrakeRequest = new SwerveRequest.SwerveDriveBrake();

    // LED patterns for shooting
    final LEDPattern patternTwo = LEDPattern.gradient(GradientType.DISCONTINUOUS, Color.BLACK, Color.BLUE)
        .scrollAtRelativeVelocity(Percent.per(Second).of(300));
    final LEDPattern patternOne = patternTwo.reversed();
    final AngularVelocity shooterAngularVelocity = RotationsPerSecond
        .of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance.in(Meters)));

    return Command
        .requiring(
            intakeMechanism,
              shooterMechanism,
              feederMechanism,
              indexerMechanism,
              drivetrainMechanism,
              ledMechanism)
        .executing(coroutine -> {
          // Get ready to shoot
          coroutine.fork(
              intakeMechanism.stop(),
                feederMechanism.stop(),
                indexerMechanism.stop(),
                shooterMechanism.runFlywheel(shooterAngularVelocity),
                drivetrainMechanism.applyRequest(() -> swerveBrakeRequest),
                ledMechanism.off());

          // Wait until the flywheel is up to speed
          coroutine.waitUntil(shooterMechanism::isFlywheelAtVelocity);

          // Shoot, using staggered mechanism starts
          coroutine.fork(ledMechanism.runPatternOnHalves(patternOne, patternTwo));
          coroutine.wait(Milliseconds.of(6.0));
          coroutine.fork(feederMechanism.feedShooter());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerMechanism.feedShooter());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(intakeMechanism.shootingSequence());
          while (!RobotController.isBrownedOut()) {
            coroutine.yield();
          }
          // Abort, brownout detected
        })
        .named("Tune Shoot");
  }

  private class ShootingState {
    boolean isAimReady;
    AngularVelocity shooterSpeed;
  }

  public Command shootAtTarget(
      Supplier<Pose2d> robotPoseSupplier,
      Function<Translation2d, Translation2d> targetTranslationSelector) {

    // LED patterns for shooting
    final LEDPattern patternTwo = LEDPattern.gradient(GradientType.DISCONTINUOUS, Color.BLACK, Color.RED)
        .scrollAtRelativeVelocity(Percent.per(Second).of(300));
    final LEDPattern patternOne = patternTwo.reversed();

    return Command
        .requiring(
            intakeMechanism,
              shooterMechanism,
              feederMechanism,
              indexerMechanism,
              drivetrainMechanism,
              ledMechanism)
        .executing(coroutine -> {
          // Create object to share the shooting state with forked commands
          ShootingState shootingState = new ShootingState();

          // Fork the commands to prepare to shoot
          coroutine.fork(
              intakeMechanism.stop(),
                feederMechanism.stop(),
                indexerMechanism.stop(),
                shooterMechanism.runFlywheel(() -> shootingState.shooterSpeed),
                drivetrainMechanism.pointAtTarget(targetTranslationSelector, SHOOTER_OFFSET_ANGLE),
                ledMechanism
                    .ledSegments(Color.GREEN, () -> shootingState.isAimReady, shooterMechanism::isFlywheelAtVelocity));

          // Wait until we're aimed and the shooter is ready
          while (!shootingState.isAimReady || !shooterMechanism.isFlywheelAtVelocity()) {
            // Calculate target information and update shooting state for forked commands
            var robotPose = robotPoseSupplier.get();
            var targetTranslation = targetTranslationSelector.apply(robotPoseSupplier.get().getTranslation());
            var headingToTarget = targetTranslation.minus(robotPose.getTranslation())
                .getAngle()
                .minus(SHOOTER_OFFSET_ANGLE);
            shootingState.isAimReady = Math.abs(
                headingToTarget.minus(robotPoseSupplier.get().getRotation()).getRadians()) <= AIM_TOLERANCE.in(Radians);
            var targetDistance = targetTranslation.getDistance(robotPose.getTranslation());
            shootingState.shooterSpeed = RotationsPerSecond.of(HUB_SETPOINTS_BY_DISTANCE_METERS.get(targetDistance));

            coroutine.yield();
          }

          // Ready! Shoot, using staggered mechanism starts
          coroutine.fork(ledMechanism.runPatternOnHalves(patternOne, patternTwo));
          coroutine.wait(Milliseconds.of(6.0));
          coroutine.fork(feederMechanism.feedShooter());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerMechanism.feedShooter());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(intakeMechanism.shootingSequence());
        })
        .named("Tune Shoot");
  }

}
