package first.robot;

import static first.robot.Constants.FeederConstants.FEEDER_FEED_VELOCITY;
import static first.robot.Constants.FieldConstants.FIELD_WIDTH;
import static first.robot.Constants.IndexerConstants.INDEXER_FEED_VELOCITY;
import static first.robot.Constants.IntakeConstants.DEPLOY_SHOOTING_CURRENT;
import static first.robot.Constants.ShooterConstants.SHOOTER_OFFSET_ANGLE;
import static first.robot.Constants.ShootingConstants.AIM_TOLERANCE;
import static first.robot.Constants.ShootingConstants.HUB_BLUE;
import static first.robot.Constants.ShootingConstants.HUB_RED;
import static first.robot.Constants.ShootingConstants.HUB_SETPOINTS_BY_DISTANCE_METERS;
import static first.robot.Constants.ShootingConstants.JAM_DEBOUNCE_TIME;
import static first.robot.Constants.ShootingConstants.JAM_THRESHOLD;
import static first.robot.Constants.ShootingConstants.RETRACTED_THRESHOLD;
import static first.robot.Constants.ShootingConstants.RETRACT_INTAKE_DELAY;
import static first.robot.Constants.ShootingConstants.SHUTTLE_BLUE_HIGH;
import static first.robot.Constants.ShootingConstants.SHUTTLE_BLUE_LOW;
import static first.robot.Constants.ShootingConstants.SHUTTLE_OFFSET_DISTANCE;
import static first.robot.Constants.ShootingConstants.SHUTTLE_RED_HIGH;
import static first.robot.Constants.ShootingConstants.SHUTTLE_RED_LOW;
import static first.robot.Constants.ShootingConstants.UNJAM_DURATION;
import static first.robot.Constants.TeleopDriveConstants.CENTER_OF_ROTATION;
import static first.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static first.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
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
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.IntakeSubsytem;
import first.robot.subsystems.LEDSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import java.util.function.Function;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.MatchState;
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
 * Factory class for creating commands with the necessary subsystem dependencies.
 */
public class CommandFactory {
  private final CommandSwerveDrivetrain drivetrainSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final IntakeSubsytem intakeSubsystem;
  private final LEDSubsystem ledSubsystem;

  /**
   * Constructor for Commandfactory, takes in all subsystems as parameters to use in command creation methods.
   * 
   * @param drivetrainSubsystem drivetrain subsystem
   * @param shooterSubsystem shooter subsystem
   * @param indexerSubsystem indexer subsystem
   * @param feederSubsystem feeder subsystem
   * @param intakeSubsystem intake subsystem
   * @param ledSubsystem led subsystem
   */
  public CommandFactory(
      CommandSwerveDrivetrain drivetrainSubsystem,
      ShooterSubsystem shooterSubsystem,
      IndexerSubsystem indexerSubsystem,
      FeederSubsystem feederSubsystem,
      IntakeSubsytem intakeSubsystem,
      LEDSubsystem ledSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
  }

  /**
   * Creates a command to shoot at the hub
   * 
   * @return a new command to shoot at the hub
   */
  public Command shootAtHub() {
    return shootAtTarget(
        () -> drivetrainSubsystem.getState().Pose,
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
    return shootAtTarget(() -> drivetrainSubsystem.getState().Pose, targetTranslation -> {
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
      Supplier<AngularVelocity> omegaSupplier) {
    /* Setting up bindings for necessary control of the swerve drive platform */
    final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withCenterOfRotation(CENTER_OF_ROTATION)
        .withDeadband(MAX_TELEOP_VELOCITY.times(0.01))
        .withRotationalDeadband(MAX_TELEOP_ANGULAR_VELOCITY.times(0.01))
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    return drivetrainSubsystem.applyRequest(
        () -> drive.withVelocityX(translationXSupplier.get())
            .withVelocityY(translationYSupplier.get())
            .withRotationalRate(omegaSupplier.get()));
  }

  public Command ejectCommand() {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(
          intakeSubsystem.ejectCommand(),
            indexerSubsystem.ejectCommand(),
            feederSubsystem.ejectCommand(),
            shooterSubsystem.ejectCommand(),
            ledSubsystem.runPatternAsCommand(
                LEDPattern.rainbow(255, 255).scrollAtRelativeVelocity(Percent.per(Second).of(200))));
    }).named("Eject");
  }

  public Command intakeWithLEDsCommand() {
    final LEDPattern patternOne = LEDPattern.gradient(GradientType.DISCONTINUOUS, Color.BLACK, Color.ORANGE)
        .scrollAtRelativeVelocity(Percent.per(Second).of(200));
    final LEDPattern patternTwo = patternOne.reversed();
    return Command.noRequirements(coroutine -> {
      coroutine
          .awaitAll(intakeSubsystem.intakeCommand(), ledSubsystem.runPatternOnHalvesAsCommand(patternOne, patternTwo));
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

    return Command.requiring(intakeSubsystem, shooterSubsystem, feederSubsystem, indexerSubsystem)
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
              intakeSubsystem.stopCommand(),
                feederSubsystem.stopCommand(),
                indexerSubsystem.stopCommand(),
                shooterSubsystem.runFlywheelAtVelocityCommand(flywheelVelocity));

          // Wait until the flywheel is up to speed
          coroutine.waitUntil(shooterSubsystem::isFlywheelAtSpeed);
          coroutine.wait(Milliseconds.of(6.0));

          // Shoot
          coroutine.fork(feederSubsystem.runFeederCommand(feederVelocity));
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerSubsystem.runIndexerCommand(indexerVelocity));
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(
              intakeSubsystem.shootingSequence(
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
            intakeSubsystem,
              shooterSubsystem,
              feederSubsystem,
              indexerSubsystem,
              drivetrainSubsystem,
              ledSubsystem)
        .executing(coroutine -> {
          // Get ready to shoot
          coroutine.fork(
              intakeSubsystem.stopCommand(),
                feederSubsystem.stopCommand(),
                indexerSubsystem.stopCommand(),
                shooterSubsystem.runFlywheelAtVelocityCommand(shooterAngularVelocity),
                drivetrainSubsystem.applyRequest(() -> swerveBrakeRequest),
                ledSubsystem.offAscommand());

          // Wait until the flywheel is up to speed
          coroutine.waitUntil(shooterSubsystem::isFlywheelAtSpeed);

          // Shoot, using staggered mechanism starts
          coroutine.fork(ledSubsystem.runPatternOnHalvesAsCommand(patternOne, patternTwo));
          coroutine.wait(Milliseconds.of(6.0));
          coroutine.fork(feederSubsystem.feedShooterAsCommand());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerSubsystem.feedShooterAsCommand());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(intakeSubsystem.shootingSequence());
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
            intakeSubsystem,
              shooterSubsystem,
              feederSubsystem,
              indexerSubsystem,
              drivetrainSubsystem,
              ledSubsystem)
        .executing(coroutine -> {
          // Create object to share the shooting state with forked commands
          ShootingState shootingState = new ShootingState();

          // Fork the commands to prepare to shoot
          coroutine.fork(
              intakeSubsystem.stopCommand(),
                feederSubsystem.stopCommand(),
                indexerSubsystem.stopCommand(),
                shooterSubsystem.runFlywheelAtVelocityCommand(() -> shootingState.shooterSpeed),
                drivetrainSubsystem.pointAtTarget(targetTranslationSelector, SHOOTER_OFFSET_ANGLE),
                ledSubsystem.ledSegmentsAsCommand(
                    Color.GREEN,
                      () -> shootingState.isAimReady,
                      shooterSubsystem::isFlywheelAtSpeed));

          // Wait until we're aimed and the shooter is ready
          while (!shootingState.isAimReady || !shooterSubsystem.isFlywheelAtSpeed()) {
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
          coroutine.fork(ledSubsystem.runPatternOnHalvesAsCommand(patternOne, patternTwo));
          coroutine.wait(Milliseconds.of(6.0));
          coroutine.fork(feederSubsystem.feedShooterAsCommand());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(indexerSubsystem.feedShooterAsCommand());
          coroutine.wait(Milliseconds.of(12.0));
          coroutine.fork(intakeSubsystem.shootingSequence());
        })
        .named("Tune Shoot");
  }

}
