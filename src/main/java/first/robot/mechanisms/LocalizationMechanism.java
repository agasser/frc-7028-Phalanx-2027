package first.robot.mechanisms;

import static first.robot.Constants.FieldConstants.isValidFieldTranslation;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.math.util.Units.degreesToRadians;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import first.robot.LimelightHelpers;
import first.robot.LimelightHelpers.PoseEstimate;
import first.robot.VisionMeasurementConsumer;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

/**
 * Mechanism for the localization system.
 * <p>
 * This mechanism integrates Limelight and QuestNav vision systems to provide pose estimation and field localization.
 * It manages camera pose configuration, vision measurement consumption, and field position validation.
 * Vision measurements are fused from multiple sources and published for use by other mechanisms.
 */
public class LocalizationMechanism extends Mechanism {
  private static final String[] APRILTAG_CAMERA_NAMES = { "limelight-left", "limelight-right", "limelight-back" };
  private static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = new Transform3d[] {
      new Transform3d(
          new Translation3d(Inches.of(-6.230084), Inches.of(-14.513163), Inches.of(15.352343)),
          new Rotation3d(0.0, degreesToRadians(16.0), Math.PI / 2.0)),
      new Transform3d(
          new Translation3d(Inches.of(-6.230084), Inches.of(14.513163), Inches.of(15.352343)),
          new Rotation3d(0.0, degreesToRadians(16.0), -Math.PI / 2.0)),
      new Transform3d(
          new Translation3d(Inches.of(-11.1515), Inches.of(6.75), Inches.of(10.012)),
          new Rotation3d(0.0, degreesToRadians(26.0), Math.PI)) };

  private static final int LIMELIGHT_BLUE_PIPELINE = 0;
  private static final int LIMELIGHT_RED_PIPELINE = 1;

  private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2.0, 2.0, 8.0);
  private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
  private static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.3;

  private static final Distance TAG_DISTANCE_THRESHOLD = Meters.of(3.5);
  private static final Distance STARTING_DISTANCE_THRESHOLD = Meters.of(3.0);
  private static final AngularVelocity ANGULAR_VELOCITY_THRESHOLD = DegreesPerSecond.of(720);

  private final VisionMeasurementConsumer visionMeasurementConsumer;
  private final Supplier<AngularVelocity> robotAngularVelocitySupplier;
  private final Map<String, StructPublisher<Pose3d>> visionPosePublishers;

  /**
   * Constructs a new LocalizationMechanism.
   *
   * @param addVisionMeasurement the consumer for vision-based pose measurements
   * @param poseResetConsumer the consumer for resetting the robot's pose when the robot is disabled
   * @param angularVelocitySupplier supplier for the robot's current angular velocity, NOT from the fused estimator but
   *          directly from the IMU
   */
  public LocalizationMechanism(
      VisionMeasurementConsumer addVisionMeasurement,
      Supplier<AngularVelocity> angularVelocitySupplier) {
    this.visionMeasurementConsumer = addVisionMeasurement;
    this.robotAngularVelocitySupplier = angularVelocitySupplier;

    var table = NetworkTableInstance.getDefault().getTable("Localization");
    visionPosePublishers = new HashMap<>();
    for (int i = 0; i < APRILTAG_CAMERA_NAMES.length; i++) {
      String cameraName = APRILTAG_CAMERA_NAMES[i];
      visionPosePublishers.put(cameraName, table.getStructTopic(cameraName, Pose3d.struct).publish());
      LimelightHelpers.setCameraPose_RobotSpace(
          cameraName,
            ROBOT_TO_CAMERA_TRANSFORMS[i].getX(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getY(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getZ(),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureX().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureY().in(Degrees),
            ROBOT_TO_CAMERA_TRANSFORMS[i].getRotation().getMeasureZ().in(Degrees));
    }

    Scheduler.getDefault().addPeriodic(() -> {
      if (RobotState.isDisabled()) {
        periodicDisabled();
      }
      periodicLimelight();
    });
  }

  /**
   * Validates a pose estimate based on common criteria.
   * <p>
   * Checks if the pose has valid tag count, is within field boundaries, and is within the tag distance threshold.
   *
   * @param poseEstimate the pose estimate to validate
   * @return true if the pose estimate meets basic validation criteria
   */
  private boolean isValidPoseEstimate(PoseEstimate poseEstimate) {
    return poseEstimate != null
        && (poseEstimate.tagCount >= 2
            || (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials[0].ambiguity <= APRILTAG_AMBIGUITY_THRESHOLD))
        && isValidFieldTranslation(poseEstimate.pose.getTranslation())
        && poseEstimate.avgTagDist < TAG_DISTANCE_THRESHOLD.in(Meters);
  }

  /**
   * Handles periodic updates when the robot is disabled.
   */
  private void periodicDisabled() {
    boolean isBlueAlliance = MatchState.getAlliance().orElse(BLUE) == BLUE;
    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      // Set the pipeline based on alliance color
      if (isBlueAlliance) {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_BLUE_PIPELINE);
      } else {
        LimelightHelpers.setPipelineIndex(cameraName, LIMELIGHT_RED_PIPELINE);
      }
    }
  }

  /**
   * Handles periodic updates when the robot is enabled.
   * <p>
   * Processes all AprilTag cameras, validates poses, adds vision measurements (when QuestNav has failed),
   * and returns the best pose estimate for comparison with QuestNav.
   *
   * @return the best validated pose estimate from all cameras, or null if no valid estimates
   */
  private void periodicLimelight() {
    AngularVelocity robotAngularVelocity = robotAngularVelocitySupplier.get();
    if (robotAngularVelocity.lt(ANGULAR_VELOCITY_THRESHOLD.unaryMinus())
        || robotAngularVelocity.gt(ANGULAR_VELOCITY_THRESHOLD)) {
      // Spinnning too fast to trust vision
      return;
    }

    for (String cameraName : APRILTAG_CAMERA_NAMES) {
      PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
      if (isValidPoseEstimate(poseEstimate)) {
        visionPosePublishers.get(cameraName).set(poseEstimate.pose);
        visionMeasurementConsumer.addVisionMeasurement(
            poseEstimate.pose.toPose2d(),
              poseEstimate.timestampSeconds,
              poseEstimate.tagCount == 1 ? SINGLE_TAG_STD_DEVS : MULTI_TAG_STD_DEVS);
      }
    }
  }

}
