package first.robot;

import static first.robot.Constants.FieldConstants.FIELD_WIDTH;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import first.robot.mechanisms.IntakeMechanism;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public final class Constants {

  private Constants() {
  } // prevent instantiation

  public static final CANBus CANIVORE_BUS = new CANBus("canivore");
  // Robot dimensions INCLUDING bumpers
  public static final Distance ROBOT_WIDTH = Meters.of(0.932);
  public static final Distance ROBOT_LENGTH = Meters.of(0.776288);

  /**
   * Constants for the field dimensions of the WELDED field.
   */
  public static class FieldConstants {
    public static final Distance FIELD_LENGTH = Inches.of(651.2);
    public static final Distance FIELD_WIDTH = Inches.of(317.7);

    /**
     * Checks if a translation is within the field boundaries
     * 
     * @param translation The translation to check
     * @return true if the translation is within the field boundaries
     */
    public static boolean isValidFieldTranslation(Translation3d translation) {
      return isValidFieldTranslation(translation.toTranslation2d());
    }

    /**
     * Checks if a translation is within the field boundaries
     * 
     * @param translation The translation to check
     * @return true if the translation is within the field boundaries
     */
    public static boolean isValidFieldTranslation(Translation2d translation) {
      return translation.getX() >= 0.0 && translation.getX() <= FIELD_LENGTH.in(Meters) && translation.getY() >= 0.0
          && translation.getY() <= FIELD_WIDTH.in(Meters);
    }
  }

  /**
   * Constants for teleoperated driver control
   */
  public static class TeleopDriveConstants {

    /** Blue reset pose is the blue corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_BLUE = new Pose3d(
        new Translation3d(ROBOT_LENGTH.in(Meters) / 2.0, ROBOT_WIDTH.in(Meters) / 2.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI));
    /** Rotate around center with intake out */
    public static final Translation2d CENTER_OF_ROTATION = new Translation2d(Inches.of(6.0), Inches.zero());
    /** Red reset pose is the red corner, bumpers against the walls, facing downfield. */
    public static final Pose3d RESET_POSE_RED = new Pose3d(
        new Translation3d(FIELD_WIDTH.in(Meters) - RESET_POSE_BLUE.getX(), RESET_POSE_BLUE.getY(), 0.0),
        new Rotation3d(RESET_POSE_BLUE.getRotation().toRotation2d().minus(Rotation2d.kPi)));

  }

  /**
   * Constants for the shooter mechanism
   */
  /**
   * Constants related to shooting fuel
   * <p>
   * These constants are not specific to the shooter mechanism, they are about the process of shooting.
   */
  public static class ShootingConstants {
    public static final Angle AIM_TOLERANCE = Degrees.of(1.5);

    // Parameters for the intake shooting sequence
    public static final Time RETRACT_INTAKE_DELAY = Seconds.of(0.18);
    public static final Time JAM_DEBOUNCE_TIME = Seconds.of(0.5);
    public static final AngularVelocity JAM_THRESHOLD = RotationsPerSecond.of(-1.0);
    public static final Time UNJAM_DURATION = Seconds.of(0.25);
    public static final Angle RETRACTED_THRESHOLD = IntakeMechanism.RETRACTED_POSITION;

    private static InterpolatingDoubleTreeMap createShooterInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(1.77, 24.5);
      map.put(3.76, 30.5);
      map.put(5.42, 34.0);

      return map;
    }

    public static final InterpolatingDoubleTreeMap HUB_SETPOINTS_BY_DISTANCE_METERS = createShooterInterpolator();

    private static InterpolatingDoubleTreeMap createShuttleInterpolator() {
      var map = new InterpolatingDoubleTreeMap();
      map.put(2.0, 20.0);
      map.put(6.0, 40.0);
      return map;
    }

    public static final InterpolatingDoubleTreeMap SHUTTLE_SETPOINTS_BY_DISTANCE_METERS = createShuttleInterpolator();

    /** Translation of the hub on the blue side */
    public static final Translation2d HUB_BLUE = new Translation2d(Inches.of(182.143595), Inches.of(158.84375));

    /** Translation of the hub on the red side */
    public static final Translation2d HUB_RED = new Translation2d(Inches.of(469.078905), Inches.of(158.84375));

    /** Translations for shuttling on the blue side, with Y > 1/2 of the field */
    public static final Translation2d SHUTTLE_BLUE_HIGH = new Translation2d(
        Inches.of(2.0),
        FIELD_WIDTH.minus(Inches.of(42)));
    /** Translations for shuttling on the blue side, with Y < 1/2 of the field */
    public static final Translation2d SHUTTLE_BLUE_LOW = new Translation2d(Inches.of(2.0), Inches.of(42));

    /** Translations for shuttling on the red side, with Y > 1/2 of the field */
    public static final Translation2d SHUTTLE_RED_HIGH = new Translation2d(
        FIELD_WIDTH.in(Meters) - SHUTTLE_BLUE_HIGH.getX(),
        SHUTTLE_BLUE_HIGH.getY());

    /** Translations for shuttling on the red side, with Y < 1/2 of the field */
    public static final Translation2d SHUTTLE_RED_LOW = new Translation2d(
        FIELD_WIDTH.in(Meters) - SHUTTLE_BLUE_LOW.getX(),
        SHUTTLE_BLUE_LOW.getY());

    /**
     * The offset distance from the shuttle target where the robot will shoot. It will shoot offset distance short
     * (closer to the robot) of the target
     */
    public static final Distance SHUTTLE_OFFSET_DISTANCE = Meters.of(3.125);
  }
}
