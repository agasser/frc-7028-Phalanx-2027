package frc.robot;

import static frc.robot.Constants.CANIVORE_BUS;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;

/** Telemeterize drivetrain information to NetworkTables and Hoot log. */
public class DrivetrainTelemetry {
  // Limit telemetry updates to prevent flooding network and Shuffleboard
  private static final double PUBLISH_FREQUENCY = 0.04;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Robot speeds for general checking */
  private final NetworkTable driveStats = inst.getTable("Drive");

  private final StructPublisher<Pose2d> odometryPublisher = driveStats.getStructTopic("odometry", Pose2d.struct)
      .publish();
  private final StructArrayPublisher<SwerveModuleVelocity> moduleStatePublisher = driveStats
      .getStructArrayTopic("Module States", SwerveModuleVelocity.struct)
      .publish();
  private final StructArrayPublisher<SwerveModuleVelocity> moduleTargetsPublisher = driveStats
      .getStructArrayTopic("Module Targets", SwerveModuleVelocity.struct)
      .publish();
  private final DoublePublisher periodPublisher = driveStats.getDoubleTopic("Period").publish();

  private final DoublePublisher xSpeedPublisher = driveStats.getDoubleTopic("xSpeed").publish();
  private final DoublePublisher ySpeedPublisher = driveStats.getDoubleTopic("ySpeed").publish();

  private final BooleanPublisher canBusStatusPublisher = driveStats.getBooleanTopic("canivore").publish();

  private final Field2d field2d = new Field2d();

  private final Timer frequencyTimer = new Timer();

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   */
  public DrivetrainTelemetry() {
    frequencyTimer.start();
    SmartDashboard.putData(field2d);
  }

  /* Accept the swerve drive state and telemeterize it to NetworkTables */
  public void telemeterize(SwerveDriveState state) {
    if (frequencyTimer.advanceIfElapsed(PUBLISH_FREQUENCY)) {
      /* Telemeterize the pose */
      odometryPublisher.set(state.Pose, (long) (Timer.getTimestamp()));

      // Publish module states and targets
      moduleStatePublisher.set(state.ModuleVelocities);
      moduleTargetsPublisher.set(state.ModuleTargets);
      periodPublisher.accept(state.OdometryPeriod);
      field2d.setRobotPose(state.Pose);
      xSpeedPublisher.set(state.Velocity.vx);
      ySpeedPublisher.set(state.Velocity.vy);

      /* Also write to hoot log file */
      SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
      SignalLogger.writeStruct("DriveState/Speeds", ChassisVelocities.struct, state.Velocity);
      SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleVelocity.struct, state.ModuleVelocities);
      SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleVelocity.struct, state.ModuleVelocities);
      SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
      SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

      // Canbus status
      canBusStatusPublisher.set(CANIVORE_BUS.getStatus().Status == StatusCode.OK);
    }
  }

}
