package first.robot.opmodes;

import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_BLUE;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_RED;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;

import first.robot.CommandFactory;
import first.robot.Robot;
import first.robot.generated.TunerConstants;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Competition teleop mode
 */
@Teleop(name = "Xbox Controller", backgroundColor = "green", textColor = "black", description = "Xbox controller teleop mode")
public class XboxOpMode extends PeriodicOpMode {

  /** Max velocity the driver can request */
  protected static final LinearVelocity MAX_TELEOP_VELOCITY = TunerConstants.kSpeedAt12Volts;
  /** Max angular velocity the driver can request */
  protected static final AngularVelocity MAX_TELEOP_ANGULAR_VELOCITY = RotationsPerSecond.of(1.75);

  protected final Robot robot;

  protected final CommandGamepad driverGamepad = new CommandGamepad(0);

  protected final CommandFactory commandFactory;

  public XboxOpMode(Robot robot) {
    this.robot = robot;
    commandFactory = new CommandFactory(
        robot.drivetrain,
        robot.shooterMechanism,
        robot.indexerMechanism,
        robot.feederMechanism,
        robot.intakeMechanism,
        robot.ledMechanism);

    configureButtonBindings();
  }

  @Override
  public void start() {
    robot.intakeMechanism.setDefaultCommand(commandFactory.intakeWithLEDsCommand());
  }

  protected void configureButtonBindings() {
    // Default drivetrain command for teleop control
    robot.drivetrain.setDefaultCommand(
        commandFactory.drive(
            () -> MAX_TELEOP_VELOCITY.times(-squareAxis(driverGamepad.getLeftX())),
              () -> MAX_TELEOP_VELOCITY.times(-squareAxis(driverGamepad.getLeftY())),
              () -> MAX_TELEOP_ANGULAR_VELOCITY.times(-squareAxis(driverGamepad.getRightY())),
              MAX_TELEOP_VELOCITY,
              MAX_TELEOP_ANGULAR_VELOCITY));

    driverGamepad.rightTrigger().whileTrue(commandFactory.shootAtHub());

    driverGamepad.northFace().whileTrue(commandFactory.tuneShootCommand(() -> robot.drivetrain.getState().Pose));

    driverGamepad.southFace().whileTrue(commandFactory.justShootCommand(Meters.of(2.1)));

    driverGamepad.eastFace().whileTrue(commandFactory.shuttleToCorner());

    driverGamepad.rightBumper().onTrue(commandFactory.intakeWithLEDsCommand());

    driverGamepad.povLeft().onTrue(robot.intakeMechanism.deploy());

    driverGamepad.povRight().onTrue(robot.intakeMechanism.retract());

    driverGamepad.westFace().whileTrue(commandFactory.ejectCommand());

    driverGamepad.start().onTrue(Command.noRequirements(coroutine -> {
      Pose3d newPose = MatchState.getAlliance().orElse(BLUE) == BLUE ? RESET_POSE_BLUE : RESET_POSE_RED;
      robot.drivetrain.resetPose(newPose.toPose2d());
    }).named("Reset Field Position"));
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

}
