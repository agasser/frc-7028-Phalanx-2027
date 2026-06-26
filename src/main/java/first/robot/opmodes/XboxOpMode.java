package first.robot.opmodes;

import static first.robot.Constants.TeleopDriveConstants.MAX_DEMO_ANGULAR_VELOCITY;
import static first.robot.Constants.TeleopDriveConstants.MAX_DEMO_VELOCITY;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_BLUE;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_RED;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.units.Units.Meters;

import first.robot.CommandFactory;
import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Competition teleop mode
 */
@Teleop(name = "Xbox Controller", backgroundColor = "green", textColor = "black", description = "Xbox controller teleop mode")
public class XboxOpMode extends PeriodicOpMode {

  protected final Robot robot;

  protected final CommandGamepad driverGamepad = new CommandGamepad(0);

  protected final CommandFactory commandFactory;

  public XboxOpMode(Robot robot) {
    this.robot = robot;
    commandFactory = new CommandFactory(
        robot.drivetrain,
        robot.shooterSubsystem,
        robot.indexerSubsystem,
        robot.feederSubsystem,
        robot.intakeSubsystem,
        robot.ledSubsystem);

    configureButtonBindings();
    robot.intakeSubsystem.setDefaultCommand(commandFactory.intakeWithLEDsCommand());
  }

  protected void configureButtonBindings() {
    // Default drivetrain command for teleop control
    robot.drivetrain.setDefaultCommand(
        commandFactory.drive(
            () -> MAX_DEMO_VELOCITY.times(-squareAxis(driverGamepad.getLeftX())),
              () -> MAX_DEMO_VELOCITY.times(-squareAxis(driverGamepad.getLeftY())),
              () -> MAX_DEMO_ANGULAR_VELOCITY.times(-squareAxis(driverGamepad.getRightY()))));

    driverGamepad.rightTrigger().whileTrue(commandFactory.shootAtHub());

    driverGamepad.northFace().whileTrue(commandFactory.tuneShootCommand(() -> robot.drivetrain.getState().Pose));

    driverGamepad.southFace().whileTrue(commandFactory.justShootCommand(Meters.of(2.1)));

    driverGamepad.eastFace().whileTrue(commandFactory.shuttleToCorner());

    driverGamepad.rightBumper().onTrue(commandFactory.intakeWithLEDsCommand());

    driverGamepad.povLeft().onTrue(robot.intakeSubsystem.deploy());

    driverGamepad.povRight().onTrue(robot.intakeSubsystem.retract());

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
