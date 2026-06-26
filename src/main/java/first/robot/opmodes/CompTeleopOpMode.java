package first.robot.opmodes;

import static first.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static first.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_BLUE;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_RED;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.units.Units.Meters;

import first.robot.CommandFactory;
import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandJoystick;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Competition teleop mode
 */
@Teleop(name = "Comp", backgroundColor = "blue", textColor = "yellow", description = "Main competition teleop mode")
public class CompTeleopOpMode extends PeriodicOpMode {

  protected final Robot robot;

  protected final CommandJoystick leftJoystick = new CommandJoystick(0);
  protected final CommandJoystick rightJoystick = new CommandJoystick(1);

  protected final CommandFactory commandFactory;

  public CompTeleopOpMode(Robot robot) {
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
            () -> MAX_TELEOP_VELOCITY.times(-squareAxis(leftJoystick.getY())),
              () -> MAX_TELEOP_VELOCITY.times(-squareAxis(leftJoystick.getX())),
              () -> MAX_TELEOP_ANGULAR_VELOCITY.times(-squareAxis(rightJoystick.getX()))));

    rightJoystick.trigger().whileTrue(commandFactory.shootAtHub());

    leftJoystick.button(4).onTrue(commandFactory.intakeWithLEDsCommand());

    leftJoystick.button(3).onTrue(robot.intakeSubsystem.stop());

    leftJoystick.trigger().onTrue(robot.intakeSubsystem.deploy());

    leftJoystick.button(2).onTrue(robot.intakeSubsystem.retract());

    leftJoystick.povDown().whileTrue(commandFactory.ejectCommand());

    rightJoystick.povUp().whileTrue(commandFactory.justShootCommand(Meters.of(2.1)));

    rightJoystick.button(3).whileTrue(commandFactory.shuttleToCorner());

    leftJoystick.button(11).onTrue(Command.noRequirements(coroutine -> {
      Pose3d newPose = MatchState.getAlliance().orElse(BLUE) == BLUE ? RESET_POSE_BLUE : RESET_POSE_RED;
      robot.drivetrain.resetPose(newPose.toPose2d());
    }).named("Reset Field Position"));
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

}
