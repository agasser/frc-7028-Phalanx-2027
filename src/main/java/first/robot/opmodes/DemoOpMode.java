package first.robot.opmodes;

import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_BLUE;
import static first.robot.Constants.TeleopDriveConstants.RESET_POSE_RED;
import static org.wpilib.driverstation.Alliance.BLUE;
import static org.wpilib.units.Units.RotationsPerSecond;

import first.robot.CommandFactory;
import first.robot.Robot;
import first.robot.generated.TunerConstants;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandJoystick;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Competition teleop mode
 */
@Teleop(name = "Demo", backgroundColor = "blue", textColor = "yellow", description = "Demo teleop mode")
public class DemoOpMode extends PeriodicOpMode {

  /** Max velocity the driver can request */
  private static final LinearVelocity MAX_DEMO_VELOCITY = TunerConstants.kSpeedAt12Volts.times(0.25);
  /** Max angular velocity the driver can request */
  private static final AngularVelocity MAX_DEMO_ANGULAR_VELOCITY = RotationsPerSecond.of(0.5);

  protected final Robot robot;

  protected final CommandJoystick leftJoystick = new CommandJoystick(0);
  protected final CommandJoystick rightJoystick = new CommandJoystick(1);

  protected final CommandFactory commandFactory;

  public DemoOpMode(Robot robot) {
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

  protected void configureButtonBindings() {
    // Default drivetrain command for teleop control
    robot.drivetrain.setDefaultCommand(
        commandFactory.drive(
            () -> MAX_DEMO_VELOCITY.times(-squareAxis(leftJoystick.getY())),
              () -> MAX_DEMO_VELOCITY.times(-squareAxis(leftJoystick.getX())),
              () -> MAX_DEMO_ANGULAR_VELOCITY.times(-squareAxis(rightJoystick.getX())),
              MAX_DEMO_VELOCITY,
              MAX_DEMO_ANGULAR_VELOCITY));

    rightJoystick.trigger().whileTrue(commandFactory.demoToss());

    leftJoystick.button(4).onTrue(commandFactory.intakeWithLEDsCommand());

    leftJoystick.button(3).onTrue(robot.intakeMechanism.stop());

    leftJoystick.trigger().onTrue(robot.intakeMechanism.deploy());

    leftJoystick.button(2).onTrue(robot.intakeMechanism.retract());

    leftJoystick.povDown().whileTrue(commandFactory.ejectCommand());

    leftJoystick.button(11).onTrue(Command.noRequirements(coroutine -> {
      Pose3d newPose = MatchState.getAlliance().orElse(BLUE) == BLUE ? RESET_POSE_BLUE : RESET_POSE_RED;
      robot.drivetrain.resetPose(newPose.toPose2d());
    }).named("Reset Field Position"));
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

}
