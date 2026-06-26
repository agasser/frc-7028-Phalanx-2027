package first.robot.opmodes;

import first.robot.Robot;
import org.wpilib.opmode.Teleop;

/**
 * OpMode that extends the comp op mode and adds shop functions.
 */
@Teleop(name = "Shop", backgroundColor = "purple", textColor = "orange", description = "Teleop mode for practicing in the shop")
public class ShopOpMode extends CompTeleopOpMode {

  public ShopOpMode(Robot robot) {
    super(robot);
  }

  @Override
  protected void configureButtonBindings() {
    // Comp bindings
    super.configureButtonBindings();

    // Shop bindings
    rightJoystick.povDown().whileTrue(commandFactory.tuneShootCommand(() -> robot.drivetrain.getState().Pose));
  }
}
