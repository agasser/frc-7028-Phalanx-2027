package frc.robot.controls;

import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MutAngularVelocity;
import org.wpilib.units.measure.MutLinearVelocity;

/** Control bindings for an Xbox controller */
public class XBoxControlBindings extends ControlBindings {

  private final CommandXboxController driverController = new CommandXboxController(0);

  // Mutable measure objects to reduce memory pressure by not creating new instances on each iteration
  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX.mut_replace(
        MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftY()),
          MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY.mut_replace(
        MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(driverController.getLeftX()),
          MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_TELEOP_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(driverController.getRightX()),
          RadiansPerSecond);
  }

  @Override
  public Optional<Trigger> resetFieldPosition() {
    return Optional.of(driverController.start());
  }

  @Override
  public Optional<Trigger> runIntake() {
    return Optional.of(driverController.rightBumper());
  }

  @Override
  public Optional<Trigger> stopIntake() {
    return Optional.of(driverController.leftBumper());
  }

  @Override
  public Optional<Trigger> eject() {
    return Optional.of(driverController.x());
  }

  @Override
  public Optional<Trigger> deployIntake() {
    return Optional.of(driverController.povLeft());
  }

  @Override
  public Optional<Trigger> retractIntake() {
    return Optional.of(driverController.povRight());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.of(driverController.a());
  }

  @Override
  public Optional<Trigger> autoShoot() {
    return Optional.of(driverController.rightTrigger());
  }

  @Override
  public Optional<Trigger> tuneShoot() {
    return Optional.of(driverController.y());
  }

  @Override
  public Optional<Trigger> shuttle() {
    return Optional.of(driverController.b());
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

}