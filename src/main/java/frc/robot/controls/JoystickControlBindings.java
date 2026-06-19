package frc.robot.controls;

import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_ANGULAR_VELOCITY;
import static frc.robot.Constants.TeleopDriveConstants.MAX_TELEOP_VELOCITY;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.Trigger;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MutAngularVelocity;
import org.wpilib.units.measure.MutLinearVelocity;

/** Control bindings for driving with joysticks */
public class JoystickControlBindings extends ControlBindings {

  private final CommandJoystick leftJoystick = new CommandJoystick(0);
  private final CommandJoystick rightJoystick = new CommandJoystick(1);

  private final MutLinearVelocity translationX = MetersPerSecond.mutable(0);
  private final MutLinearVelocity translationY = MetersPerSecond.mutable(0);
  private final MutAngularVelocity omega = RadiansPerSecond.mutable(0);

  @Override
  public Supplier<LinearVelocity> translationX() {
    return () -> translationX
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getY()), MetersPerSecond);
  }

  @Override
  public Supplier<LinearVelocity> translationY() {
    return () -> translationY
        .mut_replace(MAX_TELEOP_VELOCITY.in(MetersPerSecond) * -squareAxis(leftJoystick.getX()), MetersPerSecond);
  }

  @Override
  public Supplier<AngularVelocity> omega() {
    return () -> omega.mut_replace(
        MAX_TELEOP_ANGULAR_VELOCITY.in(RadiansPerSecond) * -squareAxis(rightJoystick.getX()),
          RadiansPerSecond);
  }

  private static double squareAxis(double value) {
    return Math.copySign(value * value, value);
  }

  @Override
  public Optional<Trigger> autoShoot() {
    return Optional.of(rightJoystick.trigger());
  }

  @Override
  public Optional<Trigger> runIntake() {
    return Optional.of(leftJoystick.button(4));
  }

  @Override
  public Optional<Trigger> stopIntake() {
    return Optional.of(leftJoystick.button(3));
  }

  @Override
  public Optional<Trigger> deployIntake() {
    return Optional.of(leftJoystick.trigger());
  }

  @Override
  public Optional<Trigger> retractIntake() {
    return Optional.of(leftJoystick.button(2));
  }

  @Override
  public Optional<Trigger> eject() {
    return Optional.of(leftJoystick.povDown());
  }

  @Override
  public Optional<Trigger> manualShoot() {
    return Optional.of(rightJoystick.povUp());
  }

  @Override
  public Optional<Trigger> tuneShoot() {
    return Optional.of(rightJoystick.povDown());
  }

  @Override
  public Optional<Trigger> shuttle() {
    return Optional.of(rightJoystick.button(3));
  }

  @Override
  public Optional<Trigger> resetFieldPosition() {
    return Optional.of(leftJoystick.button(11));
  }

}