package frc.robot.commands.led;

import static frc.robot.subsystems.LEDSubsystem.candyCane;
import static org.wpilib.hardware.led.LEDPattern.gradient;
import static org.wpilib.hardware.led.LEDPattern.solid;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.util.Color.kBlack;
import static org.wpilib.util.Color.kBlue;
import static org.wpilib.util.Color.kDarkRed;
import static org.wpilib.util.Color.kIndianRed;
import static org.wpilib.util.Color.kOrange;

import frc.robot.subsystems.LEDSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.framework.RobotState;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Color;

/**
 * The default command for controlling the LEDs
 */
public class DefaultLEDCommand extends Command {

  private static final Time CANDY_CANE_SPEED = Seconds.of(0.5);
  private static final LEDPattern dsDetachedPattern = candyCane(kDarkRed, kIndianRed, CANDY_CANE_SPEED);
  private static final LEDPattern disabledPattern = gradient(kContinuous, kBlue, kOrange)
      .scrollAtRelativeSpeed(Percent.per(Second).of(75));
  private static final LEDPattern testModePattern = candyCane(kOrange, kBlack, CANDY_CANE_SPEED);
  private static final LEDPattern enabledPatternOne = solid(Color.kBlue);
  private static final LEDPattern enabledPatternTwo = solid(Color.kOrange);

  private final LEDSubsystem ledSubsystem;

  /**
   * Creates a new DefaultLEDCommand
   * 
   * @param ledSubsystem LED subsystem
   */
  public DefaultLEDCommand(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  @Override
  public void execute() {
    if (!DriverStation.isDSAttached()) {
      ledSubsystem.runPattern(dsDetachedPattern);
    } else if (RobotState.isDisabled()) {
      ledSubsystem.runPattern(disabledPattern);
    } else if (RobotState.isTest()) {
      ledSubsystem.runPattern(testModePattern);
    } else {
      if (Timer.getTimestamp() % 1 < 0.5) {
        ledSubsystem.runPatternOnHalves(enabledPatternOne, enabledPatternTwo);
      } else {
        ledSubsystem.runPatternOnHalves(enabledPatternTwo, enabledPatternOne);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    ledSubsystem.off();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
