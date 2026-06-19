package frc.robot.commands.led;

import frc.robot.subsystems.LEDSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import org.wpilib.util.Color;

/** Command to run the boot animation on the LED strips */
public class LEDBootAnimationCommand extends Command {

  private static final int BLIP_SIZE = 5;
  private final LEDSubsystem intakeLedSubsystem;
  private final Timer timer = new Timer();

  private int blipIndex = -1;
  private boolean done = false;
  private boolean initialized = false;

  public LEDBootAnimationCommand(LEDSubsystem intakeLedSubsystem) {
    this.intakeLedSubsystem = intakeLedSubsystem;

    addRequirements(intakeLedSubsystem);
  }

  @Override
  public void initialize() {
    intakeLedSubsystem.off();
    blipIndex = -1;
    timer.start();
    done = false;
    initialized = false;
  }

  @Override
  public void execute() {
    if (timer.advanceIfElapsed(0.04) || !initialized) {
      if (!initialized) {
        intakeLedSubsystem.off();
        initialized = true;
      }
      intakeLedSubsystem.runPattern((reader, writer) -> {
        for (int index = 0; index < reader.getLength(); index++) {
          if (index <= blipIndex && index >= blipIndex - (BLIP_SIZE - 1)) {
            writer.setLED(index, Color.kOrange);
          } else {
            writer.setLED(index, Color.kBlue);
          }
        }
      });

      blipIndex++;
      done = blipIndex - (BLIP_SIZE + 1) >= intakeLedSubsystem.getStripLength();
    }
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    intakeLedSubsystem.off();
  }
}
