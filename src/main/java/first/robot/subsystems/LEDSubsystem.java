package first.robot.subsystems;

import static org.wpilib.hardware.led.LEDPattern.gradient;
import static org.wpilib.hardware.led.LEDPattern.kOff;
import static org.wpilib.hardware.led.LEDPattern.solid;
import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.AddressableLEDBufferView;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.system.RobotController;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Time;
import org.wpilib.util.Color;

/**
 * Subsystem for controlling the LEDs
 */
public class LEDSubsystem extends Mechanism {

  private static final int DEVICE_ID_LEDS = 0;
  private static final int LED_STRIP_LENGTH = 36;
  private static final int STRIP_COUNT = 2;

  public static final int TOTAL_LEDS = LED_STRIP_LENGTH * STRIP_COUNT;

  private final AddressableLED leds = new AddressableLED(DEVICE_ID_LEDS);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(TOTAL_LEDS);
  private final AddressableLEDBufferView frontStripBuffer;
  private final AddressableLEDBufferView backStripBuffer;
  private final AddressableLEDBufferView halfOneFront;
  private final AddressableLEDBufferView halfTwoFront;
  private final AddressableLEDBufferView halfOneBack;
  private final AddressableLEDBufferView halfTwoBack;

  /**
   * Creates a new LEDSubsystem
   */
  public LEDSubsystem() {
    leds.setLength(TOTAL_LEDS);
    leds.setData(ledBuffer);

    frontStripBuffer = new AddressableLEDBufferView(ledBuffer, LED_STRIP_LENGTH, 2 * LED_STRIP_LENGTH - 1).reversed();
    backStripBuffer = new AddressableLEDBufferView(ledBuffer, 0, LED_STRIP_LENGTH - 1);
    halfOneFront = new AddressableLEDBufferView(frontStripBuffer, 0, LED_STRIP_LENGTH / 2 - 1);
    halfTwoFront = new AddressableLEDBufferView(frontStripBuffer, LED_STRIP_LENGTH / 2, LED_STRIP_LENGTH - 1);
    halfOneBack = new AddressableLEDBufferView(backStripBuffer, 0, LED_STRIP_LENGTH / 2 - 1);
    halfTwoBack = new AddressableLEDBufferView(backStripBuffer, LED_STRIP_LENGTH / 2, LED_STRIP_LENGTH - 1);

    Scheduler.getDefault().addPeriodic(() -> leds.setData(ledBuffer));
  }

  /**
   * This will create a command that runs a pattern on each LED strip individually. This pattern will automatically
   * update as long as the the command is running, so this method only needs to be called one time for an animation. The
   * command will turn the LEDs off when it is completed.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   * @return A command that will run the pattern on each LED strip continuously
   */
  public Command runPatternAsCommand(LEDPattern pattern) {
    return run((coroutine) -> {
      while (true) {
        runPattern(pattern);
        coroutine.wait(Milliseconds.of(20)); // TODO this could be adjusted, but for now it maintains the old behavior
                                             // even if the robot period speeds up
      }
    }).whenCanceled(this::off).named("Run pattern: " + pattern);
  }

  /**
   * Applies the pattern to each LED strip individually. This will only happen once. If running an animation, this
   * method must be called continuously to update the led states.
   * 
   * @param pattern Pattern to set on each LED strip from bottom to top
   */
  private void runPattern(LEDPattern pattern) {
    pattern.applyTo(frontStripBuffer);
    pattern.applyTo(backStripBuffer);
  }

  /**
   * Applies the pattern to each half of each LED strip individually. This will only happen once. If running an
   * animation, this method must be called continuously to update the led states.
   * 
   * @param halfOnePattern Pattern to set on the first half of each LED strip
   * @param halfTwoPattern Pattern to set on the second half of each LED strip
   */
  private void runPatternOnHalves(LEDPattern halfOnePattern, LEDPattern halfTwoPattern) {
    halfOnePattern.applyTo(halfOneFront);
    halfTwoPattern.applyTo(halfTwoFront);
    halfOnePattern.applyTo(halfOneBack);
    halfTwoPattern.applyTo(halfTwoBack);
  }

  public Command runPatternOnHalvesAsCommand(LEDPattern halfOnePattern, LEDPattern halfTwoPattern) {
    return run(coroutine -> {
      while (true) {
        runPatternOnHalves(halfOnePattern, halfTwoPattern);
        coroutine.yield();
      }
    }).whenCanceled(this::off).named("Run pattern no halves: " + halfOnePattern + " - " + halfTwoPattern);
  }

  /**
   * Creates an LEDPattern that makes an animated candy cane effect (alternating colors)
   * 
   * @param color1 The first color
   * @param color2 The second color
   * @param period The length of time before swapping the colors
   */
  private static LEDPattern candyCane(Color color1, Color color2, Time period) {
    var periodMicros = (long) period.in(Microsecond);
    return (reader, writer) -> {
      var isOdd = (RobotController.getTime() / periodMicros) % 2 == 1;
      for (int led = 0; led < reader.getLength(); led++) {
        if (isOdd) {
          writer.setLED(led, led % 2 == 0 ? color1 : color2);
        } else {
          writer.setLED(led, led % 2 == 0 ? color2 : color1);
        }
      }
    };
  }

  /**
   * Lights up the LEDs in segments. Useful for indicating ready state, for example.
   *
   * @param color the color of the segments when lit
   * @param segmentValues array of boolean suppliers. The strip will be split into segments one segment for each element
   *          of the array.
   */
  public Command ledSegmentsAsCommand(Color color, BooleanSupplier... segmentValues) {
    return runPatternAsCommand((reader, writer) -> {
      final int ledsPerStatus = reader.getLength() / segmentValues.length;
      int ledIndex = 0;
      for (int segmentId = 0; segmentId < segmentValues.length; segmentId++) {
        for (; ledIndex < (ledsPerStatus * (segmentId + 1)); ledIndex++) {
          writer.setLED(ledIndex, segmentValues[segmentId].getAsBoolean() ? color : Color.BLACK);
        }
      }
    });
  }

  /**
   * Turns off the LEDs
   */
  private void off() {
    LEDPattern pattern = LEDPattern.kOff;
    pattern.applyTo(frontStripBuffer);
    pattern.applyTo(backStripBuffer);
  }

  public Command offAscommand() {
    return runPatternAsCommand(kOff);
  }

  public Command bootAnimation() {
    return run(coroutine -> {
      final int BLIP_SIZE = 5;
      int blipIndex = -1;
      boolean done = false;

      off();
      while (!done && RobotState.isDisabled()) {
        final int currentBlipIndex = blipIndex;
        runPattern((reader, writer) -> {
          for (int index = 0; index < reader.getLength(); index++) {
            if (index <= currentBlipIndex && index >= currentBlipIndex - (BLIP_SIZE - 1)) {
              writer.setLED(index, Color.ORANGE);
            } else {
              writer.setLED(index, Color.BLACK);
            }
          }
        });

        blipIndex++;
        done = blipIndex - (BLIP_SIZE + 1) >= LED_STRIP_LENGTH;
        coroutine.wait(Milliseconds.of(40));
      }
    }).named("Boot Animation");
  }

  public Command defaultCommand() {
    final LEDPattern dsDetachedPattern = candyCane(Color.DARK_RED, Color.INDIAN_RED, Seconds.of(0.5));
    final LEDPattern disabledPattern = gradient(GradientType.CONTINUOUS, Color.BLUE, Color.ORANGE)
        .scrollAtRelativeVelocity(Percent.per(Second).of(75));
    final LEDPattern enabledPatternOne = solid(Color.BLUE);
    final LEDPattern enabledPatternTwo = solid(Color.ORANGE);

    return run(coroutine -> {
      while (true) {
        if (!RobotState.isDSAttached()) {
          runPattern(dsDetachedPattern);
        } else if (RobotState.isDisabled()) {
          runPattern(disabledPattern);
        } else {
          if (Timer.getTimestamp() % 1 < 0.5) {
            runPatternOnHalves(enabledPatternOne, enabledPatternTwo);
          } else {
            runPatternOnHalves(enabledPatternTwo, enabledPatternOne);
          }
        }
        coroutine.yield();
      }
    }).named("Default Command");
  }

}