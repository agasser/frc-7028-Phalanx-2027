package frc.robot.commands;

import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Second;

import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.LEDSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDPattern.GradientType;
import org.wpilib.util.Color;

/**
 * Command to intake fuel from the floor by deploying the intake and running the roller.
 */
public class IntakeCommand extends Command {

  private final IntakeSubsytem intakeSubsytem;
  private final LEDSubsystem ledSubsystem;
  private final LEDPattern patternOne = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlack, Color.kOrange)
      .scrollAtRelativeSpeed(Percent.per(Second).of(200));
  private final LEDPattern patternTwo = patternOne.reversed();

  private boolean hasDeployed = false;

  /**
   * Constructor for IntakeCommand
   * 
   * @param intakeSubsytem the intake subsystem
   * @param intakeLEDSubsystem the intake LED subsystem
   */
  public IntakeCommand(IntakeSubsytem intakeSubsytem, LEDSubsystem ledSubsystem) {
    this.intakeSubsytem = intakeSubsytem;
    this.ledSubsystem = ledSubsystem;

    addRequirements(intakeSubsytem, ledSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsytem.runIntake();
    intakeSubsytem.deploy();
    hasDeployed = false;
  }

  @Override
  public void execute() {
    if (hasDeployed || intakeSubsytem.isDeployed()) {
      hasDeployed = true;
      intakeSubsytem.stopDeploy();
    }
    ledSubsystem.runPatternOnHalves(patternOne, patternTwo);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsytem.stop();
    ledSubsystem.off();
  }

}
