// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import first.robot.generated.TunerConstants;
import first.robot.mechanisms.CommandSwerveDrivetrain;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeMechanism;
import first.robot.mechanisms.LEDMechanism;
import first.robot.mechanisms.LocalizationMechanism;
import first.robot.mechanisms.ShooterMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.epilogue.Logged;
import org.wpilib.epilogue.Logged.Strategy;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.net.WebServer;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Filesystem;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends OpModeRobot {

  private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();

  // Create the drivetrain mechanism here instead of using TunerConstants.createDrivetrain() to set standard deviations
  // without editing generated TunerConstants file
  public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
      TunerConstants.DrivetrainConstants,
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight);

  public final FeederMechanism feederMechanism = new FeederMechanism();
  public final IndexerMechanism indexerMechanism = new IndexerMechanism();
  @Logged
  public final IntakeMechanism intakeMechanism = new IntakeMechanism();
  @Logged
  public final ShooterMechanism shooterMechanism = new ShooterMechanism();
  public final LEDMechanism ledMechanism = new LEDMechanism();
  public final LocalizationMechanism localizationMechanism = new LocalizationMechanism(
      drivetrain::addVisionMeasurement,
      drivetrain::getIMUYawVelocity);

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay().withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    SignalLogger.start(); // CTRE logger
    DataLogManager.start(); // WPILib logger
    DriverStation.startDataLog(DataLogManager.getLog()); // Record both DS control and joystick data
    // Epilogue.bind(this); // TODO figure out Epilogue

    // Webserver for Elastic layout
    // See https://frc-elastic.gitbook.io/docs/additional-features-and-references/remote-layout-downloading
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);

    ledMechanism.setDefaultCommand(ledMechanism.defaultCommand());
    Scheduler.getDefault().schedule(ledMechanism.bootAnimation());

    feederMechanism.setDefaultCommand(feederMechanism.stop());
    indexerMechanism.setDefaultCommand(indexerMechanism.stop());
    shooterMechanism.setDefaultCommand(shooterMechanism.stop());
    intakeMechanism.setDefaultCommand(intakeMechanism.stop());
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    Scheduler.getDefault().run();
  }
}
