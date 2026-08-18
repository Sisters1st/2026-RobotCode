// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.LogFileUtil;

import edu.wpi.first.epilogue.logging.errors.LoggerDisabler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.Enums;
import frc.robot.Logic.TeleopController;
import frc.robot.Logic.Vision;
import frc.robot.Logic.Enums.AutoRoutines;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Hopper;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Kicker;
import frc.robot.Subsystems.Shooter;
import frc.robot.Logic.MatchTimeAnalysis;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */

// changes from TimedRobot to LoggedRobot for AdvantageKit
public class Robot extends LoggedRobot {

  public static Drivebase drivebase = new Drivebase();
  public static TeleopController teleop = new TeleopController();
  public static Vision vision = new Vision();
  public static Shooter shooter = new Shooter();
  public static Dashboard dashboard = new Dashboard();
  public static Intake intake = new Intake();
  public static Hopper hopper = new Hopper();
  public static Kicker kicker = new Kicker();
  public static Enums enums = new Enums();
  public static AutoController auto = new AutoController();
  public static MatchTimeAnalysis matchTimeAnalysis = new MatchTimeAnalysis();

  public Field2d robotCurrentPose = new Field2d();
  public AutoRoutines preLoadedAuto = AutoRoutines.DoNothing;

  public static boolean onRed = true;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  public Robot() {
    // Logger.recordMetadata("2026RobotCode", "initialization"); // Set a metadata value

    // if (isReal()) {
    //   Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    //   Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      
    // } else {
    //   setUseTiming(false); // Run as fast as possible
    //   String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //   System.out.println(logPath);
    //   Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //   Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // //                 // be added.

    DataLogManager.start();

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // var configs = new CurrentLimitsConfigs();
    // configs.StatorCurrentLimitEnable = true;
    // configs.StatorCurrentLimit = 60;
    // configs.SupplyCurrentLimitEnable = true;
    // configs.SupplyCurrentLimit = 40;

    // hopper.indexerMotorTop.getConfigurator().apply(configs);
    // hopper.indexerMotorBottom.getConfigurator().apply(configs);
    // intake.intakeDeployMotor.getConfigurator().apply(configs);
    // intake.intakeMotorLeftLeader.getConfigurator().apply(configs);
    // intake.intakeMotorRightFollower.getConfigurator().apply(configs);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(25);

    shooter.shooterHoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  @Override
  public void robotPeriodic() {
    if (Settings.VisionSettings.useVision) {
      vision.updatePhotonVision();
    }
    dashboard.updateDashboard();

    robotCurrentPose.setRobotPose(drivebase.yagslDrive.getPose());
    SmartDashboard.putData("Current Drivebase Position", robotCurrentPose);

    if (Robot.teleop.manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.resetIntake)) {
      Robot.intake.intakeFudgeFactor = 0;
      Robot.intake.intakeDeployMotor.setPosition(0);
      Robot.intake.intakeDeployController.reset(Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
    }
    
    // Logger.recordOutput("Logging time", Timer.getFPGATimestamp());

  }

  @Override
  public void autonomousInit() {
    updateAlliance();
    auto.initAuto();
  }

  @Override
  public void autonomousPeriodic() {
    auto.autoPeriodic();
  }

  @Override
  public void teleopInit() {
    updateAlliance();
    Robot.dashboard.updateTCPConnectionFromDashboard();
    teleop.initTele();

  }

  @Override
  public void teleopPeriodic() {
    teleop.driveTele();
    Robot.dashboard.updateTCPConnectionFromDashboard();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    auto.updateAutoRoutineFromDashboard();

    auto.autoRoutine = auto.dashboardAutoRoutine1;

    if (auto.autoRoutine != preLoadedAuto) {
      preLoadedAuto = auto.autoRoutine;
      auto.autoPeriodic();
    }

    auto.updateAutoRoutineFromDashboard();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public static void updateAlliance() {
    try {
      onRed = DriverStation.getAlliance().get() == Alliance.Red;
    } catch (Exception e) {
      Robot.dashboard.updateAllianceFromDashboard();
    }
  }
}
