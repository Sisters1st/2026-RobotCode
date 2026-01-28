// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logic.AutoController;
import frc.robot.Logic.Dashboard;
import frc.robot.Logic.Enums;
import frc.robot.Logic.TeleopController;
import frc.robot.Logic.Vision;
import frc.robot.Subsystems.Drivebase;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  
  public static Drivebase drivebase = new Drivebase();
  public static TeleopController teleop = new TeleopController();
  public static Vision vision = new Vision();
  public static Shooter shooter = new Shooter();
  public static Dashboard dashboard = new Dashboard();
  public static Intake intake = new Intake();
  public static Enums enums = new Enums();
  public static AutoController auto = new AutoController();

  public Field2d robotCurrentPose = new Field2d();

  public SparkMax intakeMotorRight = new SparkMax(52, MotorType.kBrushless);
  public SparkMax intakeMotorLeft = new SparkMax(51, MotorType.kBrushless);


  public static boolean onRed = true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  public Robot() {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  @Override
  public void robotPeriodic() {
      if (Robot.isReal()) {
        vision.updatePhotonVision();
      }
      dashboard.updateDashboard();

      robotCurrentPose.setRobotPose(drivebase.yagslDrive.getPose());
      SmartDashboard.putData("Current Drivebase Position", robotCurrentPose);

  }

  @Override
  public void autonomousInit() {
    auto.initAuto();
  }

  @Override
  public void autonomousPeriodic() {
    auto.runAuto();
  }

  @Override
  public void teleopInit() {
    teleop.initTele();
  }

  @Override
  public void teleopPeriodic() {
    teleop.driveTele();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public static void updateAlliance(){
    try {
      onRed = DriverStation.getAlliance().get() == Alliance.Red;
    } catch (Exception e) {
      onRed = true;
    }    
  }
}

