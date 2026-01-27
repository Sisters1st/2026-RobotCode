package frc.robot.Subsystems;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

// public class Drivebase {
//     public SwerveDrive yagslDrive;
//     public Rotation2d goalHeading = new Rotation2d();
//     public PIDController rotationPidController = new PIDController(frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kP,
//             frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kI,
//             frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kD);

//     public Drivebase() {
//         File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
//         try {
//             yagslDrive = new SwerveParser(swerveJsonDirectory)
//                     .createSwerveDrive(Settings.DrivebaseSettings.maxVelocityMPS,
//                             new Pose2d(8.25, 4, new Rotation2d(0)));
//         } catch (Exception e) {
//             e.printStackTrace();
//         }
//     }

//     public void drive(Translation2d translation2d, Rotation2d heading, boolean fR) {
//         double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(heading).getDegrees();
//         double rotationCorrection = rotationPidController.calculate(angleError, 0);

//         Robot.drivebase.yagslDrive.drive(translation2d,
//                 rotationCorrection,
//                 fR,
//                 false);

//     }

// }
