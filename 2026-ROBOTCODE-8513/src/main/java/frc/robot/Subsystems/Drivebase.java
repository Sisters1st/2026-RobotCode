package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Logic.Settings;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivebase {

    public SwerveDrive yagslDrive;

    public Drivebase() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            yagslDrive = new SwerveParser(swerveJsonDirectory)
                            .createSwerveDrive(Settings.DrivebaseSettings.maxVelocityMPS, 
                                                new Pose2d(8.25, 4, new Rotation2d(0)));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
