package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Settings;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

public class Drivebase {
    public SwerveDrive yagslDrive;
    public Rotation2d goalHeading = new Rotation2d();
    public PIDController rotationPidController = new PIDController(
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kP,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kI,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kD);

    // path following variables
    PathPlannerTrajectory traj;
    boolean loadedPathHasStarted = false;
    PathPlannerPath path;
    public String pathName = "";
    double elapsedTime;
    double timePathStarted;
    public PathPlannerTrajectoryState trajGoalState = new PathPlannerTrajectoryState();
    Field2d trajGoalPosition = new Field2d();
    double otfEndVelocity = 0;
    public boolean forcePathHeading = false;
    double dvr;

    // edit these values and put in settings later
    public PIDController followPathXController = new PIDController(10, 0, 0);

    public PIDController followPathYController = new PIDController(10, 0, 0);

    public Drivebase() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            yagslDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(Settings.DrivebaseSettings.maxVelocityMPS,
                            new Pose2d(8.25, 4, new Rotation2d(0)));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void driveFacingHeading(Translation2d translation2d, Rotation2d heading, boolean fR) {
        double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(heading).getDegrees();
        double rotationCorrection = rotationPidController.calculate(angleError, 0);

        Robot.drivebase.yagslDrive.drive(translation2d,
                rotationCorrection,
                fR,
                false);

    }

    public void initPath(String pathNameIn) {
        pathName = pathNameIn;

        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            System.out.println("Error in loading path");
            e.printStackTrace();
        }

        // flip the path if on red
        if (Robot.onRed) {
            path = path.flipPath();
        }

        try {
            traj = path.getIdealTrajectory(RobotConfig.fromGUISettings()).get();
        } catch (Exception e) {
            System.out.println("Error in trajectory generation");
            e.printStackTrace();
        }

        if (Robot.isSimulation()) {
            yagslDrive.resetOdometry(path.getStartingHolonomicPose().get());
        }

        loadedPathHasStarted = false;

    }

    public boolean followLoadedPath() {
        double correctionInXV;
        double correctionInYV;

        if (!loadedPathHasStarted) {
            timePathStarted = Timer.getFPGATimestamp();
            loadedPathHasStarted = true;
        }

        elapsedTime = Timer.getFPGATimestamp() - timePathStarted;

        if (elapsedTime > traj.getTotalTimeSeconds()) {
            return true;
        } else {
            PathPlannerTrajectoryState pathGoalState = traj.sample(elapsedTime);

            double trajGoalX = pathGoalState.pose.getX();
            double trajGoalY = pathGoalState.pose.getY();
            Rotation2d trajGoalHeading = pathGoalState.pose.getRotation();
            Robot.dashboard.trajField2d.setRobotPose(trajGoalX, trajGoalY, trajGoalHeading);

            correctionInXV = followPathXController.calculate(Robot.drivebase.yagslDrive.getPose().getX(),
                    trajGoalX);
            correctionInYV = followPathYController.calculate(Robot.drivebase.yagslDrive.getPose().getY(),
                    trajGoalY);

            double xV = pathGoalState.fieldSpeeds.vxMetersPerSecond + correctionInXV;
            double yV = pathGoalState.fieldSpeeds.vyMetersPerSecond + correctionInYV;

            Translation2d correctedPathFollowingTranslation = new Translation2d(xV, yV);

            driveFacingHeading(correctedPathFollowingTranslation, trajGoalHeading, true);

            return false;

        }

    }

    public double getRotationToHub() {

        Rotation2d angleToHub;

        if (Robot.onRed) {
            // red hub location
            angleToHub = Settings.FieldInfo.hubRedLocation.getRotation()
                    .minus(yagslDrive.getPose().getRotation());
        } else {
            // blue hub location
            angleToHub = Settings.FieldInfo.hubBlueLocation.getRotation()
                    .minus(yagslDrive.getPose().getRotation());
        }
        dvr = rotationPidController.calculate(yagslDrive.getPose().getRotation().minus(angleToHub).getDegrees(), 0);

        return dvr;
    }

}
