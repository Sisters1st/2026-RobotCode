package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Logic.Settings;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drivebase {

    //yagsl object
    public SwerveDrive yagslDrive;

    //driving vars
    public Rotation2d goalHeading = new Rotation2d();

    //path vars
    public PathPlannerPath path;
    public PathPlannerTrajectory traj;
    public boolean initializedPathHasStarted = false;
    public double pathStartTime = 0;
    public boolean pathIsDone = false;

    //controllers
    public PIDController rotationPidController = new PIDController(Settings.DrivebaseSettings.RotationPIDConstants.kP,
            Settings.DrivebaseSettings.RotationPIDConstants.kI,
            Settings.DrivebaseSettings.RotationPIDConstants.kD);

    public PIDController pathFollowingXPidController = new PIDController(Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kP,
            Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kI,
            Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kD);
    
    public PIDController pathFollowingYPidController = new PIDController(Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kP,
            Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kI,
            Settings.DrivebaseSettings.PathFollowingPIDConstatnts.kD);

    //dashboard publishers
    StructPublisher<Pose2d> trajGoalPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("TrajGoalpose", Pose2d.struct).publish();


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

    public void drive(Translation2d translation2d, Rotation2d heading, boolean fR) {
        double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(heading).getDegrees();
        double rotationCorrection = rotationPidController.calculate(angleError, 0);

        Robot.drivebase.yagslDrive.drive(translation2d,
                rotationCorrection,
                fR,
                false);
        double vX = yagslDrive.getFieldVelocity().vxMetersPerSecond;
        double vY = yagslDrive.getFieldVelocity().vyMetersPerSecond;
        SmartDashboard.putNumber("DrivebaseVelocity", Math.sqrt(vX * vX + vY * vY) );
    }

    public void driveFacingPose(Translation2d translation2d, Pose2d facePoint, boolean fR){
        
        Pose2d currentPose = Robot.drivebase.yagslDrive.getPose();
        Rotation2d angleToFace;
        if(Robot.onRed){
            angleToFace = currentPose.minus(facePoint).getTranslation().getAngle();
        } else {
            angleToFace = currentPose.minus(facePoint).getTranslation().getAngle().plus(new Rotation2d(Math.PI));
        }
        goalHeading = angleToFace;
        drive(translation2d, angleToFace, fR);

    }

 public void initPathFromFile(String pathNameIn) {
        try {
            path = PathPlannerPath.fromPathFile(pathNameIn);
        } catch (Exception e) {
            System.out.println("Error in loading path");
            e.printStackTrace();
        }

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

        initializedPathHasStarted = false;
    }

    public void followPath(){
        if(traj != null){
            if(initializedPathHasStarted == false){
                pathStartTime = Timer.getFPGATimestamp();
                initializedPathHasStarted = true;
                pathIsDone = false;
            }

            double deltaT = Timer.getFPGATimestamp() - pathStartTime;

            if(deltaT > traj.getTotalTimeSeconds()){
                pathIsDone = true;
            } else {           
                PathPlannerTrajectoryState pathGoalState = traj.sample(deltaT);

                double goalX = pathGoalState.pose.getX();
                double goalY = pathGoalState.pose.getY();

                double correctionInXV = pathFollowingXPidController.calculate(Robot.drivebase.yagslDrive.getPose().getX(), goalX);
                double correctionInYV = pathFollowingYPidController.calculate(Robot.drivebase.yagslDrive.getPose().getY(), goalY);

                double xV = pathGoalState.fieldSpeeds.vxMetersPerSecond + correctionInXV;
                double yV = pathGoalState.fieldSpeeds.vyMetersPerSecond + correctionInYV;

                Translation2d correctedPathFollowingTranslation = new Translation2d(xV, yV);

                drive(correctedPathFollowingTranslation, pathGoalState.pose.getRotation(), true);

                trajGoalPosePublisher.set(pathGoalState.pose);
            }
        }
    }

}
