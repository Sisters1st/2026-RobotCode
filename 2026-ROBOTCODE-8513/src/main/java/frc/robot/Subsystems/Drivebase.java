package frc.robot.Subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
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
    public ProfiledPIDController rotationPidController = new ProfiledPIDController(
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kP,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kI,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kD, new Constraints(90, 500));
    public ProfiledPIDController faceGoalPidController = new ProfiledPIDController(
            frc.robot.Settings.DrivebaseSettings.FaceGoalPIDConstants.kP,
            frc.robot.Settings.DrivebaseSettings.FaceGoalPIDConstants.kI,
            frc.robot.Settings.DrivebaseSettings.FaceGoalPIDConstants.kD, new Constraints(90, 500));


    // path following variables
    public PathPlannerTrajectory traj;
    public boolean loadedPathHasStarted = false;
    PathPlannerPath path;
    public String pathName = "";
    double elapsedTime;
    double timePathStarted;
    public PathPlannerTrajectoryState trajGoalState = new PathPlannerTrajectoryState();
    Field2d trajGoalPosition = new Field2d();
    double otfEndVelocity = 0;
    public boolean forcePathHeading = false;
    double dvr;
    public Pose2d goalAimPose = new Pose2d();

    // edit these values and put in settings later
    public PIDController followPathXController = new PIDController(10, 0, 0);

    public PIDController followPathYController = new PIDController(10, 0, 0);

    public double timeOfFlight;

    public double aimFudgeFactor = 0;

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
        goalHeading = heading;

        Robot.drivebase.yagslDrive.drive(translation2d,
                rotationCorrection,
                fR,
                false);

    }

    public void driveFacingPose(Translation2d translation2d, Pose2d pose, boolean fR) {
        double angleError = Robot.drivebase.yagslDrive.getOdometryHeading().minus(pose.getTranslation().getAngle()).getDegrees();
        double rotationCorrection = rotationPidController.calculate(angleError, 0);

        Robot.drivebase.yagslDrive.drive(translation2d,
                rotationCorrection,
                fR,
                false);
    }

    public void initPath(String pathNameIn, boolean mirrorPath) {
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

        if (mirrorPath) {
            path = path.mirrorPath();
        }

        try {
            traj = path.getIdealTrajectory(RobotConfig.fromGUISettings()).get();
            if (Robot.isSimulation() || true) {
                Robot.dashboard.trajField2d.getObject("traj").setTrajectory(ppTrajToWPITraj(traj));
            }
        } catch (Exception e) {
            System.out.println("Error in trajectory generation");
            e.printStackTrace();
        }

        if (Robot.isSimulation()) {
            yagslDrive.resetOdometry(path.getStartingHolonomicPose().get());
        }

        loadedPathHasStarted = false;

    }

    //PP trajectories cant be put on dashboard so they need to get point by point converted to WPI trajectories
    //so we can visualize them in AdvantageScope
    public Trajectory ppTrajToWPITraj(PathPlannerTrajectory traj) {
        List<PathPlannerTrajectoryState> stateList = traj.getStates();
        List<Trajectory.State> wpiStateLists = new ArrayList<Trajectory.State>();
        for (PathPlannerTrajectoryState state : stateList) {
            Trajectory.State thisWPIState = new Trajectory.State(state.timeSeconds,
                    state.linearVelocity,
                    0,
                    state.pose,
                    0);
            wpiStateLists.add(thisWPIState);
        }
        return new Trajectory(wpiStateLists);
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
            goalHeading = trajGoalHeading;

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

    public double getPowerToFaceHub() {
        Rotation2d angleToHub;

        if (Robot.onRed) {
            // red hub location
            goalAimPose = offsetPose2dByVelocity(Settings.FieldInfo.redHubCenterPoint);
            Pose2d currPose2d = yagslDrive.getPose();
            Transform2d robotToShooterTransform2d = new Transform2d(0, Units.inchesToMeters(-2.25), new Rotation2d(0));
            Pose2d shooterPose = currPose2d.plus(robotToShooterTransform2d);
            angleToHub = shooterPose.minus(goalAimPose)
                    .getTranslation().rotateBy(new Rotation2d()).getAngle();
        } else {
            // blue hub location
            goalAimPose = offsetPose2dByVelocity(Settings.FieldInfo.blueHubCenterPoint);
            Pose2d currPose2d = yagslDrive.getPose();
            Transform2d robotToShooterTransform2d = new Transform2d(0, Units.inchesToMeters(-2.25), new Rotation2d(0));
            Pose2d shooterPose = currPose2d.plus(robotToShooterTransform2d);
            angleToHub = shooterPose.minus(goalAimPose)
                    .getTranslation().rotateBy(new Rotation2d()).getAngle();
        }
        goalHeading = angleToHub.plus(new Rotation2d(aimFudgeFactor));
        Robot.dashboard.scoreHubField2d.setRobotPose(goalAimPose.getX(), goalAimPose.getY(),
                goalAimPose.getRotation());

        dvr = faceGoalPidController.calculate(yagslDrive.getOdometryHeading().minus(goalHeading).getDegrees(), 0);

        return dvr;
    }

    public double getPowerToFacePose(Pose2d goalPose) {
        //we use this to shuttle, we dont care (we think) about the offset when shuttleing
        Rotation2d angleToHub;

        if (Robot.onRed) {
            // red hub location
            angleToHub = yagslDrive.getPose().minus(goalPose)
                    .getTranslation().rotateBy(new Rotation2d()).getAngle();
        } else {
            // blue hub location
            angleToHub = yagslDrive.getPose().minus(goalPose)
                    .getTranslation().getAngle()
                    .plus(new Rotation2d());
        }
        goalHeading = angleToHub.plus(new Rotation2d(aimFudgeFactor));
        dvr = rotationPidController.calculate(yagslDrive.getOdometryHeading().minus(angleToHub).getDegrees(), 0);

        return dvr;
    }

    // calculates the tof
    public double timeOfFlight() {
        Robot.shooter.distanceBetweenCurrentAndGoalInMeters = Robot.drivebase
                .getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(), Robot.drivebase.goalAimPose);
        timeOfFlight = Robot.shooter.getInterpolatedTimeOfFlightFromDistance(Robot.shooter.distanceBetweenCurrentAndGoalInMeters);
        return timeOfFlight;
    }

    // returns the new Pose2d score location
    double shotDisplacement;

    public Pose2d offsetPose2dByVelocity(Pose2d originalPose2d) {
        timeOfFlight();
        double rVX = yagslDrive.getFieldVelocity().vxMetersPerSecond;
        double rVY = yagslDrive.getFieldVelocity().vyMetersPerSecond;
        double x = rVX * timeOfFlight;
        double y = rVY * timeOfFlight;
        Transform2d transform = new Transform2d(-x, -y, new Rotation2d());
        return originalPose2d.transformBy(transform);
    }

    public double getDistanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
        double x = pose2.getX() - pose1.getX();
        double y = pose2.getY() - pose1.getY();
        double distance = Math.sqrt(x * x + y * y);
        return distance;
    }

    public void faceHub() {
        double powerToAngleToHub = getPowerToFaceHub();
        yagslDrive.drive(new Translation2d(), powerToAngleToHub, true, false);
    }

    public boolean drivingOverBump() {
        if ((Robot.drivebase.yagslDrive.getPose().getX() > 11
                && Robot.drivebase.yagslDrive.getPose().getX() < 12.7)
                || (Robot.drivebase.yagslDrive.getPose().getX() > 3.85
                        && Robot.drivebase.yagslDrive.getPose().getX() < 5.4)) {
            return true;
        } else {
            return false;
        }
    }

    public double getPowerToReachRotation(Rotation2d goalRotation) {
        dvr = rotationPidController.calculate(yagslDrive.getOdometryHeading().minus(goalRotation).getDegrees(), 0);
        return dvr;
    }

    public double getRobotVelocityHypotenuse() {
        double rXV = yagslDrive.getFieldVelocity().vxMetersPerSecond;
        double rYV = yagslDrive.getFieldVelocity().vyMetersPerSecond;
        double velocity = Math.sqrt(Math.pow(rXV, 2) + Math.pow(rYV, 2));
        return velocity;
    }

}
