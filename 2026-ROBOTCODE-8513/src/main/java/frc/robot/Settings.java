package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class Settings {
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltWelded);

    public static class DrivebaseSettings {

        public static final double maxVelocityMPS = 4.2;
        public static boolean getPIDValuesFromDashboard = false;

        public static class RotationPIDConstants {
            public static final double kP = 0.1;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
        }
        public static class FaceGoalPIDConstants {
            public static final double kP = 0.175;
            public static final double kI = 0.0125;
            public static final double kD = 0.0;
        }
    }

    public static class PhysicalRobotValues {
        public static double robotHeight = 30;
        public static double bumpMaxVelocity = 0.5;
    }

    public static class TeleopSettings {
        public static int driverJoystickPort = 1;
        public static int copilotJoystick1Port = 2;
        public static int copilotJoystick2Port = 3;
        public static int manualJoystickPort = 4;
        public static int tuningJoystickPort = 5;
        public static double joystickDeadband = 0.02;
        public static double snakeModeJoystickDeadband = 0.35;
        public static double specialRotationJoystickDeadband = 0.90;
        public static double drivingWhileShootingSpeed = 0.3;

        // joystick axis
        public static int forwardBackwardsAxis = 1;
        public static int leftRightAxis = 0;
        public static int rotAxis = 4;
        public static int RforwardBackwardsAxis = 4;
        public static int RleftRightAxis = 5;
        public static int rightJoystickX = 5;
        public static int rightJoystickY = 6;

        public static boolean headingJoystickControls = true;
        public static double snakeModeVelocityFactor = 0.4;

        public static class ButtonIDs {            
            // driver controller
            public static int deployIntake = 6;
            public static int faceGoal = 7;
            public static int stowIntake = 5;
            public static int straightenBump = 8;
            public static int snakeMode = 1;
            public static int resetHeading = 10;
            public static int faceRight = 2; 
            public static int faceLeft = 3; 

            // copilot controller 1
            public static int increaseShotDistance = 4;
            public static int decreaseShotDistance = 6;
            public static int moveScorePoseRight = 3;
            public static int moveScorePoseLeft = 5;
            public static int copilotStowIntake = 10;
            public static int copilotDeployIntake = 8;
            public static int copilotStopIntakeWheels = 9;
            public static int jiggleIntake = 7;
            public static int forceShoot = 2; // force shoot
            public static int forceDontShoot = 1; // force not shoot?
            public static int intakeBackward = 11;
            public static int reverseEverything = 12;
            

            // copilot controller 2
            public static int redDepotTrenchButton = 9;
            public static int blueDepotTrenchButton = 10;
            public static int redOutpostTrenchButton = 7;
            public static int blueOutpostTrenchButton = 11;
            public static int nuetralZoneButton1 = 8;
            public static int nuetralZoneButton2 = 12;


            // manual controller
            public static int intakeToggle = 4;
            public static int kickerToggle = 1;
            public static int shooterToggle = 3;
            public static int resetIntake = 8;
            public static int gradualShooterSpinUp = 0;
            public static int heightenIntake = 5;
            public static int lowerIntake = 6;

            //joystick 5 tuning joystick
            public static int shooterManualIncreaseVelocity = 2; // CHANGE THESE WHEN TUNING
            public static int shooterManualDecreaseVelocity =1;
            public static int incHoodPos = 3;
            public static int decHoodPos = 4;


        }

    }

    public class IntakeSettings {
        public static double stowPosition = 0.58;
        public static double deployPosition = 0;
        public static double intakeFudgeFactor = 0.02;
        public static double shootingPosition = 0.35;
        public static double maxIntakePositionToNotHitHood = 0.36;
        public static Constraints deployConstraints = new Constraints(1.75, 3.0);
        public static Constraints shootingConstraints = new Constraints(1.75, 3.0);
        public static double spinBackwardsThreshold = 0.05;
    }

    public class ShooterSettings {
        public static double angleFudgeDelta = 0.1; 
        public static double shotDistanceFudgeDelta = 0.1; 
        public static double maxShooterVelocity = 47;
        public static double manualVelocityTuningFactor = 0.5;
        public static double manualHoodPosTuningfactor = 0.01;
        public static double lowestHoodPosition = 0.24;
        public static double lowestPositionIntakeCanComeBack = 0.89;
        public static double highestHoodPosition = 0.93;



    }

    public class VisionSettings {
        public static double maxATDistDisabeled = 5;
        public static boolean useVision = true;
    }

    public class FieldInfo {

        // hub locations
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top

        public static final Pose2d hubRedLocation = new Pose2d(
                11.919, 4.029, new Rotation2d(180));
        public static final Pose2d hubBlueLocation = new Pose2d(
                4.621, 4.029, new Rotation2d(0));

        public static final Translation3d blueHubCenterPointTrans3d = new Translation3d(
                aprilTagFieldLayout.getTagPose(26).get().getX() + width / 2.0,
                aprilTagFieldLayout.getFieldWidth() / 2.0,
                height);
        public static final Translation3d redHubCenterPointTrans3d = new Translation3d(
                aprilTagFieldLayout.getTagPose(4).get().getX() + width / 2.0,
                aprilTagFieldLayout.getFieldWidth() / 2.0,
                height);

        public static final Pose2d blueHubCenterPoint = new Pose2d(
                blueHubCenterPointTrans3d.getX(),
                blueHubCenterPointTrans3d.getY(),
                new Rotation2d(0));
        public static final Pose2d redHubCenterPoint = new Pose2d(
                redHubCenterPointTrans3d.getX(),
                redHubCenterPointTrans3d.getY(),
                new Rotation2d(0));

        public static double hubHeight = 10;

        // shuttling positions
        public class ShuttlingPositions {
            public static Pose2d redDepotTrench = new Pose2d(12.935, 1.6, new Rotation2d());
            public static Pose2d blueDepotTrench = new Pose2d(1.514, 6.4, new Rotation2d());
            public static Pose2d redOutpostTrench = new Pose2d(12.935, 6.4, new Rotation2d());
            public static Pose2d blueOutpostTrench = new Pose2d(1.514, 1.6, new Rotation2d());
            public static Pose2d neutralZone1 = new Pose2d(8.283, 1.635, new Rotation2d());
            public static Pose2d neutralZone2 = new Pose2d(8.283, 6.668, new Rotation2d());

        }

    }

    public class AutoSettings {
        public class Thresholds {
            public static double drivebaseShootRotationTHold = 2.75;
            public static double drivebaseLockPoseWhenShootingThold = 1.5;
            public static double drivebaseShuttleRotationTHold = 10;
            public static double shootHoodPositionTHold = 0.05;
            public static double shuttleHoodPositionTHold = 0.1;
            public static double shooterVelocityTHold = 3;
            public static double shooterShuttleVelocityTHold = 6;
            public static double autoDetectedBumpPitchTHold = 6;
            public static double autoDetectedBumpPitchCount = 6;
            public static double detectedFlatTHold = 5;
        }
        public static double autoIntakeBringInDelay = 3.25;
        public static double autoIValue = 0.0275;
    }

    public static void resetTalon(TalonFX m_talonFX) {
        var stickyFaults = m_talonFX.getStickyFault_OverSupplyV().getValue();
        if (stickyFaults) {

            m_talonFX.clearStickyFaults();

            applyConfigs(m_talonFX);
        }
    }

    private static void applyConfigs(TalonFX m_talonFX) {
        var configs = new CurrentLimitsConfigs();
        configs.StatorCurrentLimitEnable = true;
        configs.StatorCurrentLimit = 120;
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = 90;

        m_talonFX.getConfigurator().apply(configs);
    }
}
