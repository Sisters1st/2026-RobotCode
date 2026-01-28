package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Settings {
   public static class DrivebaseSettings{
        
        public static final double maxVelocityMPS = 5;

        public static class RotationPIDConstants{
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }
    public static class TeleopSettings{
        public static int driverJoystickPort = 1;
        public static double joystickDeadband = 0.1;

        // joystick axis
        public static int forwardBackwardsAxis = 1;
        public static int leftRightAxis = 0;
        public static int rotAxis = 4;

        public static int rightJoystickX = 5;
        public static int rightJoystickY = 6;

        public static boolean headingJoystickControls = true;

        // driver controller button ID
        public static class ButtonIDs{
            public static int intake = 2;
            public static int faceHub = 1;
        }

    }

    public class VisionSettings{
        public static double maxATDistDisabeled = 10;
    }

    public class FieldInfo{
        // hub locations
        public static final Pose2d hubRedLocation = new Pose2d(
                11.919, 4.029, new Rotation2d(180));
        public static final Pose2d hubBlueLocation = new Pose2d(
                4.621, 4.029, new Rotation2d(0));
    }
 
}
