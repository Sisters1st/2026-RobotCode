package frc.robot;

public class Settings {
   public static class DrivebaseSettings{
        
        public static final double maxVelocityMPS = 5;

        public static class RotationPIDConstants{
            public static final double kP = 0.1;
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
            public static int intake = 1;
        }

    }

    public class VisionSettings{
        public static double maxATDistDisabeled = 10;
    }
 
}
