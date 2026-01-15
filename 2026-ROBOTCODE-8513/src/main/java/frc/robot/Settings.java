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
        public static int forwardBackwardsAxis = 2;
        public static int leftRightAxis = 1;
        public static int rotAxis = 5;

        public static int rightJoystickX = 5;
        public static int rightJoystickY = 6;

        public static Joystick rJoystickController;

        public static boolean headingJoystickControls = true;


    }
 
}
