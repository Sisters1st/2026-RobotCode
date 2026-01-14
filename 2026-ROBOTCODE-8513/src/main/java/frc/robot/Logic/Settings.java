package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Settings {

    public static class DrivebaseSettings{
        public static final double maxVelocityMPS = 3.0;
        public static final double maxRotationalVelocityRPS = 6.28;

        public static class RotationPIDConstants{
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

        }
    }

    public static class TeleopSettings{

        public static class DriverJoystick{
            public static final int port = 0;
            public static final int xAxis = 1;
            public static final int yAxis = 0;
            public static final int rAxis = 4;
            
            public static final double deadband = 0.1;

            public static final double rotationalJoystickSensitivity = 1;

            public static final int faceHubButton = 1;

        }
    }

    public static class Field{
        public static class Dimmentions{
            public static final double width = 16.54;
            public static final double height = 8;
        }  

        public static class Poses{ 
            public static final Pose2d blueHub = new Pose2d(4.631, 4, new Rotation2d());
            public static final Pose2d redHub = flipToRed(blueHub);
        }
    }


    public static Pose2d flipToRed(Pose2d bluePose){
        //blue pose was already a red pose
        if(bluePose.getX() > Settings.Field.Dimmentions.width){
            return bluePose;
        }
        return new Pose2d(Settings.Field.Dimmentions.width - bluePose.getX(),
                            Settings.Field.Dimmentions.height - bluePose.getY(),
                            bluePose.getRotation().plus(new Rotation2d(Math.PI)));
    }
    
}
