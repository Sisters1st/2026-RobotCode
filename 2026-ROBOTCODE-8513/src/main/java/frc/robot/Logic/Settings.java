package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Settings {

    public static class DrivebaseSettings{
        public static final double maxVelocityMPS = 3.8;
        public static final double maxRotationalVelocityRPS = 6.28;

        public static class RotationPIDConstants{
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;

        }

        public static class PathFollowingPIDConstatnts{
            public static final double kP = 10;
            public static final double kI = 0;
            public static final double kD = 0;

        }
    }

    public static class ShooterSettings{
        public static final double targetVelocity = 2000;
        public static final double readyToShootThold = 50;

        public static int shooterMotor1CANID = 1;
        public static int shooterMotor2CANID = 2;

        public static class SpinUpShooterPIDFConstants{
            public static final double kS = 0.1;
            public static final double kV = 0.12;
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kF = 0;
        }

        public static class MaintainShooterPIDFConstants{
            public static final double kS = 0.1;
            public static final double kV = 0.12;
            public static final double kP = 0.1;
            public static final double kI = 0;
            public static final double kD = 0;
            public static final double kF = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kF;
        }
    }

    public static class TeleopSettings{

        public static class DriverJoystick{
            public static final int port = 0;
            public static final int xAxis = 1;
            public static final int yAxis = 0;
            public static final int rAxis = 4;

            public static final boolean usePointToFaceControl = false;
            public static final int rxAxisPTF = 4;
            public static final int ryAxisPTF = 5;
            public static final double pointToFaceRotationCutoff = 0.5;

            
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
