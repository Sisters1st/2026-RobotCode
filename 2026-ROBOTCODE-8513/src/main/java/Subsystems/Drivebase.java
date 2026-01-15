public class Drivebase {
    public SwerveDrive yagslDrive;
    public Rotation2d goalHeading = new Rotation2d();
    public PIDController rotationPidController = new PIDController(frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kP,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kI,
            frc.robot.Settings.DrivebaseSettings.RotationPIDConstants.kD);

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
}
