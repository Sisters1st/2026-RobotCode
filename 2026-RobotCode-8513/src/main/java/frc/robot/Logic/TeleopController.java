package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Joystick driverJoystick = new Joystick(Settings.TeleopSettings.DriverJoystick.port);
    public boolean fieldRelative = true;
    public Rotation2d lastPTFHeading = new Rotation2d();

    public TeleopController() {

    }

    public void teleopInit(){
        Robot.updateAlliance();
        Robot.drivebase.goalHeading = Robot.drivebase.yagslDrive.getOdometryHeading();
    }

    public void teleopLoop() {
        driveWithJoystick();
    }

    public void driveWithJoystick(){
        //read all joysticks and deadband them
        double xJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.xAxis);
        double yJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.yAxis);
        double rJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.rAxis);

        double rxJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.rxAxisPTF);
        double ryJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.ryAxisPTF);

        if (Math.abs(xJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            xJoystickValule = 0;
        }
        if (Math.abs(yJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            yJoystickValule = 0;
        }
        if (Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            rJoystickValule = 0;
        }
        if (Math.abs(rxJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            rxJoystickValule = 0;
        }
        if (Math.abs(ryJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            ryJoystickValule = 0;
        }

        //invert if on red
        if (Robot.onRed == false) {
            xJoystickValule *= -1;
            yJoystickValule *= -1;
        }

        //calcualte robot velocity
        Translation2d robotVelocity = new Translation2d(xJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS,
                yJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS);
        
        //press A to face hub
        if(driverJoystick.getRawButton(Settings.TeleopSettings.DriverJoystick.faceHubButton)){
            Pose2d hubPose = Settings.Field.Poses.blueHub;
            if(Robot.onRed){
                hubPose = Settings.Field.Poses.redHub;
            }
            Robot.drivebase.driveFacingPose(robotVelocity, hubPose, fieldRelative);
            lastPTFHeading = Robot.drivebase.yagslDrive.getOdometryHeading();

        } else if(Settings.TeleopSettings.DriverJoystick.usePointToFaceControl){
            //use point to face control on the rotation joystick
            Rotation2d pointToFaceGoalHeading;
            if(Math.abs(ryJoystickValule) < Settings.TeleopSettings.DriverJoystick.pointToFaceRotationCutoff 
                && Math.abs(rxJoystickValule) < Settings.TeleopSettings.DriverJoystick.pointToFaceRotationCutoff){
                pointToFaceGoalHeading = lastPTFHeading;
            } else {
                pointToFaceGoalHeading = new Rotation2d(ryJoystickValule, rxJoystickValule);
                lastPTFHeading = pointToFaceGoalHeading;
            }
            if(Robot.onRed){
                pointToFaceGoalHeading = pointToFaceGoalHeading.plus(new Rotation2d(Math.PI));
            }
            Robot.drivebase.drive(robotVelocity, pointToFaceGoalHeading, fieldRelative);

        } else {
            //standard swerve rotation control
            Rotation2d goalHeading = Robot.drivebase.goalHeading;
            if (!(Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband)) {
                Rotation2d currentHeading = Robot.drivebase.yagslDrive.getOdometryHeading();
                goalHeading = currentHeading.plus(new Rotation2d(
                                rJoystickValule * Settings.TeleopSettings.DriverJoystick.rotationalJoystickSensitivity));
                Robot.drivebase.goalHeading = goalHeading;
            }
            Robot.drivebase.drive(robotVelocity, goalHeading, fieldRelative);
        }

    }

}
