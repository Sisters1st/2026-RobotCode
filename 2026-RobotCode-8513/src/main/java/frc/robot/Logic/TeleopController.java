package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Joystick driverJoystick = new Joystick(Settings.TeleopSettings.DriverJoystick.port);
    public boolean fieldRelative = true;

    public TeleopController() {

    }

    public void teleopLoop() {
        driveWithJoystick();
    }

    public void driveWithJoystick(){

        double xJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.xAxis);
        double yJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.yAxis);
        double rJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.DriverJoystick.rAxis);

        if (Math.abs(xJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            xJoystickValule = 0;
        }
        if (Math.abs(yJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            yJoystickValule = 0;
        }
        if (Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband) {
            rJoystickValule = 0;
        }

        if (Robot.onRed == false) {
            xJoystickValule *= -1;
            yJoystickValule *= -1;
        }

        Translation2d robotVelocity = new Translation2d(xJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS,
                yJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS);
        Rotation2d goalHeading = Robot.drivebase.yagslDrive.getOdometryHeading();

        if (!(Math.abs(rJoystickValule) < Settings.TeleopSettings.DriverJoystick.deadband)) {
            goalHeading = goalHeading.plus(new Rotation2d(
                            rJoystickValule * Settings.TeleopSettings.DriverJoystick.rotationalJoystickSensitivity));
        }

        if(driverJoystick.getRawButton(Settings.TeleopSettings.DriverJoystick.faceHubButton)){
            Pose2d hubPose = Settings.Field.Poses.blueHub;
            if(Robot.onRed){
                hubPose = Settings.Field.Poses.redHub;
            }
            Robot.drivebase.driveFacingPose(robotVelocity, hubPose, fieldRelative);
        } else {
            Robot.drivebase.drive(robotVelocity, goalHeading, fieldRelative);
        }

    }

}
