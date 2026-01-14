package frc.robot.Logic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;

public class TeleopController {

    Joystick driverJoystick = new Joystick(Settings.TeleopSettings.driverJoystick.port);
    public boolean fieldRelative = true;

    public Rotation2d goalHeading = new Rotation2d();

    public TeleopController(){

    }

    public void teleopLoop(){
        double xJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.driverJoystick.xAxis);
        double yJoystickValule = driverJoystick.getRawAxis(Settings.TeleopSettings.driverJoystick.yAxis);
        double rJoystickValule = -driverJoystick.getRawAxis(Settings.TeleopSettings.driverJoystick.rAxis);

        if(Robot.onRed == false){
            xJoystickValule *= -1;
            yJoystickValule *= -1;
        }

        Translation2d robotVelocity = new Translation2d(xJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS, 
                                                        yJoystickValule * Settings.DrivebaseSettings.maxVelocityMPS);

        Robot.drivebase.yagslDrive.drive(robotVelocity,
                                            rJoystickValule * Settings.DrivebaseSettings.maxRotationalVelocityRPS, 
                                            fieldRelative,
                                            false);
    }
    
}
