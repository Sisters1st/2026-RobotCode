package frc.robot.Logic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.IntakeStates;

public class TeleopController {

    public Joystick driverXboxController = new Joystick(Settings.TeleopSettings.driverJoystickPort);
    public PIDController rJoystickController = new PIDController(0.1, 0, 0);

    public Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public void initTele() {
        Robot.shooter.initShooter();

    }

    public void driveTele() {

        double xSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.forwardBackwardsAxis); // forward back
        if (xSpeedJoystick < Settings.TeleopSettings.joystickDeadband && xSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.leftRightAxis); // left right
        if (ySpeedJoystick < Settings.TeleopSettings.joystickDeadband && ySpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            ySpeedJoystick = 0;

        }
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);

        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.rotAxis); // left right 2 at home, 4 on xbox
        if (rSpeedJoystick < Settings.TeleopSettings.joystickDeadband && rSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            rSpeedJoystick = 0;

        }
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        double xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.6;
        double yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.6;
        double rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();


        Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), rV, true, false);

        
        // intake controls
        if (Robot.intake.intakeState == IntakeStates.stationary && Robot.teleop.driverXboxController.getRawButton(Settings.TeleopSettings.intake_buttonID)) {
            Robot.intake.intakeState = IntakeStates.intaking;
        } else if (Robot.intake.intakeState == IntakeStates.intaking && Robot.teleop.driverXboxController.getRawButton(Settings.TeleopSettings.intake_buttonID)) {
            Robot.intake.intakeState = IntakeStates.stationary;
        }


        // Subsystem set motor power
        Robot.shooter.setMotorPower();
        Robot.intake.setMotorPower();  

    }
}
