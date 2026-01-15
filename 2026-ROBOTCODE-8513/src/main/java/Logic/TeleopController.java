public class TeleopController {
    
    public Joystick driverXboxController = new Joystick(Settings.TeleopSettings.driverJoystickPort);

    Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

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

        double xV = xInput * 
.swerveDrive.getMaximumChassisVelocity();
        double yV = yInput * drivebase.swerveDrive.getMaximumChassisVelocity();
        double rV = rInput * drivebase.swerveDrive.getMaximumChassisAngularVelocity();

        double jX = driverXboxController.getRawAxis(Settings.TeleopSettings.rightJoystickX);
        double jY = driverXboxController.getRawAxis(Settings.TeleopSettings.rightJoystickY);

        if (Settings.TeleopSettings.headingJoystickControls) {
            if (Math.sqrt(jX * jX + jY * jY) > 0.5) {
                goalHeading = new Rotation2d(jX, -jY);
                goalHeading = goalHeading.minus(Rotation2d.fromDegrees(90));
            }
            rV = Settings.TeleopSettings.rJoystickController.calculate(
                    drivebase.swerveDrive.getPose().getRotation().minus(goalHeading).getDegrees(), 0);
        }
    }
}
