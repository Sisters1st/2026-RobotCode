package frc.robot.Logic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Dashboard {

    public static void updateDashboard() {
        SmartDashboard.putNumber("Shooter Velocity", Robot.shooter.shooterMotorLeft.getEncoder().getVelocity());
    }
}
