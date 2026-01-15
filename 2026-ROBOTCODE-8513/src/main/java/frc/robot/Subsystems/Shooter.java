package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.Logic.Settings;

public class Shooter {

    public boolean isShooting = false;
    public boolean readyToShoot = false;

    public TalonFX shooterMotor1 = new TalonFX(Settings.ShooterSettings.shooterMotor1CANID);
    public TalonFX shooterMotor2 = new TalonFX(Settings.ShooterSettings.shooterMotor2CANID);

    public Shooter() {
        // redo this with magic motion

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kS;
        slot0Configs.kV = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kV;
        slot0Configs.kP = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kP;
        slot0Configs.kI = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kI; // no output for integrated error
        slot0Configs.kD = Settings.ShooterSettings.SpinUpShooterPIDFConstants.kD; // no output for error derivative

        shooterMotor1.getConfigurator().apply(slot0Configs);
        shooterMotor2.getConfigurator().apply(slot0Configs);

        Slot1Configs slot1Configs = new Slot1Configs();
        slot1Configs.kS = Settings.ShooterSettings.MaintainShooterPIDFConstants.kS;
        slot1Configs.kV = Settings.ShooterSettings.MaintainShooterPIDFConstants.kV;
        slot1Configs.kP = Settings.ShooterSettings.MaintainShooterPIDFConstants.kP;
        slot1Configs.kI = Settings.ShooterSettings.MaintainShooterPIDFConstants.kI; // no output for integrated error
        slot1Configs.kD = Settings.ShooterSettings.MaintainShooterPIDFConstants.kD; // no output for error derivative

        shooterMotor1.getConfigurator().apply(slot1Configs);
        shooterMotor2.getConfigurator().apply(slot1Configs);
    }

    public void runShooter(double targetRPS) {
        VelocityVoltage targetRequest;
        if (!readyToShoot) {
            targetRequest = new VelocityVoltage(targetRPS).withSlot(0);
        } else {
            targetRequest = new VelocityVoltage(targetRPS).withSlot(1);
        }

        double ffPower = targetRPS * Settings.ShooterSettings.SpinUpShooterPIDFConstants.kF;

        shooterMotor1.setControl(targetRequest.withVelocity(targetRPS).withFeedForward(ffPower));
        shooterMotor2.setControl(targetRequest.withVelocity(targetRPS).withFeedForward(ffPower));

        if (shooterMotor1.getClosedLoopError().getValue() < Settings.ShooterSettings.readyToShootThold) {
            readyToShoot = true;
        }

    }

    public void coastShooter() {
        readyToShoot = false;
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    public Pose2d calculateGoalPoseOnTheFly() {
        Pose2d goalPose;

        if (Robot.onRed) {
            goalPose = Settings.Field.Poses.redHub;
        } else {
            goalPose = Settings.Field.Poses.blueHub;
        }

        double newX = goalPose.getX()
                - Robot.drivebase.yagslDrive.getFieldVelocity().vxMetersPerSecond * calculateTimeOfFuelInAir();
        double newY = goalPose.getY()
                - Robot.drivebase.yagslDrive.getFieldVelocity().vyMetersPerSecond * calculateTimeOfFuelInAir();
        if (Robot.onRed) {
            return new Pose2d(newX, newY, new Rotation2d(Math.PI));

        } else {
            return new Pose2d(newX, newY, new Rotation2d());

        }
    }

    public double calculateTimeOfFuelInAir() {
        // need to test imperically
        return 1;
    }

}
