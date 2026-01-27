package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;
import frc.robot.Logic.Enums.ShooterStates;

public class Shooter {

    public static SparkMax shooterMotorLeft = new SparkMax(11, MotorType.kBrushless);
    public static SparkFlex shooterMotorRight = new SparkFlex(12, MotorType.kBrushless);

    public PIDController shooterMotorController = new PIDController(0.0005, 0.000001, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public Shooter() {


    }

    public void setMotorPower() {
        if (Robot.teleop.driverXboxController.getRawButton(5)) {
            shooterMotorLeft.set(updateMotorPower());
            shooterMotorRight.set(updateMotorPower());

        } else {
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }
    }

    public double updateMotorPower() {
        double currentVelocity = shooterMotorLeft.getEncoder().getVelocity();
        double targetVelocity = 2900;
        double outputPower = shooterMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity/6140;
    }
}
