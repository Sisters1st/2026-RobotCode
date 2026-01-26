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
    public static SparkFlex shooterMotorKracken = new SparkFlex(13, MotorType.kBrushless);

    public PIDController shooterMotorController = new PIDController(0.0001, 0.000001, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public boolean useInternalController = true;

    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    // create a velocity closed-loop request, voltage output, slot 0 configs
    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public Shooter() {

    }

    public void initShooter() {
        shooterMotorKracken.getConfigurator().apply(slot0Configs);
    }

    public void setMotorPower() {
        // shooter controller if not using interal
        if (shooterState == ShooterStates.shooting && useInternalController == false) {
            shooterMotorLeft.set(updateMotorPower());
            shooterMotorRight.set(updateMotorPower());

        } else if (shooterState == ShooterStates.stationary && useInternalController == false){
            shooterMotorLeft.set(0);
            shooterMotorRight.set(0);
        }

        // shooter controller if using interal
        if (shooterState == ShooterStates.shooting && useInternalController == true) {
            shooterMotorKracken.setControl(m_request.withVelocity(8).withFeedForward(0.5));

        } else if (shooterState == ShooterStates.stationary && useInternalController == true){
            shooterMotorKracken.setControl(m_request.withVelocity(0).withFeedForward(0));

        }
    }

    public double updateMotorPower() {
        double currentVelocity = shooterMotorLeft.getEncoder().getVelocity();
        double targetVelocity = 3000;
        double outputPower = shooterMotorController.calculate(currentVelocity, targetVelocity);
        return outputPower + targetVelocity/6140;
    }
}
