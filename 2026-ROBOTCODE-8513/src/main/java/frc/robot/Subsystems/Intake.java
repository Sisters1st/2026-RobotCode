package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.IntakeStates;

public class Intake {
    public IntakeStates intakeState = IntakeStates.stowed;

    // class member variable
    public TalonFX intakeMotorLeftLeader = new TalonFX(32);
    public TalonFX intakeMotorRightFollower = new TalonFX(31);
    public TalonFX intakeDeployMotor = new TalonFX(33);

    public PIDController intakeMotorController = new PIDController(0.1, 0, 0);
    public ProfiledPIDController intakeDeployController = new ProfiledPIDController(0.1, 0, 0,
            Settings.IntakeSettings.deployConstraints);
    public ProfiledPIDController beeftakeDeployController = new ProfiledPIDController(4.5, 0, 0,
            Settings.IntakeSettings.deployConstraints);

    public boolean useManualIntakeControl = false;
    public double intakeFudgeFactor = 0;
    public double timeLeftStowedState;

    public DutyCycleEncoder intakeAbosoluteEncoder = new DutyCycleEncoder(1);

    // in init function, set slot 0 gains
    public Slot0Configs slot0Configs = new Slot0Configs();

    // create a velocity closed-loop request, voltage output, slot 0 configs
    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    public Intake() {
        // internal pid controller shooter motors
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.5; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.05; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        intakeMotorRightFollower
                .setControl(new Follower(intakeMotorLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        intakeMotorLeftLeader.getConfigurator().apply(slot0Configs);

        m_request.UpdateFreqHz = 1000;
        // disable FOC
        m_request.withEnableFOC(true);
    }

    public void setMotorPower() {

        double targetV = 100;
        SmartDashboard.putNumber("ShooterV", intakeMotorRightFollower.getVelocity().getValueAsDouble());

        if (intakeState == IntakeStates.intaking) {

            beeftakeDeployController.setConstraints(Settings.IntakeSettings.deployConstraints);

            // deploy intake
            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.deployPosition + intakeFudgeFactor));

            if (Robot.teleop.copilotJoystick1.getRawButton(Settings.TeleopSettings.ButtonIDs.intakeBackward)) {
                intakeMotorLeftLeader
                        .setControl(m_request.withVelocity(-targetV).withFeedForward(RPStoVoltage(RPStoVoltage(targetV))));
            } else {
                if (adjustedEncoderPosition() > Settings.IntakeSettings.spinBackwardsThreshold) {
                    intakeMotorLeftLeader
                            .setControl(new DutyCycleOut(0));
                } else {
                    // intake wheels on
                    intakeMotorLeftLeader
                            .setControl(m_request.withVelocity(targetV).withFeedForward(RPStoVoltage(targetV)));
                }
            }

        } else if (intakeState == IntakeStates.stowed) {

            beeftakeDeployController.setConstraints(Settings.IntakeSettings.deployConstraints);

            // stow intake
            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.stowPosition + intakeFudgeFactor));

            // intake wheels off
            intakeMotorLeftLeader.set(0);

            timeLeftStowedState = Timer.getFPGATimestamp();

        } else if (intakeState == IntakeStates.outtaking) {

            beeftakeDeployController.setConstraints(Settings.IntakeSettings.deployConstraints);

            // deploy intake
            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.deployPosition + intakeFudgeFactor));

            // intake wheels on
            intakeMotorLeftLeader
                    .setControl(m_request.withVelocity(-targetV).withFeedForward(RPStoVoltage(RPStoVoltage(targetV))));

        } else if (intakeState == IntakeStates.stationaryDeployed) {
            if (Robot.teleop.copilotJoystick1.getRawButton(Settings.TeleopSettings.ButtonIDs.intakeBackward)) {
                intakeMotorLeftLeader
                        .setControl(m_request.withVelocity(-targetV).withFeedForward(RPStoVoltage(RPStoVoltage(targetV))));
            } else {
                intakeMotorLeftLeader.set(0);

            }// intake wheels off
            beeftakeDeployController.setConstraints(Settings.IntakeSettings.deployConstraints);
            // deploy intake
            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.deployPosition + intakeFudgeFactor));

            

        } else if (intakeState == IntakeStates.shooting) {

            beeftakeDeployController.setConstraints(Settings.IntakeSettings.shootingConstraints);

            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.shootingPosition + intakeFudgeFactor));

            // intake wheels off
            intakeMotorLeftLeader.set(0);
        } else if (intakeState == IntakeStates.maxPositionWhenShooting) {

            beeftakeDeployController.setConstraints(Settings.IntakeSettings.shootingConstraints);

            intakeDeployMotor.set(deployBeeftake(Settings.IntakeSettings.maxIntakePositionToNotHitHood));

            // intake wheels off
            intakeMotorLeftLeader.set(0);
        }

        // TEMPORARY: manual joystick control for intake
        if (useManualIntakeControl) {
            double intakeJoystickValue = Robot.teleop.manualJoystick.getRawAxis(5) * 0.2;

            if (Math.abs(intakeJoystickValue) > Settings.TeleopSettings.joystickDeadband) {
                intakeDeployMotor.set(intakeJoystickValue);
                double currentPosition = intakeDeployMotor.getPosition().getValueAsDouble();
                intakeMotorController.setSetpoint(currentPosition);
            } else {
                double power = intakeMotorController.calculate(intakeDeployMotor.getPosition().getValueAsDouble());
                intakeDeployMotor.set(power);
            }

            if (Robot.teleop.manualJoystick.getRawButtonPressed(8)) {
                intakeDeployMotor.setPosition(0);
            }
        }
    }

    public double deployPower(double targetPosition) {
        double currentPosition = intakeDeployMotor.getPosition().getValueAsDouble();
        double outputPower = intakeDeployController.calculate(currentPosition, targetPosition);
        return outputPower;
    }

    // public void spinIntakeBackward() {
    // if (intakeDeployMotor.getPosition().getValueAsDouble() < 0) {
    // intakeMotorLeftLeader.setControl(m_request.withOutput(-1.0));
    // }
    // }

    // copied from shooter, check the values
    public double RPStoVoltage(double RPS) {
        double voltage = (RPS + 0.773138) / 8.70426;
        return voltage;
    }

    public double deployBeeftake(double targetPosition) {
        double currentPosition = adjustedEncoderPosition();
        double outputPower = beeftakeDeployController.calculate(currentPosition, targetPosition);
        return -outputPower;
    }

    public double adjustedEncoderPosition() {
        //this needs to get tuned for new intake positions
        double currentPosition = intakeAbosoluteEncoder.get();
        double adjustedPosition;

        if (currentPosition  > 0.2) {
            adjustedPosition = currentPosition - 0.35;
        } else {
            adjustedPosition = currentPosition + 1 - 0.35;
        }
        return adjustedPosition;
    }

    public boolean intakeIsStowed() {
        if (adjustedEncoderPosition() > Settings.IntakeSettings.maxIntakePositionToNotHitHood) {
            return true;
        } else {
            return false;
        }
    }

}
