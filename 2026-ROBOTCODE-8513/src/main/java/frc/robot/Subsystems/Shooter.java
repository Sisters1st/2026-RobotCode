package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.ShooterStates;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Shooter {

    public TalonFX shooterMotorLeftLeader = new TalonFX(13);
    public TalonFX shooterMotorRightFollower = new TalonFX(14);
    public TalonFX shooterMotorThirdFollower = new TalonFX(16);
    public SparkMax shooterHoodMotor = new SparkMax(15, MotorType.kBrushless);

    public PIDController shooterMotorController = new PIDController(1.5, 0.1, 0);

    public ShooterStates shooterState = ShooterStates.stationary;

    public boolean useInternalController = true;

    public double shotDistanceFudgeFactor = 0;
    public double targetV = 0;

    public double goalHoodPosition;
    double readyToShootInHubCounter = 0;

    public double distanceToScoreHub;
    public double goalShooterVelocity;
    public boolean manualShooterTuning = false;
    public double manualTuningHoodPosition = Settings.ShooterSettings.highestHoodPosition;
    public boolean manualHoodTuning = manualShooterTuning;
    public double distanceBetweenCurrentAndGoalInMeters;

    // ready to shoot checks
    public boolean hoodPositionReady = false;
    public boolean velocityReady = false;

    // in init function, set slot 0 gains
    public Slot0Configs slot0Configs = new Slot0Configs();

    // create a velocity closed-loop request, voltage output, slot 0 configs
    public final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    // interpolation
    public InterpolatingDoubleTreeMap distToHoodEncoderValuesTable = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap distToshooterVelocityEncoderValuesTable = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap distToTimeOfFlightValuesTable = new InterpolatingDoubleTreeMap();

    public Shooter() {
        // internal pid controller shooter motors
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.55; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.05; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // interpolating
        //dist,hood
        distToHoodEncoderValuesTable.put(1.55, 0.90);
        distToHoodEncoderValuesTable.put(2.06, 0.85);
        distToHoodEncoderValuesTable.put(2.57, 0.79);
        distToHoodEncoderValuesTable.put(2.90, 0.71);
        distToHoodEncoderValuesTable.put(3.48, 0.69);
        distToHoodEncoderValuesTable.put(3.95, 0.60);
        distToHoodEncoderValuesTable.put(4.74, 0.56);
        
        distToHoodEncoderValuesTable.put(5.00, 0.4);
        distToHoodEncoderValuesTable.put(9.00, 0.25);

        //dist.veloc
        distToshooterVelocityEncoderValuesTable.put(1.4, 35.00);
        distToshooterVelocityEncoderValuesTable.put(1.55, 36.00);
        distToshooterVelocityEncoderValuesTable.put(2.06, 37.50);
        distToshooterVelocityEncoderValuesTable.put(2.57, 39.5);
        distToshooterVelocityEncoderValuesTable.put(2.90, 41.0);
        distToshooterVelocityEncoderValuesTable.put(3.48, 43.0);
        distToshooterVelocityEncoderValuesTable.put(3.95, 44.75);
        distToshooterVelocityEncoderValuesTable.put(4.74, 46.75);
        
        distToshooterVelocityEncoderValuesTable.put(5.0, 51.0);
        distToshooterVelocityEncoderValuesTable.put(9.0, 56.0);
        

        distToTimeOfFlightValuesTable.put(2.05, 0.78);
        distToTimeOfFlightValuesTable.put(2.49, 0.9);
        distToTimeOfFlightValuesTable.put(3.52, 1.13);
        distToTimeOfFlightValuesTable.put(3.94, 1.21);

        // internal pid
        shooterMotorLeftLeader.getConfigurator().apply(slot0Configs);

        // follower leader set up
        shooterMotorRightFollower
                .setControl(new Follower(shooterMotorLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        shooterMotorThirdFollower
                .setControl(new Follower(shooterMotorLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

        m_request.UpdateFreqHz = 500;

    }

    public void setMotorPower() {

        // shooter controller if using interal pid
        targetV = -35;

        if (shooterState == ShooterStates.shooting && useInternalController == true) {
            setCurrentLimits(80, 40);
            targetV = targetV + shotDistanceFudgeFactor;
            if (manualShooterTuning == false) {
                shooterMotorLeftLeader
                        .setControl(m_request.withVelocity(targetV).withFeedForward(RPStoVoltage(targetV)));

            } else {
                manualShooterTuning();
            }

        } else if (shooterState == ShooterStates.stationary && useInternalController == true) {
            if (Robot.teleop.copilotJoystick2.getRawButton(Settings.TeleopSettings.ButtonIDs.nuetralZoneButton1) && Robot.teleop.copilotJoystick2.getRawButton(Settings.TeleopSettings.ButtonIDs.nuetralZoneButton2)) {
                setCurrentLimits(40, 10);
                shooterMotorLeftLeader.setControl(m_request.withVelocity(-35).withFeedForward(RPStoVoltage(-35)));
            } else {
                shooterMotorLeftLeader.set(0);

            }
        }

        setHoodAngle();
    }

    public void setHoodAngle() {
        distanceBetweenCurrentAndGoalInMeters = Robot.drivebase
                .getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(), Robot.drivebase.goalAimPose);
        double hoodPos = Settings.ShooterSettings.highestHoodPosition;
        if (manualHoodTuning == false) {
            if (Robot.intake.intakeIsStowed() || shooterState == ShooterStates.stationary) {
                shooterHoodMotor
                        .set(-hoodAnglePower(Settings.ShooterSettings.highestHoodPosition));
            } else {
                shooterHoodMotor
                        .set(-hoodAnglePower(hoodPos + Robot.drivebase.aimFudgeFactor));
            }

        } else {
            shooterHoodMotor
                    .set(-hoodAnglePower(manualTuningHoodPosition));
        }
        goalHoodPosition = getInterpolatedEncoderValueDistanceToHood(distanceBetweenCurrentAndGoalInMeters);
    }

    public double getInterpolatedShooterVelocity() {
        distanceBetweenCurrentAndGoalInMeters = Robot.drivebase
                .getDistanceBetweenTwoPoses(Robot.drivebase.yagslDrive.getPose(), Robot.drivebase.goalAimPose);
        goalShooterVelocity = getInterpolatedShooterVelocityFromDistance(distanceBetweenCurrentAndGoalInMeters);
        return goalShooterVelocity;
    }

    // input target encoder position from the interpolation chart, PIDs, then
    // returns the power to maintain that position
    public double hoodAnglePower(double targetPosition) {
        double currentPosition = shooterHoodMotor.getAbsoluteEncoder().getPosition();
        if (targetPosition < Settings.ShooterSettings.lowestHoodPosition) {
            targetPosition = Settings.ShooterSettings.lowestHoodPosition;
        } else if (targetPosition > Settings.ShooterSettings.highestHoodPosition) {
            targetPosition = Settings.ShooterSettings.highestHoodPosition;
        }
        double outputPower = shooterMotorController.calculate(currentPosition, targetPosition);
        return outputPower;
    }

    // input distance, returns encoder position
    public double getInterpolatedEncoderValueDistanceToHood(double distanceFromGoal) {
        distanceToScoreHub = distToHoodEncoderValuesTable.get(distanceFromGoal + shotDistanceFudgeFactor);
        return distanceToScoreHub;
    }

    // input distance, return shooterVelocity
    public double getInterpolatedShooterVelocityFromDistance(double distanceFromGoal) {
        double velocityToScoreHub = distToshooterVelocityEncoderValuesTable.get(distanceFromGoal);
        return velocityToScoreHub;
    }

    // input distance, return timeOfFlight
    public double getInterpolatedTimeOfFlightFromDistance(double distanceFromGoal) {
        double TOFToScoreHub = distToTimeOfFlightValuesTable.get(distanceFromGoal);
        return TOFToScoreHub;
    }

    public double RPStoVoltage(double RPS) {
        double voltage = (RPS + 0.773138) / 8.70426;
        return voltage;
    }

    // Checks the anle error, hood position error, and shooter velocity error to
    // determine if the shooter is ready to shoot. Thresholds for each of these
    // values are in settings.
    public boolean readyToShootInHub() {
        if (Math.abs(goalHoodPosition - shooterHoodMotor.getAbsoluteEncoder()
                .getPosition()) < Settings.AutoSettings.Thresholds.shootHoodPositionTHold) {
            hoodPositionReady = true;
        } else {
            hoodPositionReady = false;
        }

        if (Math.abs(shooterMotorLeftLeader.getVelocity().getValueAsDouble()
                - targetV) < Settings.AutoSettings.Thresholds.shooterVelocityTHold) {
            velocityReady = true;
        } else {
            velocityReady = false;
        }

        if (facingHub(Settings.AutoSettings.Thresholds.drivebaseShootRotationTHold)
                && hoodPositionReady
                && velocityReady
                && timeCheckReadyToShoot()) {
            readyToShootInHubCounter++;
        } else {
            readyToShootInHubCounter = 0;
        }

        if (readyToShootInHubCounter >= 4) {
            return true;
        } else {
            return false;
        }
    }

    public boolean facingHub(double precision) {
        if (Math.abs(Robot.drivebase.yagslDrive.getOdometryHeading().minus(Robot.drivebase.goalHeading)
                .getDegrees()) < precision) {
            return true;
        } else {
            return false;
        }
    }

    public boolean readyToShuttle() {
        if (facingHub(Settings.AutoSettings.Thresholds.drivebaseShuttleRotationTHold)
                && (Math.abs(goalHoodPosition - shooterHoodMotor.getEncoder()
                        .getPosition()) < Settings.AutoSettings.Thresholds.shuttleHoodPositionTHold)
                && (Math.abs(shooterMotorRightFollower.getVelocity().getValueAsDouble()
                        - targetV) < Settings.AutoSettings.Thresholds.shooterShuttleVelocityTHold)) {
            return true;
        }
        return false;
    }

    public boolean timeCheckReadyToShoot() {
        if (Robot.matchTimeAnalysis.activeOrInactive().equals("Active")
                || (Robot.matchTimeAnalysis.activeOrInactive().equals("Inactive")
                        && Robot.matchTimeAnalysis.getTimeLeftInPeriod() <= 1.5)) {
            return true;
        } else {
            return false;
        }
    }

    public void setCurrentLimits(double statorLimit, double supplyLimit) {
        var configs = new CurrentLimitsConfigs();
        configs.StatorCurrentLimitEnable = true;
        configs.StatorCurrentLimit = statorLimit;
        configs.SupplyCurrentLimitEnable = true;
        configs.SupplyCurrentLimit = supplyLimit;
        configs.SupplyCurrentLowerLimit = supplyLimit;
        configs.SupplyCurrentLowerTime = 3;

        shooterMotorLeftLeader.getConfigurator().apply(configs);
        shooterMotorRightFollower.getConfigurator().apply(configs);
        shooterMotorThirdFollower.getConfigurator().apply(configs);

    }

    public double manualTargetV = 25;

    public void manualShooterTuning() {
        if (Robot.teleop.tuningJoystick
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shooterManualIncreaseVelocity)) {
            manualTargetV = manualTargetV + Settings.ShooterSettings.manualVelocityTuningFactor;
        } else if (Robot.teleop.tuningJoystick
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shooterManualDecreaseVelocity)) {
            manualTargetV = manualTargetV - Settings.ShooterSettings.manualVelocityTuningFactor;
        }

        if (Robot.teleop.tuningJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.incHoodPos)) {
            Robot.shooter.manualTuningHoodPosition += Settings.ShooterSettings.manualHoodPosTuningfactor;
        } else if (Robot.teleop.tuningJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.decHoodPos)) {
            Robot.shooter.manualTuningHoodPosition -= Settings.ShooterSettings.manualHoodPosTuningfactor;

        }

        shooterMotorLeftLeader
                .setControl(m_request.withVelocity(-manualTargetV).withFeedForward(-RPStoVoltage(manualTargetV)));
    }

    public void gradualSpinUp() {
        if (Timer.getFPGATimestamp() - Robot.teleop.timeGradualWasPressed <= 0.5)
            setCurrentLimits(20, 30);
    }
}