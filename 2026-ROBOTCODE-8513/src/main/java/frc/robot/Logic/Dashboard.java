package frc.robot.Logic;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AllianceSelector;
import frc.robot.Logic.Enums.TCPChooser;
import frc.robot.Logic.Enums.selfTestModeSelector;

public class Dashboard {
        public SendableChooser<String> TCPSelector;
        public TCPChooser tCPSelected = TCPChooser.autoDetectWinnerOfAuto;
        public SendableChooser<String> AllianceSelector;
        public AllianceSelector allianceSelected = frc.robot.Logic.Enums.AllianceSelector.redAlliance;

        // sendableChooser for self-test mode
        public SendableChooser<String> selfTestModeSelector;
        public selfTestModeSelector selfTestModeSelected = frc.robot.Logic.Enums.selfTestModeSelector.normalMode;

        public Field2d trajField2d = new Field2d();
        public Field2d copilotField2d = new Field2d();
        public Field2d scoreHubField2d = new Field2d();

        public Dashboard() {
                TCPSelector = new SendableChooser<>();
                TCPSelector.setDefaultOption(TCPChooser.values()[0].toString(), TCPChooser.values()[0].toString());
                for (int i = 1; i < TCPChooser.values().length; i++) {
                        if (TCPChooser.values()[i].toString().charAt(0) != '~') {
                                TCPSelector.addOption(TCPChooser.values()[i].toString(),
                                                TCPChooser.values()[i].toString());
                        }

                }
                SmartDashboard.putData("TCP Selector", TCPSelector);

                updateTCPConnectionFromDashboard();

                AllianceSelector = new SendableChooser<>();
                AllianceSelector.setDefaultOption(frc.robot.Logic.Enums.AllianceSelector.values()[0].toString(),
                                frc.robot.Logic.Enums.AllianceSelector.values()[0].toString());
                for (int i = 1; i < frc.robot.Logic.Enums.AllianceSelector.values().length; i++) {
                        if (frc.robot.Logic.Enums.AllianceSelector.values()[i].toString().charAt(0) != '~') {
                                AllianceSelector.addOption(
                                                frc.robot.Logic.Enums.AllianceSelector.values()[i].toString(),
                                                frc.robot.Logic.Enums.AllianceSelector.values()[i].toString());
                        }

                }
                SmartDashboard.putData("Manual Alliance Selector", AllianceSelector);
                updateAllianceFromDashboard();

                // self-test mode selector
                selfTestModeSelector = new SendableChooser<>();
                selfTestModeSelector.setDefaultOption(frc.robot.Logic.Enums.selfTestModeSelector.values()[0].toString(),
                                frc.robot.Logic.Enums.selfTestModeSelector.values()[0].toString());
                for (int i = 1; i < frc.robot.Logic.Enums.selfTestModeSelector.values().length; i++) {
                        if (frc.robot.Logic.Enums.selfTestModeSelector.values()[i].toString().charAt(0) != '~') {
                                selfTestModeSelector.addOption(
                                                frc.robot.Logic.Enums.selfTestModeSelector.values()[i].toString(),
                                                frc.robot.Logic.Enums.selfTestModeSelector.values()[i].toString());
                        }

                }
                SmartDashboard.putData("Manual Self-Test Mode Selector", selfTestModeSelector);
                updateSelfTestModeFromDashboard();
        }

        public void updateDashboard() {

                // Robot position
                SmartDashboard.putData("trajGoalPose", trajField2d);
                SmartDashboard.putNumber("actualHeading", Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees());
                SmartDashboard.putNumber("goalHeading", Robot.drivebase.goalHeading.getDegrees());
                SmartDashboard.putNumber("pitch", Robot.drivebase.yagslDrive.getPitch().getDegrees());

                // manual alliance selector
                SmartDashboard.putString("Selected Auto", allianceSelected.toString());

                // Auto selection
                SmartDashboard.putString("AutoRoutine", Robot.auto.autoRoutine.name());
                SmartDashboard.putString("Auto 1 selected", Robot.auto.dashboardAutoRoutine1.name());
                SmartDashboard.putString("Auto 2 selected", Robot.auto.dashboardAutoRoutine2.name());
                SmartDashboard.putBoolean("Auto Initialized", Robot.auto.pathInitialized);

                SmartDashboard.putNumber("autoStep", Robot.auto.autoStep);
                SmartDashboard.putString("autoToReturnTO", Robot.auto.autoToReturnTo.name());
                SmartDashboard.putNumber("timeInStep", Timer.getFPGATimestamp() - Robot.auto.timeStepStarted);

                // Subsystem states
                SmartDashboard.putString("intakeState", Robot.intake.intakeState.name());
                SmartDashboard.putString("shooterStates", Robot.shooter.shooterState.name());
                SmartDashboard.putString("kickerStates", Robot.kicker.kickerState.name());
                SmartDashboard.putString("hopperStates", Robot.hopper.hopperState.name());
                SmartDashboard.putData("copilotShuttlePosition", copilotField2d);
                SmartDashboard.putData("hubPos", scoreHubField2d);

                // Motor locations/positions
                // intake
                SmartDashboard.putNumber("intakeDeployPosition",
                                Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble());
                SmartDashboard.putNumber("intakerightvelocity",
                                Robot.intake.intakeMotorRightFollower.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("intakeleftvelocity",
                                Robot.intake.intakeMotorLeftLeader.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("left intake motor power", Robot.intake.intakeMotorLeftLeader.get());
                SmartDashboard.putNumber("right intake motor power", Robot.intake.intakeMotorRightFollower.get());
                SmartDashboard.putNumber("IntakeMotorCurrentLeft",
                                Robot.intake.intakeMotorLeftLeader.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("IntakeMotorCurrentRight",
                                Robot.intake.intakeMotorRightFollower.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("time shooter button pressed",
                                Robot.teleop.timeIntakeShootingButtonPressed);
                SmartDashboard.putNumber("adjusted intake pos", Robot.intake.adjustedEncoderPosition());

                SmartDashboard.putBoolean("Intake is Stowed", Robot.intake.intakeIsStowed());

                SmartDashboard.putNumber("beeftakeDeployControllerConstraints",
                                Robot.intake.beeftakeDeployController.getConstraints().maxVelocity);

                // shooter
                SmartDashboard.putNumber("shooterMotorLeftVelocity",
                                Robot.shooter.shooterMotorLeftLeader.getVelocity().getValueAsDouble());
                
                SmartDashboard.putNumber("shooterMotorOutsideLeftVelocity",
                                Robot.shooter.shooterMotorThirdFollower.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterMotorOutsideLeftCurrent",
                                Robot.shooter.shooterMotorThirdFollower.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("shooterMotorRightVelocity",
                                Robot.shooter.shooterMotorRightFollower.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("shooterHoodMotorPosition",
                                Robot.shooter.shooterHoodMotor.getEncoder().getPosition());
                SmartDashboard.putNumber("ShooterMotorCurrentLeft",
                                Robot.shooter.shooterMotorLeftLeader.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("ShooterMotorCurrentRight",
                                Robot.shooter.shooterMotorRightFollower.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("shooterHoodABSOLUTEMotorPosition",
                                Robot.shooter.shooterHoodMotor.getAbsoluteEncoder().getPosition());
                SmartDashboard.putNumber("manualTuningHoodPosition", Robot.shooter.manualTuningHoodPosition);
                SmartDashboard.putNumber("manualTargetV", Robot.shooter.manualTargetV);
                SmartDashboard.putNumber(" distance to hub", Robot.shooter.distanceBetweenCurrentAndGoalInMeters);
                SmartDashboard.putNumber("targetV", Robot.shooter.targetV);

                // kicker
                SmartDashboard.putNumber("kickerMotorVelocity",
                                Robot.kicker.kickerMotor.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("kickerMotorCurrent",
                                Robot.kicker.kickerMotor.getSupplyCurrent().getValueAsDouble());

                // hopper
                SmartDashboard.putNumber("hopperMotorVelocityTop",
                                Robot.hopper.indexerMotorTop.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("hopperMotorVelocityBottom",
                                Robot.hopper.indexerMotorBottom.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("hopperMotorCurrentBottom",

                                Robot.hopper.indexerMotorBottom.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("hopperMotorCurrentTop",

                                Robot.hopper.indexerMotorTop.getSupplyCurrent().getValueAsDouble());
                SmartDashboard.putNumber("Goal hood position", Robot.shooter.goalHoodPosition);

                // Fudge factors
                SmartDashboard.putNumber("shotDistanceFudgeFactorValue", Robot.shooter.shotDistanceFudgeFactor);
                SmartDashboard.putNumber("drivebaseAimFudgeFactorValue", Robot.drivebase.aimFudgeFactor);
                SmartDashboard.putNumber("intakeFudgeFactor", Robot.intake.intakeFudgeFactor);

                // Prompts user input for PID values
                SmartDashboard.putNumber("output shooter kP", Robot.shooter.slot0Configs.kP);
                SmartDashboard.putNumber("output shooter kI", Robot.shooter.slot0Configs.kI);
                SmartDashboard.putNumber("output shooter kD", Robot.shooter.slot0Configs.kD);

                SmartDashboard.putNumber("output intake kP", Robot.intake.intakeMotorController.getP());
                SmartDashboard.putNumber("output intake kI", Robot.intake.intakeMotorController.getI());
                SmartDashboard.putNumber("output intake kD", Robot.intake.intakeMotorController.getD());

                SmartDashboard.putNumber("output kicker kP", Robot.kicker.kickerMotorController.getP());
                SmartDashboard.putNumber("output kicker kI", Robot.kicker.kickerMotorController.getI());
                SmartDashboard.putNumber("output kicker kD", Robot.kicker.kickerMotorController.getD());

                SmartDashboard.putNumber("output indexer kP", Robot.hopper.indexerMotorController.getP());
                SmartDashboard.putNumber("output indexer kI", Robot.hopper.indexerMotorController.getI());
                SmartDashboard.putNumber("output indexer kD", Robot.hopper.indexerMotorController.getD());

                // match values
                SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
                SmartDashboard.putNumber("Time Left in Period", Robot.matchTimeAnalysis.getTimeLeftInPeriod());
                SmartDashboard.putString("Period", Robot.matchTimeAnalysis.getShift());
                SmartDashboard.putString("Hub Status", Robot.matchTimeAnalysis.activeOrInactive());
                activeOrInactiveColor();
                allianceColor();
                SmartDashboard.putString("Game data", DriverStation.getGameSpecificMessage());

                // self test mode
                isInSelfTestModeColor();

                // auto ready to shoot values
                SmartDashboard.putBoolean("hoodPositionReady", Robot.shooter.hoodPositionReady);
                SmartDashboard.putBoolean("velocityReady", Robot.shooter.velocityReady);
                SmartDashboard.putBoolean("facingHub",
                                Robot.shooter.facingHub(Settings.AutoSettings.Thresholds.drivebaseShootRotationTHold));
                SmartDashboard.putBoolean("timeCheckReadyToShoot", Robot.shooter.timeCheckReadyToShoot());

                // interpolated values
                SmartDashboard.putNumber("interpolatedHoodPosition", Robot.shooter.goalHoodPosition);
                SmartDashboard.putNumber("interpolatedShooterVelocity", Robot.shooter.goalShooterVelocity);
                SmartDashboard.putNumber("interpolatedTOF", Robot.drivebase.timeOfFlight);
                SmartDashboard.putNumber("distanceToGoal", Robot.shooter.distanceBetweenCurrentAndGoalInMeters);

                // get TCP communication
                isTCPConnectedColor();

                // camera color status
                cameraStatusLight(Robot.vision.leftCam);
                cameraStatusLight(Robot.vision.rightCam);
                cameraStatusLight(Robot.vision.leftShooterCam);
                cameraStatusLight(Robot.vision.rightShooterCam);

                updateSelfTestModeFromDashboard();

        }

        public void getPIDValues() {
                // get pid values from dashboard if the setting is enabled
                if (Settings.DrivebaseSettings.getPIDValuesFromDashboard) {

                        // set shooter pid values
                        Robot.shooter.slot0Configs.kP = SmartDashboard.getNumber("input shooter kP", 0);
                        Robot.shooter.slot0Configs.kI = SmartDashboard.getNumber("input shooter kI", 0);
                        Robot.shooter.slot0Configs.kD = SmartDashboard.getNumber("input shooter kD", 0);

                        // set intake pid values
                        Robot.intake.intakeMotorController.setP(SmartDashboard.getNumber("input intake kP", 0));
                        Robot.intake.intakeMotorController.setI(SmartDashboard.getNumber("input intake kI", 0));
                        Robot.intake.intakeMotorController.setD(SmartDashboard.getNumber("input intake kD", 0));

                        // set kicker pid values
                        Robot.kicker.kickerMotorController.setP(SmartDashboard.getNumber("input kicker kP", 0));
                        Robot.kicker.kickerMotorController.setI(SmartDashboard.getNumber("input kicker kI", 0));
                        Robot.kicker.kickerMotorController.setD(SmartDashboard.getNumber("input kicker kD", 0));

                        // set indexer pid values
                        Robot.hopper.indexerMotorController.setP(SmartDashboard.getNumber("input indexer kP", 0));
                        Robot.hopper.indexerMotorController.setI(SmartDashboard.getNumber("input indexer kI", 0));
                        Robot.hopper.indexerMotorController.setD(SmartDashboard.getNumber("input indexer kD", 0));

                }

        }

        public void activeOrInactiveColor() {
                Color red = new Color(255, 0, 0);
                Color green = new Color(0, 255, 0);
                if (Robot.matchTimeAnalysis.activeOrInactive() == "Active") {
                        SmartDashboard.putString("Color", green.toHexString());
                } else if (Robot.matchTimeAnalysis.activeOrInactive() == "Inactive") {
                        SmartDashboard.putString("Color", red.toHexString());
                }
        }

        public void allianceColor() {
                Color red = new Color(255, 0, 0);
                Color blue = new Color(0, 0, 255);
                if (Robot.onRed) {
                        SmartDashboard.putString("Alliance", red.toHexString());
                } else {
                        SmartDashboard.putString("Alliance", blue.toHexString());
                }
        }

        public void isInSelfTestModeColor() {
                Color red = new Color(255, 0, 0);
                Color green = new Color(0, 255, 0);
                if (Robot.isInSelfTestMode) {
                        SmartDashboard.putString("isInSelfTestMode", green.toHexString());
                } else {
                        SmartDashboard.putString("isInSelfTestMode", red.toHexString());
                }
        }

        public void isTCPConnectedColor() {
                ConnectionInfo[] TCPConnection = NetworkTableInstance.getDefault().getConnections();
                Color red = new Color(255, 0, 0);
                Color green = new Color(0, 255, 0);
                if (TCPConnection.length == 0) {
                        SmartDashboard.putString("TCP Connection", green.toHexString());
                } else {
                        SmartDashboard.putString("TCP Connection", red.toHexString());
                }
        }

        public void updateTCPConnectionFromDashboard() {
                try {
                        tCPSelected = TCPChooser.valueOf(TCPSelector.getSelected());
                } catch (Exception e) {
                        tCPSelected = TCPChooser.autoDetectWinnerOfAuto;
                }
        }

        public void cameraStatusLight(PhotonCamera photonCamera) {
                Color red = new Color(255, 0, 0);
                Color green = new Color(0, 255, 0);
                if (photonCamera.isConnected()) {
                        SmartDashboard.putString(photonCamera.getName() + "Status", green.toHexString());
                } else {
                        SmartDashboard.putString(photonCamera.getName() + "Status", red.toHexString());
                }
        }

        public void updateAllianceFromDashboard() {
                try {
                        allianceSelected = frc.robot.Logic.Enums.AllianceSelector
                                        .valueOf(AllianceSelector.getSelected());
                } catch (Exception e) {
                        allianceSelected = frc.robot.Logic.Enums.AllianceSelector.redAlliance;
                }

                if (allianceSelected == frc.robot.Logic.Enums.AllianceSelector.redAlliance) {
                        Robot.onRed = true;
                } else if (allianceSelected == frc.robot.Logic.Enums.AllianceSelector.blueAlliance) {
                        Robot.onRed = false;
                }

        }

        public void updateSelfTestModeFromDashboard() {
                try {
                        selfTestModeSelected = frc.robot.Logic.Enums.selfTestModeSelector
                                        .valueOf(selfTestModeSelector.getSelected());
                } catch (Exception e) {
                        selfTestModeSelected = frc.robot.Logic.Enums.selfTestModeSelector.normalMode;
                }

                if (selfTestModeSelected == frc.robot.Logic.Enums.selfTestModeSelector.normalMode) {
                        Robot.isInSelfTestMode = false;
                } else if (selfTestModeSelected == frc.robot.Logic.Enums.selfTestModeSelector.selfTestMode) {
                        Robot.isInSelfTestMode = true;
                }

        }

}
