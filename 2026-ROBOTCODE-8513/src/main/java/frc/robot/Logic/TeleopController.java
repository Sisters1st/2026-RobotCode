package frc.robot.Logic;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;
import frc.robot.Logic.Enums.ShooterStates;

public class TeleopController {

    public Joystick driverXboxController = new Joystick(Settings.TeleopSettings.driverJoystickPort);
    public Joystick copilotJoystick1 = new Joystick(Settings.TeleopSettings.copilotJoystick1Port);
    public Joystick copilotJoystick2 = new Joystick(Settings.TeleopSettings.copilotJoystick2Port);
    public Joystick manualJoystick = new Joystick(Settings.TeleopSettings.manualJoystickPort);
    public Joystick tuningJoystick = new Joystick(Settings.TeleopSettings.tuningJoystickPort);

    public PIDController rJoystickController = new PIDController(0.1, 0, 0);
    public PIDController bumpPidController = new PIDController(10, 0, 0);

    public double timeIncreaseShotPressed;
    public double timeDecreaseShotPressed;
    public double timeRightMoveScorePosePressed;
    public double timeLeftMoveScorePosePressed;

    public Rotation2d goalHeading = new Rotation2d();

    SlewRateLimiter xfilter = new SlewRateLimiter(4);
    SlewRateLimiter yfilter = new SlewRateLimiter(4);
    SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public boolean shootingFacingHub = false;
    public boolean useSpecialNewRotation = true;
    public boolean autoShooting = true;
    public double timeGradualWasPressed;
    public double timeIntakeShootingButtonPressed;
    public boolean unjam = false;

    public String selfTestModeFailPoint;
    public int testStep = 0;

    public double shooterButtonTime;

    public Pose2d copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.neutralZone1;

    public double xV;
    public double yV;
    public double rV;

    public double teleStartTime;

    public void initTele() {
        Robot.updateAlliance();
        Robot.dashboard.getPIDValues();
        Elastic.selectTab("Teleoperated");
        teleStartTime = Timer.getFPGATimestamp();

        Robot.shooter.shooterState = ShooterStates.stationary;
        Robot.hopper.hopperState = HopperStates.stationary;
        Robot.kicker.kickerState = KickerStates.stationary;

        Robot.intake.intakeDeployController.reset(Robot.intake.adjustedEncoderPosition());

        if (Robot.intake.adjustedEncoderPosition() < 0.1) {
            Robot.intake.intakeState = IntakeStates.intaking;
        } else {
            Robot.intake.intakeState = IntakeStates.stowed;
        }

        // consume all buttons pressed
        for (int i = 0; i < 14; i++) {
            driverXboxController.getRawButtonPressed(i);
            copilotJoystick1.getRawButtonPressed(i);
            copilotJoystick2.getRawButtonPressed(i);
            manualJoystick.getRawButtonPressed(i);

            driverXboxController.getRawButtonReleased(i);
            copilotJoystick1.getRawButtonReleased(i);
            copilotJoystick2.getRawButtonReleased(i);
            manualJoystick.getRawButtonReleased(i);

        }

        // sets the goal aim pose to the hub (when the aim pose is change bc of
        // velocity, it is updated)
        if (Robot.onRed) {
            Robot.drivebase.goalAimPose = Settings.FieldInfo.redHubCenterPoint;

        } else {
            Robot.drivebase.goalAimPose = Settings.FieldInfo.blueHubCenterPoint;

        }

        Robot.drivebase.goalHeading = Robot.drivebase.yagslDrive.getOdometryHeading();
        Robot.drivebase.rotationPidController.reset(0);

    }

    public double xSpeedJoystick;
    public double ySpeedJoystick;

    public void driveTele() {

        // drivebase controls
        xSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.forwardBackwardsAxis); // forward
                                                                                                         // back
        if (xSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && xSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        ySpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.leftRightAxis); // left right
        if (ySpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && ySpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            ySpeedJoystick = 0;

        }
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);

        double rSpeedJoystick = -driverXboxController.getRawAxis(Settings.TeleopSettings.rotAxis); // left right 2 at
                                                                                                   // home, 4 on xbox
        double rySpeedJoystick = driverXboxController.getRawAxis(Settings.TeleopSettings.RleftRightAxis);
        double rxSpeedJoystick = driverXboxController.getRawAxis(Settings.TeleopSettings.RforwardBackwardsAxis);

        if (rSpeedJoystick < Settings.TeleopSettings.joystickDeadband
                && rSpeedJoystick > -Settings.TeleopSettings.joystickDeadband) {
            rSpeedJoystick = 0;
        }

        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);

        // cube the joystick values for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        // if facing hub button, slow speed to 0.3

        if (shootingFacingHub) {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * Settings.TeleopSettings.drivingWhileShootingSpeed);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * Settings.TeleopSettings.drivingWhileShootingSpeed);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * Settings.TeleopSettings.drivingWhileShootingSpeed;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity()
                        * Settings.TeleopSettings.drivingWhileShootingSpeed;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        } else {
            if (Robot.onRed) {
                xV = -(xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.85);
                yV = -(yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.85);
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            } else {
                xV = xInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.85;
                yV = yInput * Robot.drivebase.yagslDrive.getMaximumChassisVelocity() * 0.85;
                rV = rInput * Robot.drivebase.yagslDrive.getMaximumChassisAngularVelocity();
            }
        }

        // special rotation
        if (Math.hypot(rySpeedJoystick,
                rxSpeedJoystick) < Settings.TeleopSettings.specialRotationJoystickDeadband) {
            Robot.drivebase.goalHeading = Robot.drivebase.goalHeading;
        } else {
            Rotation2d rotation;
            if (Robot.onRed) {
                rotation = new Rotation2d(rySpeedJoystick, rxSpeedJoystick);
            } else {
                rotation = new Rotation2d(-rySpeedJoystick, -rxSpeedJoystick);
            }
            Robot.drivebase.goalHeading = rotation;
        }

        // if face hub button is pressed, rotate to hub then set the kicker and hopper
        // states
        if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.faceGoal)) {
            if (Robot.onRed) {
                if (Robot.drivebase.yagslDrive.getPose().getX() > 11.8) {
                    shootingFacingHub = true;
                    // lcoks pose if no driver translation input
                    Robot.drivebase.getPowerToFaceHub();
                    if (Math.abs(xV) < 0.1 && Math.abs(yV) < 0.1 && Robot.shooter
                            .facingHub(Settings.AutoSettings.Thresholds.drivebaseLockPoseWhenShootingThold)) {
                        Robot.drivebase.yagslDrive.lockPose();
                    } else {
                        Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(),
                                true,
                                false);
                    }
                } else {
                    Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV),
                            Robot.drivebase.getPowerToFacePose(copilotShuttlePosition), true, false);
                }
            } else {
                if (Robot.drivebase.yagslDrive.getPose().getX() < 4.66) {
                    shootingFacingHub = true;
                    Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(),
                            true,
                            false);
                    // lcoks pose if no driver translation input
                    Robot.drivebase.getPowerToFaceHub();
                    if (Math.abs(xV) < 0.1 && Math.abs(yV) < 0.1 && Robot.shooter
                            .facingHub(Settings.AutoSettings.Thresholds.drivebaseLockPoseWhenShootingThold)) {
                        Robot.drivebase.yagslDrive.lockPose();
                    } else {
                        Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), Robot.drivebase.getPowerToFaceHub(),
                                true,
                                false);
                    }
                } else {
                    Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV),
                            Robot.drivebase.getPowerToFacePose(copilotShuttlePosition), true, false);
                }
            }
            Robot.shooter.shooterState = ShooterStates.shooting;
            if ((Robot.shooter.readyToShuttle() || Robot.shooter.readyToShootInHub()) && autoShooting
                    && Robot.shooter.shooterState == ShooterStates.shooting && !unjam) {
                Robot.kicker.kickerState = KickerStates.shooting;
                Robot.hopper.hopperState = HopperStates.indexing;
            } else {
                Robot.kicker.kickerState = KickerStates.stationary;
                Robot.hopper.hopperState = HopperStates.stationary;
            }

            // if straighten bump pressed, check rotation and switch to nearest 0 or 180
        } else if (driverXboxController.getRawButton(Settings.TeleopSettings.ButtonIDs.straightenBump)) {
            Robot.shooter.shooterState = ShooterStates.stationary;
            double currentY = Robot.drivebase.yagslDrive.getPose().getY();
            if (Math.abs(Robot.drivebase.yagslDrive.getOdometryHeading().getDegrees()) <= 90) {
                // y correction for driving over bump
                if (currentY >= 4.08) {
                    Robot.drivebase.yagslDrive.drive(
                            new Translation2d(xV, bumpPidController.calculate(currentY, 5.555)),
                            Robot.drivebase.getPowerToReachRotation(new Rotation2d(0)), true,
                            false);
                    Robot.drivebase.goalHeading = new Rotation2d();

                } else if (currentY < 4.08) {
                    Robot.drivebase.yagslDrive.drive(
                            new Translation2d(xV, bumpPidController.calculate(currentY, 2.476)),
                            Robot.drivebase.getPowerToReachRotation(new Rotation2d(0)), true,
                            false);
                    Robot.drivebase.goalHeading = new Rotation2d();

                }
                Robot.drivebase.goalHeading = new Rotation2d(0);
            } else {
                if (currentY >= 4.08) {
                    Robot.drivebase.yagslDrive.drive(
                            new Translation2d(xV, bumpPidController.calculate(currentY, 5.555)),
                            Robot.drivebase.getPowerToReachRotation(new Rotation2d(Math.PI)), true,
                            false);
                    Robot.drivebase.goalHeading = new Rotation2d(Math.PI);

                } else if (currentY < 4.08) {
                    Robot.drivebase.yagslDrive.drive(
                            new Translation2d(xV, bumpPidController.calculate(currentY, 2.476)),
                            Robot.drivebase.getPowerToReachRotation(new Rotation2d(Math.PI)), true,
                            false);
                    Robot.drivebase.goalHeading = new Rotation2d(Math.PI);
                }
                // otherwise, use the normal drive
            }
        } else if (driverXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.snakeMode)) {
            if (Math.abs(xSpeedJoystick) < Settings.TeleopSettings.snakeModeJoystickDeadband
                    && Math.abs(ySpeedJoystick) < Settings.TeleopSettings.snakeModeJoystickDeadband) {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), Robot.drivebase.goalHeading, true);
            } else {
                Rotation2d snakeModeRotation = new Translation2d(xV, yV).getAngle();
                Robot.drivebase
                        .driveFacingHeading(new Translation2d(xV * Settings.TeleopSettings.snakeModeVelocityFactor,
                                yV * Settings.TeleopSettings.snakeModeVelocityFactor), snakeModeRotation, true);
                Robot.drivebase.goalHeading = snakeModeRotation;
            }
            Robot.shooter.shooterState = ShooterStates.stationary;

        } else {
            Robot.shooter.shooterState = ShooterStates.stationary;
            shootingFacingHub = false;
            Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV),
                    Robot.drivebase.goalHeading, true);
            // Robot.drivebase.yagslDrive.drive(new Translation2d(xV, yV), rV, true, false);
        }

        if (driverXboxController.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.resetHeading)) {
            if (Robot.onRed) {
                Robot.drivebase.yagslDrive.resetOdometry(
                        new Pose2d(Robot.drivebase.yagslDrive.getPose().getTranslation(), new Rotation2d(Math.PI)));

            } else {
                Robot.drivebase.yagslDrive.resetOdometry(
                        new Pose2d(Robot.drivebase.yagslDrive.getPose().getTranslation(), new Rotation2d()));

            }
        }

        // when the driver releasese the shooting button the mechanisms stop
        if (driverXboxController.getRawButtonReleased(Settings.TeleopSettings.ButtonIDs.faceGoal)) {
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
            Robot.intake.intakeState = IntakeStates.intaking;
        }

        buttonControls();
        Robot.dashboard.copilotField2d.setRobotPose(copilotShuttlePosition.getX(), copilotShuttlePosition.getY(),
                copilotShuttlePosition.getRotation());

        // Subsystem set motor power
        Robot.intake.setMotorPower();
        Robot.shooter.setMotorPower();
        Robot.kicker.setMotorPower();
        Robot.hopper.setMotorPower();
    }

    // button pressed --> state machine
    public void buttonControls() {
        // intake controls
        boolean deployIntakeButtonPressed = Robot.teleop.driverXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.deployIntake);
        boolean stowIntakeButtonPressed = Robot.teleop.driverXboxController
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.stowIntake);
        if (deployIntakeButtonPressed) {
            Robot.intake.intakeState = IntakeStates.intaking;
        }
        if (stowIntakeButtonPressed) {
            // this should check if hood is below some safe threshold
            if (Robot.shooter.shooterHoodMotor.getAbsoluteEncoder()
                    .getPosition() < Settings.ShooterSettings.lowestPositionIntakeCanComeBack) {
                Robot.intake.intakeState = IntakeStates.maxPositionWhenShooting;
            } else {
                Robot.intake.intakeState = IntakeStates.stowed;
            }
        }

        // button when driver hits x face right and B face left
        if (Robot.teleop.driverXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.faceRight)) {
            if (Robot.onRed) {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), new Rotation2d(Math.toRadians(90)), true);
            } else {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), new Rotation2d(Math.toRadians(-90)),
                        true);
            }
        }
        if (Robot.teleop.driverXboxController
                .getRawButton(Settings.TeleopSettings.ButtonIDs.faceLeft)) {
            if (Robot.onRed) {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), new Rotation2d(Math.toRadians(-90)),
                        true);
            } else {
                Robot.drivebase.driveFacingHeading(new Translation2d(xV, yV), new Rotation2d(Math.toRadians(90)), true);
            }
        }

        // COPILOT CONTROLS
        // adjustment for shooter hood angle
        if (copilotJoystick1.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.increaseShotDistance)) {
            timeIncreaseShotPressed = Timer.getFPGATimestamp();
            Robot.shooter.shotDistanceFudgeFactor += Settings.ShooterSettings.shotDistanceFudgeDelta;
        }

        if (copilotJoystick1.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.decreaseShotDistance)) {
            timeDecreaseShotPressed = Timer.getFPGATimestamp();
            Robot.shooter.shotDistanceFudgeFactor -= Settings.ShooterSettings.shotDistanceFudgeDelta;
        }

        if (Math.abs(timeDecreaseShotPressed - timeIncreaseShotPressed) < 0.15) {
            Robot.shooter.shotDistanceFudgeFactor = 0;
        }

        // MANUAL JOYSTICK intake Fudge Factor
        if (manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.heightenIntake)) {
            Robot.intake.intakeFudgeFactor -= Settings.IntakeSettings.intakeFudgeFactor;
        }

        if (manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.lowerIntake)) {
            Robot.intake.intakeFudgeFactor += Settings.IntakeSettings.intakeFudgeFactor;
        }

        // adjustment for drivebase goal aim fudge factor
        if (copilotJoystick1.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseRight)) {
            timeRightMoveScorePosePressed = Timer.getFPGATimestamp();
            Robot.drivebase.aimFudgeFactor -= Settings.ShooterSettings.angleFudgeDelta;
        }

        if (copilotJoystick1.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.moveScorePoseLeft)) {
            timeLeftMoveScorePosePressed = Timer.getFPGATimestamp();
            Robot.drivebase.aimFudgeFactor += Settings.ShooterSettings.angleFudgeDelta;
        }

        if (Math.abs(timeRightMoveScorePosePressed - timeLeftMoveScorePosePressed) < 0.15) {
            Robot.drivebase.aimFudgeFactor = 0;
        }

        // Intake copilot controls
        boolean copilotStowIntakeButtonPressed = Robot.teleop.copilotJoystick1
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.copilotStowIntake);
        boolean copilotDeplotIntakeButtonPressed = Robot.teleop.copilotJoystick1
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.copilotDeployIntake);
        boolean copilotStationaryDeployIntakeButtonPressed = Robot.teleop.copilotJoystick1
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.copilotStopIntakeWheels);
        if (copilotStowIntakeButtonPressed) {
            // this should check if hood is below some safe threshold
            if (Robot.shooter.shooterHoodMotor.getAbsoluteEncoder()
                    .getPosition() < Settings.ShooterSettings.lowestPositionIntakeCanComeBack) {
                Robot.intake.intakeState = IntakeStates.maxPositionWhenShooting;
            } else {
                Robot.intake.intakeState = IntakeStates.stowed;
            }
        }
        if (copilotDeplotIntakeButtonPressed) {
            // if the robot is intaking or stationary
            Robot.intake.intakeState = IntakeStates.intaking;
        }
        if (copilotStationaryDeployIntakeButtonPressed) {
            // if the robot is intaking or stationary
            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
        }

        if (Robot.teleop.copilotJoystick1
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.jiggleIntake)) {
            Robot.intake.intakeState = IntakeStates.shooting;
            timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
        }

        // shooter controls
        boolean shootButtonPressed = copilotJoystick1
                .getRawButton(Settings.TeleopSettings.ButtonIDs.forceShoot);
        if (shootButtonPressed && Robot.shooter.shooterState == ShooterStates.shooting) {
            Robot.kicker.kickerState = KickerStates.shooting;
            Robot.hopper.hopperState = HopperStates.indexing;
        }
        boolean dontShootButtonPresseed = copilotJoystick1
                .getRawButton(Settings.TeleopSettings.ButtonIDs.forceDontShoot);
        if (dontShootButtonPresseed) {
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
        }

        // reverse everything
        boolean reverseKicker = copilotJoystick1
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.reverseEverything);
        boolean releasedKicker = copilotJoystick1
                .getRawButtonReleased(Settings.TeleopSettings.ButtonIDs.reverseEverything);
        if (reverseKicker && Robot.kicker.kickerState == KickerStates.shooting) {
            unjam = true;
            Robot.kicker.kickerState = KickerStates.intaking;
            Robot.hopper.hopperState = HopperStates.unjam;
        } else if (reverseKicker) {
            unjam = true;
            Robot.kicker.kickerState = KickerStates.intaking;
            Robot.hopper.hopperState = HopperStates.unjam;
            Robot.intake.intakeState = IntakeStates.outtaking;
        } else if (releasedKicker) {
            unjam = false;
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;
            Robot.intake.intakeState = IntakeStates.intaking;
        }

        // shuttle position buttons
        boolean redDepotTrenchButtonPressed = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.redDepotTrenchButton);
        boolean blueDepotTrenchButtonPressed = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.blueDepotTrenchButton);
        boolean redOutpostTrenchButtonPressed = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.redOutpostTrenchButton);
        boolean blueOutpostTrenchButtonPressed = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.blueOutpostTrenchButton);
        boolean nuetralZoneButtonPressed1 = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.nuetralZoneButton1);
        boolean nuetralZoneButtonPressed2 = copilotJoystick2
                .getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.nuetralZoneButton2);
        if (redDepotTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.redDepotTrench;
        } else if (blueDepotTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.blueDepotTrench;
        } else if (redOutpostTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.redOutpostTrench;
        } else if (blueOutpostTrenchButtonPressed) {
            copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.blueOutpostTrench;
        } else if (nuetralZoneButtonPressed1) {
            // copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.neutralZone1;
        } else if (nuetralZoneButtonPressed2) {
            // copilotShuttlePosition = Settings.FieldInfo.ShuttlingPositions.neutralZone2;
        }

        // MANUAL Controller
        // intake
        if ((Robot.intake.intakeState == IntakeStates.stationaryDeployed
                || Robot.intake.intakeState == IntakeStates.stowed)
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.intakeToggle)) {
            Robot.intake.intakeState = IntakeStates.intaking;
        } else if (Robot.intake.intakeState == IntakeStates.intaking
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.intakeToggle)) {
            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
        }

        // kicker and shooter
        if (Robot.kicker.kickerState == KickerStates.stationary
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.kickerToggle)) {
            Robot.kicker.kickerState = KickerStates.shooting;
            Robot.hopper.hopperState = HopperStates.indexing;

        } else if (Robot.kicker.kickerState == KickerStates.shooting
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.kickerToggle)) {
            Robot.kicker.kickerState = KickerStates.stationary;
            Robot.hopper.hopperState = HopperStates.stationary;

        }

        // shooter
        if (Robot.shooter.shooterState == ShooterStates.stationary
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shooterToggle)) {
            Robot.shooter.shooterState = ShooterStates.shooting;
        } else if (Robot.shooter.shooterState == ShooterStates.shooting
                && manualJoystick.getRawButtonPressed(Settings.TeleopSettings.ButtonIDs.shooterToggle)) {
            Robot.shooter.shooterState = ShooterStates.stationary;
        }

    }

    public boolean selfTestMode() {
        // run all subsystems in self test mode
        switch (testStep) {
            case 0:
                // intake
                selfTestModeFailPoint = "intake";
                Robot.intake.intakeState = IntakeStates.intaking;
                if (Robot.intake.intakeMotorLeftLeader.get() < 0.5) { // replace 0.5 with the speed Threshold
                    return false;
                }
                Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                testStep = 1;
                break;
            case 1:
                // intake deploy
                selfTestModeFailPoint = "intake deploy";
                Robot.intake.intakeState = IntakeStates.stowed;
                if (Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble() > 0.1
                        && Robot.intake.adjustedEncoderPosition() < 0.9) { // replace 0.1 and
                                                                           // 0.9 with the
                                                                           // stowed and adjusted position
                    return false;
                }
                Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                if (Robot.intake.intakeDeployMotor.getPosition().getValueAsDouble() < 0.1
                        && Math.abs(Robot.intake.adjustedEncoderPosition()) < 0.5) { // replace not adjusted position
                    return false;
                }

                testStep = 2;
                break;
            case 2:
                System.out.println("Wednesday"); // This block will execute
                break;
            default:
                System.out.println("Weekend or invalid day"); // Executes if no cases match
                break;
        }
        return true;
    }
}
