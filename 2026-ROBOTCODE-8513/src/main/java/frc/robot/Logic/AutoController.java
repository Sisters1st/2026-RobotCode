package frc.robot.Logic;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Settings;
import frc.robot.Logic.Enums.AutoRoutines;
import frc.robot.Logic.Enums.HopperStates;
import frc.robot.Logic.Enums.IntakeStates;
import frc.robot.Logic.Enums.KickerStates;
import frc.robot.Logic.Enums.ShooterStates;

public class AutoController {

    public SendableChooser<String> autoSelectorPart1;
    public SendableChooser<String> autoSelectorPart2;

    public AutoRoutines autoRoutine = AutoRoutines.DoNothing;
    public AutoRoutines dashboardAutoRoutine1 = AutoRoutines.DoNothing;
    public AutoRoutines dashboardAutoRoutine2 = AutoRoutines.DoNothing;

    public AutoRoutines mainReturnAuto = AutoRoutines.Depot_AnyTwoParts;

    public boolean pathInitialized = false;
    public double shotTimeOut = 1.5;
    public double alignTimeOut = 0.25;

    public AutoRoutines autoToReturnTo = AutoRoutines.DoNothing;

    double outpostDeployIntakeTime = 0.5;

    public int autoStep;
    double timeStepStarted = 0;
    int bumpTholdCounter = 0;

    public AutoController() {
        // create auto selector with each enum option
        autoSelectorPart1 = new SendableChooser<>();
        autoSelectorPart1.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if (AutoRoutines.values()[i].toString().charAt(0) != '~') {
                autoSelectorPart1.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }
        }

        autoSelectorPart2 = new SendableChooser<>();
        autoSelectorPart2.setDefaultOption(AutoRoutines.values()[0].toString(), AutoRoutines.values()[0].toString());
        for (int i = 1; i < AutoRoutines.values().length; i++) {
            if (AutoRoutines.values()[i].toString().charAt(0) != '~') {
                autoSelectorPart2.addOption(AutoRoutines.values()[i].toString(), AutoRoutines.values()[i].toString());
            }

        }
        SmartDashboard.putData("Auton Selector Part 1", autoSelectorPart1);
        SmartDashboard.putData("Auton Selector Part 2", autoSelectorPart2);
    }

    public void initAuto() {
        Elastic.selectTab("Autonomous");
        updateAutoRoutineFromDashboard();
        autoStep = 0;
        timeStepStarted = Timer.getFPGATimestamp();
        Robot.intake.intakeDeployController.reset(Robot.intake.adjustedEncoderPosition());
        Robot.drivebase.rotationPidController.reset(0);

    }

    public void autoPeriodic() {

        switch (autoRoutine) {
            case DoNothing:
                Robot.drivebase.yagslDrive.lockPose();
                break;
            case Depot_OneCycle_Close:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Close", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_Mid:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Mid", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_Far:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Far", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_FullAcross_OneCycle_Far:
                switch (autoStep) {
                    case 0:

                        autoToReturnTo = AutoRoutines.Depot_FullAcross_OneCycle_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Far", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();

                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_FullAcross_OneCycle_Close:
                switch (autoStep) {
                    case 0:

                        autoToReturnTo = AutoRoutines.Depot_FullAcross_OneCycle_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Close", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();

                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_FullAcross_OneCycle_Mid:
                switch (autoStep) {
                    case 0:

                        autoToReturnTo = AutoRoutines.Depot_FullAcross_OneCycle_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Mid", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();

                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_CloseNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_CloseNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_CloseNORETURN", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_MidNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_MidNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_MidNORETURN", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_FarNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_FarNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_FarNORETURN", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_SweepHub:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_SweepHub;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_SweepHub", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.15) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Inward_Close:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Inward_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Close", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Inward_Mid:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Inward_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Mid", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Inward_Far:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Inward_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Far", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_Inward_Close:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Inward_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Close", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_Inward_Mid:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Inward_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Mid", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_OneCycle_Inward_Far:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_OneCycle_Inward_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Inward_Far", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Close:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Close", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Mid:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Mid", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_Far:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_Far", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.46) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_FullAcross_OneCycle_Close:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_FullAcross_OneCycle_Close;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Close", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_FullAcross_OneCycle_Mid:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_FullAcross_OneCycle_Mid;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Mid", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_FullAcross_OneCycle_Far:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_FullAcross_OneCycle_Far;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_fullAcross_OneCycle_Far", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.42) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > shotTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();
                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_CloseNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_CloseNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_CloseNORETURN", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_MidNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_MidNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_MidNORETURN", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_FarNORETURN:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_FarNORETURN;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_FarNORETURN", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_OneCycle_SweepHub:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_OneCycle_SweepHub;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_OneCycle_SweepHub", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.15) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot:
                switch (autoStep) {
                    case 0:
                        // if (Robot.onRed) {
                        // ifNoCameraAssumeRobotPos(new Pose2d(12.824, 2.489, new Rotation2d(180)));
                        // } else {
                        // ifNoCameraAssumeRobotPos(new Pose2d(3.677, 5.58, new Rotation2d(0)));
                        // }

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Depot", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 1.5) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 5) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.shooting;
                        break;
                }
                break;
            case MiddlePreLoaded:
                switch (autoStep) {
                    case 0:

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("Middle", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 5;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        if (Robot.shooter.readyToShootInHub() || Robot.isSimulation()) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.stowed;
                        break;
                }
                break;

            case Outpost:
                switch (autoStep) {
                    case 0:
                        // if (Robot.onRed) {
                        // ifNoCameraAssumeRobotPos(new Pose2d(12.837, 5.636, new Rotation2d(180)));
                        // } else {
                        // ifNoCameraAssumeRobotPos(new Pose2d(3.677, 2.644, new Rotation2d(0)));
                        // }

                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Outpost_Pt1", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 10;
                        break;

                    case 10:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 12;
                        }
                        break;
                    case 12:
                        Robot.drivebase.yagslDrive.lockPose();
                        if (Timer.getFPGATimestamp() - timeStepStarted > 2) {
                            autoStep = 15;
                        }
                        break;

                    case 15:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stowed;

                        Robot.drivebase.initPath("Outpost_Outpost_Pt2", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 20;
                        break;
                    case 20:
                        if (Robot.drivebase.followLoadedPath()) {
                            autoStep = 25;
                        }
                        break;
                    case 25:
                        Robot.shooter.shooterState = ShooterStates.shooting;
                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        Robot.drivebase.yagslDrive.lockPose();
                        break;
                }
                break;
            case OliviaAttemptGoOverBump:
                if (Robot.isReal()) {
                    switch (autoStep) {
                        case 0:
                            Robot.shooter.shooterState = ShooterStates.stationary;
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                            // path is over
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 10;
                        case 10:
                            if (Robot.onRed) {
                                Robot.drivebase.driveFacingHeading(new Translation2d(-2, 0), new Rotation2d(Math.PI),
                                        true);
                            } else {
                                Robot.drivebase.driveFacingHeading(new Translation2d(2, 0), new Rotation2d(0), true);
                            }
                            if (Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees() > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 1) {
                                bumpTholdCounter++;
                            } else {
                                bumpTholdCounter = 0;
                            }
                            if (bumpTholdCounter >= Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                                bumpTholdCounter = 0;
                                autoStep = 15;
                            }
                            break;
                        case 15:
                            if (Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees() < -Settings.AutoSettings.Thresholds.autoDetectedBumpPitchTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 2) {
                                bumpTholdCounter++;
                            } else {
                                bumpTholdCounter = 0;
                            }
                            if (Robot.onRed) {
                                Robot.drivebase.driveFacingHeading(new Translation2d(-2, 0), new Rotation2d(Math.PI),
                                        true);
                            } else {
                                Robot.drivebase.driveFacingHeading(new Translation2d(2, 0), new Rotation2d(0), true);
                            }

                            if (bumpTholdCounter > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                                bumpTholdCounter = 0;
                                autoStep = 20;
                            }
                            break;
                        case 20:
                            if (Math.abs(Robot.drivebase.yagslDrive.getPitch()
                                    .getDegrees()) < Settings.AutoSettings.Thresholds.detectedFlatTHold
                                    || Timer.getFPGATimestamp() - timeStepStarted >= 2) {
                                bumpTholdCounter++;
                            } else {
                                bumpTholdCounter = 0;
                            }

                            if (Robot.onRed) {
                                Robot.drivebase.driveFacingHeading(new Translation2d(-2, 0), new Rotation2d(Math.PI),
                                        true);
                            } else {
                                Robot.drivebase.driveFacingHeading(new Translation2d(2, 0), new Rotation2d(0), true);
                            }

                            if (bumpTholdCounter > Settings.AutoSettings.Thresholds.autoDetectedBumpPitchCount) {
                                bumpTholdCounter = 0;
                                autoStep = 5;
                                autoRoutine = autoToReturnTo;
                                timeStepStarted = Timer.getFPGATimestamp();
                            }
                    }
                } else {
                    autoRoutine = autoToReturnTo;
                    autoStep = 5;
                }
                break;

            case Depot_Cycle1_Fun:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_Cycle1_Fun;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Cycle1_Fun", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.3) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_Cycle1_Fun:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_Cycle1_Fun;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Cycle1_Fun", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.3) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Outpost_Cycle2_Fun:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_Cycle2_Fun;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Cycle2_Fun", false);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;
            case Depot_Cycle2_Fun:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_Cycle2_Fun;
                        autoRoutine = AutoRoutines.OliviaAttemptGoOverBump;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        Robot.drivebase.initPath("Outpost_Cycle2_Fun", true);

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 15;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= 0.2) {
                            Robot.intake.intakeState = IntakeStates.intaking;
                        }

                        if (Timer.getFPGATimestamp() - timeStepStarted >= Robot.drivebase.traj.getTotalTimeSeconds()
                                - 1) {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                            Robot.shooter.shooterState = ShooterStates.shooting;
                        }

                        break;
                    case 15:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            autoStep = 20;
                            Robot.teleop.timeIntakeShootingButtonPressed = Timer.getFPGATimestamp();
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        break;
                    case 20:
                        Robot.drivebase.faceHub();
                        if (Robot.shooter.readyToShootInHub()
                                || Timer.getFPGATimestamp() - timeStepStarted > alignTimeOut) {
                            Robot.hopper.hopperState = HopperStates.indexing;
                            Robot.kicker.kickerState = KickerStates.shooting;
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.hopper.hopperState = HopperStates.stationary;
                            Robot.kicker.kickerState = KickerStates.stationary;
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }
                        if (Timer.getFPGATimestamp() - timeStepStarted < 0.4) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                            timeStepStarted = Timer.getFPGATimestamp();

                        }
                        autoRoutine = AutoRoutines.Depot_AnyTwoParts;
                        autoStep = 5;
                        break;
                }
                break;

            case Outpost_AnyTwoParts:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Outpost_AnyTwoParts;
                        autoRoutine = dashboardAutoRoutine1;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        Robot.drivebase.faceHub();

                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;

                        // old intake shooting timing
                        if (Timer.getFPGATimestamp() - timeStepStarted >= getShootingTime()
                                - Settings.AutoSettings.autoIntakeBringInDelay) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if ((Timer.getFPGATimestamp() - timeStepStarted >= getShootingTime())) {
                            Robot.drivebase.initPath("Outpost_OneCycle_Reset", false);
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        autoToReturnTo = AutoRoutines.Outpost_AnyTwoParts;
                        autoRoutine = dashboardAutoRoutine2;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;
                }
                break;
            // make sure all the names for paths allign
            case Depot_AnyTwoParts:
                switch (autoStep) {
                    case 0:
                        autoToReturnTo = AutoRoutines.Depot_AnyTwoParts;
                        autoRoutine = dashboardAutoRoutine1;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;

                    case 5:
                        Robot.drivebase.faceHub();

                        Robot.hopper.hopperState = HopperStates.indexing;
                        Robot.kicker.kickerState = KickerStates.shooting;

                        // old intake shooting timing
                        if (Timer.getFPGATimestamp() - timeStepStarted >= getShootingTime()
                                - Settings.AutoSettings.autoIntakeBringInDelay) {
                            Robot.intake.intakeState = IntakeStates.shooting;
                        } else {
                            Robot.intake.intakeState = IntakeStates.stationaryDeployed;
                        }

                        if ((Timer.getFPGATimestamp() - timeStepStarted >= getShootingTime())) {
                            Robot.drivebase.initPath("Outpost_OneCycle_Reset", true);
                            autoStep = 30;
                        }
                        break;

                    case 30:
                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.intaking;

                        if (Robot.drivebase.followLoadedPath()) {
                            timeStepStarted = Timer.getFPGATimestamp();
                            autoStep = 35;
                        }
                        break;
                    case 35:
                        autoToReturnTo = AutoRoutines.Depot_AnyTwoParts;
                        autoRoutine = dashboardAutoRoutine2;

                        Robot.shooter.shooterState = ShooterStates.stationary;
                        Robot.hopper.hopperState = HopperStates.stationary;
                        Robot.kicker.kickerState = KickerStates.stationary;
                        Robot.intake.intakeState = IntakeStates.stationaryDeployed;

                        // path is over
                        timeStepStarted = Timer.getFPGATimestamp();
                        autoStep = 0;
                        break;
                }
                break;
        }

        if (DriverStation.isAutonomousEnabled()) {
            Robot.intake.setMotorPower();
            Robot.hopper.setMotorPower();
            Robot.kicker.setMotorPower();
            Robot.shooter.setMotorPower();
        }
    }

    public void updateAutoRoutineFromDashboard() {
        try {
            dashboardAutoRoutine1 = AutoRoutines.valueOf(autoSelectorPart1.getSelected());
            dashboardAutoRoutine2 = AutoRoutines.valueOf(autoSelectorPart2.getSelected());

            if (dashboardAutoRoutine2.toString().startsWith("Outpost")) {
                autoRoutine = AutoRoutines.Outpost_AnyTwoParts;
            } else if (dashboardAutoRoutine2.toString().startsWith("Depot")) {
                autoRoutine = AutoRoutines.Depot_AnyTwoParts;
            }
            
            if (dashboardAutoRoutine1.toString().startsWith("Mid")) {
                autoRoutine = AutoRoutines.MiddlePreLoaded;
            }

        } catch (Exception e) {
            autoRoutine = AutoRoutines.DoNothing;
        }
    }

    // asumes the robot pose if there has been no apriltags
    public void ifNoCameraAssumeRobotPos(Pose2d autoStartPose) {
        if (Timer.getFPGATimestamp() - Robot.vision.timeATLastSeen > 20) {
            Robot.drivebase.yagslDrive.resetOdometry(autoStartPose);
        }
    }

    public double getShootingTime() {
        if (dashboardAutoRoutine1.name().toString().contains("Full")
                || dashboardAutoRoutine1.name().toString().contains("Far")) {
            return 4.5;
        } else {
            return 4;
        }
    }

}
