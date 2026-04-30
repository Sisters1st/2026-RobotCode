package frc.robot.Logic;

public class Enums {
    public enum IntakeStates {
        intaking, outtaking, stowed, stationaryDeployed, shooting, maxPositionWhenShooting
    }
    public enum ClimberStates {
        stowed, deployed, climbed
    }
    public enum HopperStates {
        stationary, indexing, unjam
    }
    public enum KickerStates {
        stationary, intaking, shooting
    }
    public enum ShooterStates {
        stationary, shooting
    }
    public enum DrivebaseStates {
        locked, drive 
    }

    public enum AutoRoutines {

        MiddlePreLoaded, Outpost_OneCycle_Close, Outpost_OneCycle_Mid, Outpost_OneCycle_Far, Outpost_FullAcross_OneCycle_Close,
        Outpost_FullAcross_OneCycle_Mid, Outpost_FullAcross_OneCycle_Far, 
        Outpost_OneCycle_CloseNORETURN, Outpost_OneCycle_MidNORETURN,Outpost_OneCycle_FarNORETURN,
        Outpost_OneCycle_SweepHub,
        Outpost_OneCycle_Inward_Close, Outpost_OneCycle_Inward_Mid, Outpost_OneCycle_Inward_Far, Outpost_Cycle1_Fun, Outpost_Cycle2_Fun, Outpost,
        Depot_OneCycle_Close, Depot_OneCycle_Mid, Depot_OneCycle_Far, Depot_FullAcross_OneCycle_Close,
        Depot_FullAcross_OneCycle_Mid, Depot_FullAcross_OneCycle_Far, 
        Depot_OneCycle_CloseNORETURN, Depot_OneCycle_MidNORETURN, Depot_OneCycle_FarNORETURN, 
        Depot_OneCycle_SweepHub, 
        Depot_OneCycle_Inward_Close, Depot_OneCycle_Inward_Mid, Depot_OneCycle_Inward_Far, Depot, Depot_Cycle1_Fun, Depot_Cycle2_Fun,
        DoNothing, OliviaAttemptGoOverBump, Outpost_AnyTwoParts, Depot_AnyTwoParts,  
    }

    public enum TCPChooser {
        autoDetectWinnerOfAuto, blueWonAuto, redWonAuto
    }

    public enum AllianceSelector {
        redAlliance, blueAlliance
    }

}
