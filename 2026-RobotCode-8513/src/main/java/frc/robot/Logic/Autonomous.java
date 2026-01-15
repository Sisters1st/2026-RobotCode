package frc.robot.Logic;

import frc.robot.Robot;

public class Autonomous {
    
    public Autonomous(){

    }

    public void autoInit(){
        Robot.drivebase.initPathFromFile("ExamplePath");
    }

    public void autoLoop(){
        Robot.drivebase.followPath();
    }
}
