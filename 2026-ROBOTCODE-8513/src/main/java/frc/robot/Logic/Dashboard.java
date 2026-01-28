package frc.robot.Logic;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {
    public Field2d trajField2d = new Field2d();

    public void updateDashboard() {

        SmartDashboard.putData("trajGoalPose", trajField2d);

    }
}
