package org.firstinspires.ftc.teamcode.autonomous.actions;


import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;


public class SpeedrunAction extends Action {

    public SpeedrunAction(int index, int priority){
        super(index, priority);
    }

    public void execute(Hardware hardware, Localization localization){
        long time = System.currentTimeMillis();

        if (!Constants.IS_BLUE_TEAM) {
            hardware.setMotorValues(-1, 1);
        }
        else{
            hardware.setMotorValues(-1, 1);
        }

        while(System.currentTimeMillis() - time < 2500)
        {
        }

        hardware.setMotorValues(0,0);
    }
}
