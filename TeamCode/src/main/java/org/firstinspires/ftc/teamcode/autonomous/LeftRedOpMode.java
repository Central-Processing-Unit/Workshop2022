package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LeftRedOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_LEFT_OPMODE = true;
		Constants.IS_BLUE_TEAM = false;
        new AutonCore().runCore(Constants.INITIAL_X, Constants.RED_LEFT_INITIAL_Y + 200, Constants.RED_INITIAL_THETA,this, telemetry);
    }

}
