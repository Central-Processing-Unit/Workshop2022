package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RightBlueOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_BLUE_TEAM = true;
		Constants.IS_LEFT_OPMODE = false;
        new AutonCore().runCore(Constants.INITIAL_X, Constants.BLUE_RIGHT_INITIAL_Y, Constants.BLUE_INITIAL_THETA, this, telemetry);
    }

}
