package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RightBlueOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.IS_BLUE_TEAM = true;
        new AutonCore().runCore(Constants.INITIAL_X, Constants.RIGHT_INITIAL_Y, Constants.BLUE_INITIAL_THETA, this);
    }

}
