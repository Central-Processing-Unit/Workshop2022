package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class SpinCarouselAction extends Action {

    public SpinCarouselAction(int index, int priority)
    {
        super(index, priority);
    }

    @Override
    public void execute(Hardware hardware, Localization localization)
    {
        long time = System.currentTimeMillis();
        hardware.carouselMotor.setPower(-0.4);
        while(System.currentTimeMillis() - time < 2500)
        {
        }
        hardware.carouselMotor.setPower(0);
    }
}
