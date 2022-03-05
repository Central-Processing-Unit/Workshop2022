package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class SpinCarouselAction extends Action {

    public SpinCarouselAction()
    {
        super();
    }

    public SpinCarouselAction(Hardware hardware, Instructions instructions, int index, int priority)
    {
        super(hardware, instructions, index, priority);
    }

    @Override
    public void execute() {

        long time = System.currentTimeMillis();
        hardware.carouselMotor.setPower(Constants.IS_BLUE_TEAM ? 0.2 : -0.2);
        while (System.currentTimeMillis() - time < 1500)
        {
        }
        hardware.carouselMotor.setPower(Constants.IS_BLUE_TEAM ? 0.5 : -0.5);
        while (System.currentTimeMillis() - time < 2000) {
        }
        hardware.carouselMotor.setPower(0);
//        Position p = new Position(575, 444, 0); // todo: find x ad y
//        instructions.navigation._localization.increment(p);
//        instructions.navigation._localization.increment(p);
    }
}
