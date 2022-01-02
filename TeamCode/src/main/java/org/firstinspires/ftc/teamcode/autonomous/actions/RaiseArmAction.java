package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class RaiseArmAction extends Action{
    public RaiseArmAction(int index, int priority) {super(index, priority);}

    @Override
    public void execute(Hardware hardware, Localization localization)
    {
        hardware.armMotor.setPower(-1);
    }
}
