package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class ChangeArmTargetAction extends Action{
    double pos;
    public ChangeArmTargetAction(int index, int priority, double pos) {
        super(index, priority);
        this.pos = pos;
    }

    public void execute(Hardware hardware, Localization localization)
    {
        ArmPositionAction.targetArmPos = pos;
    }
}