package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class ChangeArmTargetAction extends Action{
    double pos;

    public ChangeArmTargetAction(double pos) {
        super();
        this.pos = pos;
    }

    public ChangeArmTargetAction(Hardware hardware, Instructions instructions, int index, int priority, double pos) {
        super(hardware, instructions, index, priority);
        this.pos = pos;
    }

    public void execute()
    {
        ArmPositionAction.targetArmPos = pos;
    }
}