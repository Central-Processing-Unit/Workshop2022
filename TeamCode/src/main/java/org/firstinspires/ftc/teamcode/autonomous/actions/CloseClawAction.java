package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class CloseClawAction extends Action{
    public CloseClawAction(Hardware hardware, Instructions instructions, int index, int priority) {super(hardware, instructions, index, priority);}

    public CloseClawAction() {
        super();
    }

    public void execute()
    {
        hardware.clawServo.setPosition(0.75);
    }
}
