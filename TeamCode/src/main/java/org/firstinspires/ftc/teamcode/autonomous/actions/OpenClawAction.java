package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class OpenClawAction extends Action{
    public OpenClawAction(Hardware hardware, Instructions instructions, int index, int priority) {super(hardware, instructions, index, priority);}

    public void execute()
    {
        hardware.clawServo.setPosition(0.35);
    }
}
