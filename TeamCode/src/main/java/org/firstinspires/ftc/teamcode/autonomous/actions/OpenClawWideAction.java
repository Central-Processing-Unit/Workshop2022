package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class OpenClawWideAction extends Action {
    public OpenClawWideAction() {
        super();
    }

    public OpenClawWideAction(Hardware hardware, Instructions instructions, int index, int priority) {
        super(hardware, instructions, index, priority);
    }

    public void execute()
    {
        hardware.clawServo.setPosition(0.25);
    }
}
