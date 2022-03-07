package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class CloseClawSyncAction extends Action {
    public CloseClawSyncAction(Hardware hardware, Instructions instructions, int index, int priority) {super(hardware, instructions, index, priority);}

    public CloseClawSyncAction() {
        super();
    }

    public void execute()
    {
        hardware.clawServo.setPosition(0.75);
        long t = System.currentTimeMillis();
        while (System.currentTimeMillis() - t < 1500) {
        }
    }
}
