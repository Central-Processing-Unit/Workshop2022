package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class ColorWaypointJumpAction extends Action {

    private int waypointIndex;

    public ColorWaypointJumpAction(Hardware hardware, Instructions instructions, int index, int priority, int waypointIndex) {
        super(hardware, instructions, index, priority);
        this.waypointIndex = waypointIndex;
    }

    @Override
    public void execute() {
        if (hardware.colorSensor.green() < 80) { // >=80 means that the block/ball/duck is in the claw
            instructions.waypointManager.setIndex(waypointIndex);
        }
    }
}
