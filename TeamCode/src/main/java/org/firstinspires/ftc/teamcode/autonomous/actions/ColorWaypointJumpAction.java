package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class ColorWaypointJumpAction extends Action {

    private int waypointIndex;
    private boolean isInClaw;

    public ColorWaypointJumpAction(int waypointIndex) {
        super();
        this.waypointIndex = waypointIndex;
    }

    public ColorWaypointJumpAction(Hardware hardware, Instructions instructions, int index, int priority, int waypointIndex) {
        super(hardware, instructions, index, priority);
        this.waypointIndex = waypointIndex;
    }

    @Override
    public void execute() {
        isInClaw = hardware.colorSensor.green() >= 110 && hardware.colorSensor.red() >= 80;
        if (!isInClaw) { // >=80 means that the block/ball/duck is in the claw
            instructions.waypointManager.setIndex(waypointIndex - 1);
        }
    }

    @Override
    public boolean shouldContinueExecutingActionsAtCurrentIndex() {
        return isInClaw;
    }
}
