package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class Action {

    private int index;
    private int priority;
    protected Instructions instructions;
    protected Hardware hardware;

    public Action() {
        index = -1;
        priority = -1;
    }

    public Action(Hardware hardware, Instructions instructions) {
        this.hardware = hardware;
        this.instructions = instructions;
        index = -1;
        priority = -1;
    }

    public Action(Hardware hardware, Instructions instructions, int index, int priority)
    {
        this.hardware = hardware;
        this.instructions = instructions;
        this.index = index;
        this.priority = priority;
    }

    public abstract void execute();

    public boolean shouldContinueExecutingActionsAtCurrentIndex() {
        return true;
    }

    public boolean isContinuous() {
        return false;
    }

    public int getIndex() {
        return index;
    }

    public void setIndex(int index) {
        this.index = index;
    }

    public int getPriority() {
        return priority;
    }

    public void setPriority(int priority) {
        this.priority = priority;
    }

    public void setInstructions(Instructions instructions) {
        this.instructions = instructions;
    }

    public void setHardware(Hardware hardware) {
        this.hardware = hardware;
    }

}
