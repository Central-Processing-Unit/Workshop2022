package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class Action {

    public final int index;
    public final int priority;
    protected Instructions instructions;
    protected Hardware hardware;

    public Action(Hardware hardware, Instructions instructions) {
        this.hardware = hardware;
        this.instructions = instructions;
        index = -1;
        priority = -1;
        if (!isContinuous()) {
            AutonCore.telem.addLine("Error: action added without index or priority.");
            AutonCore.telem.update();
        }
    }

    public Action(Hardware hardware, Instructions instructions, int index, int priority)
    {
        this.hardware = hardware;
        this.instructions = instructions;
        this.index = index;
        this.priority = priority;
    }

    public abstract void execute();

    public boolean isContinuous() {
        return false;
    }

}
