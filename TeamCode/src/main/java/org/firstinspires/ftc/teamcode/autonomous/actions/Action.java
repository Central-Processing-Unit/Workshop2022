package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class Action {

    public final int index;
    public final int priority;

    public Action() {
        index = -1;
        priority = -1;
        if (!isContinuous()) {
            AutonCore.telem.addLine("Error: action added without index or priority.");
            AutonCore.telem.update();
        }
    }

    public Action(int index, int priority)
    {
        this.index = index;
        this.priority = priority;
    }

    public abstract void execute(Hardware hardware, Localization localization);

    public boolean isContinuous() {
        return false;
    }

}
