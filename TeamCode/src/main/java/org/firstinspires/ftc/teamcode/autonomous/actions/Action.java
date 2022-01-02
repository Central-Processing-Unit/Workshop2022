package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class Action {

    public final int index;
    public final int priority;

    public Action(int index, int priority)
    {
        this.index = index;
        this.priority = priority;
    }

    public abstract void execute(Hardware hardware, Localization localization);

}
