package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class FullStopAction extends Action{
    public FullStopAction(int index, int priority) { super(index, priority); }

    public void execute(Hardware hardware, Localization localization)
    {
        hardware.setAllMotorPowers(0);
    }
}
