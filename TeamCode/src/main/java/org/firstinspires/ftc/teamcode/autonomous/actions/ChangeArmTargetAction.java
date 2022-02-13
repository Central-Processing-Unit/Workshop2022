package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class ChangeArmTargetAction extends Action{
    double _pos;
    public ChangeArmTargetAction(int index, int priority, double pos) {
        super(index, priority);
        _pos = pos;
    }

    public void execute(Hardware hardware, Localization localization)
    {
        Constants.ARM_TARGET = _pos;
    }
}