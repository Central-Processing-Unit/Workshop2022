package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class ContinuousAction extends Action {

    public ContinuousAction() {

    }

    public abstract void initialize(Hardware hardware, Localization localization);

    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean isContinuous() {
        return true;
    }

}
