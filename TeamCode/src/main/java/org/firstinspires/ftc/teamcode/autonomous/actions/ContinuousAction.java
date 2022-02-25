package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class ContinuousAction extends Action {

    public ContinuousAction(Hardware hardware, Instructions instructions) {
        super(hardware, instructions);
    }

    public abstract void initialize();

    public boolean isFinished() {
        return true;
    }
    @Override
    public boolean isContinuous() {
        return true;
    }

}
