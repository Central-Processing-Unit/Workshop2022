package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class WaitAction extends Action{
    public WaitAction(int index, int priority) {super(index, priority);}
    public void execute(Hardware hardware, Localization localization)
    {
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 5000)
        {

        }
    }
}
