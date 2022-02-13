package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.List;

public class WaitForActionsAction extends Action {

    private final List<ContinuousAction> continuousActions;

    public WaitForActionsAction(int index, int priority, Actions actions) {
        super(index, priority);
        continuousActions = actions.getContinuousActions();
    }

    @Override
    public void execute(Hardware hardware, Localization localization) {
        boolean areAllFinished = false;
        while (!areAllFinished) {
            areAllFinished = true;
            for (ContinuousAction ca : continuousActions) {
                if (!ca.isFinished()) {
                    areAllFinished = false;
                    ca.execute(hardware, localization);
                }
            }
        }
    }
}
