package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.List;

public class WaitForActionsAction extends Action {

    private final List<ContinuousAction> continuousActions;

    public WaitForActionsAction(Actions actions) {
        super();
        continuousActions = actions.getContinuousActions();
    }

    public WaitForActionsAction(Hardware hardware, Instructions instructions, int index, int priority, Actions actions) {
        super(hardware, instructions, index, priority);
        continuousActions = actions.getContinuousActions();
    }

    @Override
    public void execute() {
        boolean areAllFinished = false;
        while (!areAllFinished) {
            areAllFinished = true;
            for (ContinuousAction ca : continuousActions) {
                if (!ca.isFinished()) {
                    areAllFinished = false;
                }
                ca.execute();
            }
        }
    }
}
