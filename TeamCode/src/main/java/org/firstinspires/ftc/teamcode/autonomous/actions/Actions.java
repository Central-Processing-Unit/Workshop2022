package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class Actions {
    private final Map<Integer, Map<Integer, Action>> actions = new HashMap<>();
    private final List<ContinuousAction> continuousActions = new LinkedList<>();

    public Actions()
    {
    }

    public void addAction(Action action)
    {
        if (actions.containsKey(action.getIndex())) {
            actions.get(action.getIndex()).put(action.getPriority(), action);
        } else {
            Map<Integer, Action> actionMap = new HashMap<>();
            actionMap.put(action.getPriority(), action);
            actions.put(action.getIndex(), actionMap);
        }
    }

    public void addContinuousAction(ContinuousAction continuousAction) {
        continuousActions.add(continuousAction);
    }

    public void executeContinuousActions() {
        for (ContinuousAction a : continuousActions) {
            a.execute();
        }
    }

    // Should be run right before autonomous movement starts
    public void initialize() {
        for (ContinuousAction a : continuousActions) {
            a.initialize();
        }
    }

    // Executes the task at the waypoint with a given index
    public void executeActions(int index)
    {
        Map<Integer, Action> actionMap = actions.get(index);
        if (actionMap == null) {
            return;
        }
        for (int i = 0; i < actionMap.size(); i++) {
            Action action = actionMap.get(i);
            if (action == null) {
                throw new RuntimeException("Invalid action priority: expected one priority for each value from 0 to " + (actionMap.size() - 1) + ", but couldn't find an action with priority " + i);
            }
            action.execute();
            if (!action.shouldContinueExecutingActionsAtCurrentIndex()) {
                break;
            }
        }
    }

    public void reset() {
        actions.clear();
    }

    public List<ContinuousAction> getContinuousActions() {
        return continuousActions;
    }

}
