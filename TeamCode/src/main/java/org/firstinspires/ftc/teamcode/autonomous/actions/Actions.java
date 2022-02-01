package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.HashMap;
import java.util.Map;

public class Actions {
    private final Map<Integer, Map<Integer, Action>> actions = new HashMap<>();
    private final Hardware hardware;
    private final Localization localization;

    public Actions(Hardware hardware, Localization localization)
    {
        this.hardware = hardware;
        this.localization = localization;
    }

    public void addTask(Action action)
    {
        if (actions.containsKey(action.index)) {
            actions.get(action.index).put(action.priority, action);
        } else {
            Map<Integer, Action> actionMap = new HashMap<>();
            actionMap.put(action.priority, action);
            actions.put(action.index, actionMap);
        }
    }

    // Executes the task at the waypoint with a given index
    public void executeTask(int index)
    {
        Map<Integer, Action> actionMap = actions.get(index);
        if (actionMap == null) {
            return;
        }
        for (int i = 0; i < actionMap.size(); i++) {
            Action action = actionMap.get(i);
            if (action == null) {
                throw new RuntimeException("Invalid action priority: expected one priority for each value from 0 to " + (actionMap.size() - 1) + ", but instead found " + i);
            }
            action.execute(hardware, localization);
        }
    }

    public void reset() {
        actions.clear();
    }

}
