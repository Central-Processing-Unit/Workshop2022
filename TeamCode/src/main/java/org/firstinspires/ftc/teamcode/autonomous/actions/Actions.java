package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Actions {
    private final Map<Integer, List<Action>> actions = new HashMap<>();
    private final Hardware hardware;
    private final Localization localization;

    public Actions(Hardware hardware, Localization localization)
    {
        this.hardware = hardware;
        this.localization = localization;
    }

    // IS SENSITIVE to the order in which actions are added
    public void addTask(Action action)
    {
        if (actions.containsKey(action.index)) {
            actions.get(action.index).add(action);
        } else {
            List<Action> actionList = new ArrayList<>();
            actionList.add(action);
            actions.put(action.index, actionList);
        }
    }

    // Executes the task at the waypoint with a given index
    public void executeTask(int index)
    {
        List<Action> actionList = actions.get(index);
        if (actionList == null) {
            return;
        }
        for (Action action : actionList) {
            action.execute(hardware, localization);
        }
    }

    public void reset() {
        actions.clear();
    }

}
