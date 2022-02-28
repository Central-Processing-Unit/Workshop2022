package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.actions.Action;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.ContinuousAction;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class WaypointBuilder {

    private final WaypointManager waypointManager;
    private final Actions actions;
    private final Hardware hardware;
    private final Instructions instructions;

    private int index = -1;
    private int priority = 0;

    public WaypointBuilder(WaypointManager waypointManager, Actions actions, Hardware hardware, Instructions instructions) {
        this.waypointManager = waypointManager;
        this.actions = actions;
        this.hardware = hardware;
        this.instructions = instructions;
    }

    public WaypointBuilder move(Position p) {
        waypointManager.addWaypoint(new Waypoint(p));
        index++;
        priority = 0;
        return this;
    }

    public WaypointBuilder move(Position startPos, Position splinePos1, Position splinePos2, Position targetPos) {
        waypointManager.addWaypoint(new Waypoint(startPos, splinePos1, splinePos2, targetPos));
        index++;
        priority = 0;
        return this;
    }

    public WaypointBuilder move(Waypoint w) {
        waypointManager.addWaypoint(w);
        index++;
        priority = 0;
        return this;
    }

    public WaypointBuilder run(Action a) {
        a.setIndex(index);
        a.setPriority(priority++);
        a.setHardware(hardware);
        a.setInstructions(instructions);
        actions.addAction(a);
        return this;
    }

    public WaypointBuilder runContinuously(ContinuousAction a) {
        actions.addContinuousAction(a);
        return this;
    }

}
