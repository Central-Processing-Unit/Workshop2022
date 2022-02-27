package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;

import java.util.ArrayList;
import java.util.List;

public class WaypointManager {

    private final List<Waypoint> waypoints = new ArrayList<>();
    private int waypointIndex = -1;

    /**
     * @return The current waypoint, if present, or null, if not present
     */
    public Waypoint getCurrentWaypoint() {
        if (waypointIndex >= waypoints.size()) {
            return null;
        }
        return waypoints.get(waypointIndex);
    }

    public boolean nextWaypoint() {
        waypointIndex++;
        return waypointIndex < waypoints.size();
    }

    public void setIndex(int i) {
        waypointIndex = i;
    }

    public int getIndex() {
        return waypointIndex;
    }

    public void addWaypoint(Waypoint waypoint) {
        if (!Constants.IS_BLUE_TEAM) {
            waypoint.targetPos.x *= -1;
            if (waypoint.isSpline) {
                waypoint.startingPos.x *= -1;
                waypoint.splinePos1.x *= -1;
                waypoint.splinePos2.x *= -1;
            }
        }
        waypoint.targetPos.t = Instructions.initialTheta + (Constants.IS_BLUE_TEAM ? 1 : -1) * waypoint.targetPos.t;

        waypoints.add(waypoint);
    }

}
