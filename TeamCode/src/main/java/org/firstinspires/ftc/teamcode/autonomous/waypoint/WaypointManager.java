package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.Constants;

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

    public void addWaypoint(Waypoint waypoint) {
        if (!Constants.IS_BLUE_TEAM) {
            waypoint.startingPos.x *= -1;
            waypoint.targetPos.x *= -1;
            if (waypoint.isSpline) {
                waypoint.splinePos1.x *= -1;
                waypoint.splinePos2.x *= -1;
            }
            waypoint.targetPos.t -= Math.PI;
        }
        waypoints.add(waypoint);
    }

}
