package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Waypoint {
    public Position targetPos;
    public Position splinePos1;
    public Position splinePos2;
    public Position startingPos;
    public boolean onlyRotate;
    public boolean isSpline;

    public Waypoint(Position targetPosition)
    {
        startingPos = new Position();
        targetPos = targetPosition;
        isSpline = false;
    }

    public Waypoint(Position targetPosition, boolean onlyRotate)
    {
        targetPos = targetPosition;
        this.onlyRotate = onlyRotate;
        isSpline = false;
    }

    public Waypoint(Position startingPosition, Position splinePosition1, Position splinePosition2, Position targetPosition)
    {
        startingPos = startingPosition;
        splinePos1 = splinePosition1;
        splinePos2 = splinePosition2;
        targetPos = targetPosition;
        isSpline = true;
    }
}
