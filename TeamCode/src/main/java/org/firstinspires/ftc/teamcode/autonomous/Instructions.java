package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.WaypointBuilder;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.WaypointManager;

/*
The Instructions class contains a map of waypoints and actions. It outlines all of the
tasks that the robot carries out during the autonomous phase. This class provides a method of
isolating these instructions to a separate class, rather than clogging up the AutonCore class.

This class also provides a method of disposing resources.
 */
public class Instructions {
    public static double initialX, initialY, initialTheta;

    public Navigation navigation;
    public Actions actions;
    public final WaypointManager waypointManager;
    private final LinearOpMode opMode;

    public Instructions(Hardware hardware, Localization localization, LinearOpMode opMode, double _initialX, double _initialY, double _initialTheta)
    {
        initialX = _initialX;
        initialY = _initialY;
        initialTheta = _initialTheta;
        this.waypointManager = new WaypointManager();
        this.opMode = opMode;
        navigation = new Navigation(hardware, localization);
        registerNav(initialX, initialY, initialTheta, hardware);
    }

    //Enter initial navigation waypoints here.
    private void registerNav(double initialX, double initialY, double initialTheta, Hardware hardware)
    {
        actions = new Actions();
        WaypointBuilder waypointBuilder = new WaypointBuilder(waypointManager, actions, hardware, this);

        waypointBuilder.move(new Position(initialX, initialY, 0));
    }

    public void runTasks()
    {
        actions.initialize();
        while (waypointManager.nextWaypoint()) {
            Waypoint waypoint = waypointManager.getCurrentWaypoint();
            if (opMode.isStopRequested()) {
                break;
            }

            double time = System.currentTimeMillis();
            navigation.clear();
            while (System.currentTimeMillis() - time < 500)
            {
                //nothing
            }

            loopToWaypoint(waypoint, false);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            actions.executeActions(waypointManager.getIndex());
        }
    }

    public void loopToWaypoint(Waypoint waypoint, boolean isForDuck) {
        while (!navigation.isTargetReached(waypoint) && !opMode.isStopRequested()) {
            actions.executeContinuousActions();
            if (waypoint.isSpline) {
                navigation.driveToTarget(waypoint.startingPos, waypoint.splinePos1, waypoint.splinePos2, waypoint.targetPos, true, waypoint.onlyRotate);
            } else {
                navigation.driveToTarget(waypoint.targetPos, false, waypoint.onlyRotate);
            }
        }
    }


    public void reset() {
        navigation.reset();
        actions.reset();
    }
}
