package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ChangeArmTargetAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.DriveToFreightAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OpenClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitForActionsAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;
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
    public ObjectDetector objectDetector;
    private final WaypointManager waypointManager;
    private final LinearOpMode opMode;
    public ArmPositionAction armPositionAction;

    public Instructions(Hardware hardware, Localization localization, LinearOpMode opMode, double _initialX, double _initialY, double _initialTheta, ObjectDetector objectDetector)
    {
        this.objectDetector = objectDetector;
        initialX = _initialX;
        initialY = _initialY;
        initialTheta = _initialTheta;
        objectDetector.calculateState();
        this.waypointManager = new WaypointManager();
        this.opMode = opMode;
        navigation = new Navigation(hardware, localization);
        armPositionAction = new ArmPositionAction(hardware, this);
        registerActions(hardware);
        registerNav(initialX, initialY, initialTheta);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware)
    {
        actions = new Actions();
        double targetArmPos = 0;
        switch (objectDetector.getTeamElementLocation()) {
            case LEFT:
                targetArmPos = -1200;
                break;
            case CENTER:
                targetArmPos = -3100;
                break;
            case RIGHT:
                targetArmPos = -4800;
                break;
        }

        actions.addAction(new DriveToFreightAction(hardware, this, 0,0, objectDetector));
        /*actions.addContinuousAction(armPositionAction);
        actions.addAction(new CloseClawAction(hardware, this, 0, 0));
        actions.addAction(new ChangeArmTargetAction(hardware, this, 0, 1, targetArmPos));
        if (Constants.IS_LEFT_OPMODE) {
            actions.addAction(new WaitForActionsAction(hardware, this, 0, 2, actions));
        }
        actions.addAction(new WaitForActionsAction(hardware, this, 1, 0, actions));
        actions.addAction(new OpenClawAction(hardware, this, 1, 1));
        actions.addAction(new ChangeArmTargetAction(hardware, this, 2, 0, -500));
        // Run at end to lower arm to 0 for drive
        actions.addAction(new ChangeArmTargetAction(hardware, this, 4, 0, 0));
        actions.addAction(new CloseClawAction(hardware, this, 4, 1));
        actions.addAction(new WaitForActionsAction(hardware, this, 4, 2, actions));

        if (!Constants.IS_LEFT_OPMODE) {
//            actions.addTask(new FullStopAction(3, 0));
            actions.addAction(new SpinCarouselAction(hardware, this, 3, 0));
        }*/
    }

    //Enter initial navigation waypoints here.
    private void registerNav(double initialX, double initialY, double initialTheta)
    {
        if (!Constants.IS_LEFT_OPMODE)
        {
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(973, 1282, initialTheta - Math.PI/4)));
            waypointManager.addWaypoint(new Waypoint(new Position(973, 1282), new Position(564, 1250), new Position(390, 1040), new Position(345, 464)));
            waypointManager.addWaypoint(new Waypoint(new Position(345, 345, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(911, 304, initialTheta)));
        }
        else{
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta)));
//            double targetX = 910 + (objectDetector.getTeamElementLocation() == ObjectDetector.TeamElementLocation.LEFT && Constants.IS_BLUE_TEAM ? 40 : 0);
//            double targetY = 1520 + (Constants.IS_BLUE_TEAM ? 0 : -30);
//            waypointManager.addWaypoint(new Waypoint(new Position(targetX, targetY, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(720, 1520, 0), false));
            waypointManager.addWaypoint(new Waypoint(new Position(720, 1520), new Position(10, 1790), new Position(160, 2770), new Position(230, 2850)));
            waypointManager.addWaypoint(new Waypoint(new Position(230,2850, Math.PI/2), true));
        }
    }

    public void runTasks()
    {
        int taskIndex = 0;
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

            loopToWaypoint(waypoint);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            actions.executeActions(taskIndex++);
        }
    }

    public void loopToWaypoint(Waypoint waypoint) {
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
