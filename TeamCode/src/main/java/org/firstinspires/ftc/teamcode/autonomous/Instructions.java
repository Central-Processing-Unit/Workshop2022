package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ChangeArmTargetAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
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
        armPositionAction = new ArmPositionAction();
        registerActions(hardware, localization);
        registerNav(initialX, initialY, initialTheta);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware, Localization localization)
    {
        actions = new Actions(hardware, localization);
        double targetArmPos = 0;
        switch (objectDetector.getTeamElementLocation()) {
            case LEFT:
                targetArmPos = -4800;
                break;
            case CENTER:
                targetArmPos = -3100;
                break;
            case RIGHT:
                targetArmPos = -1200;
                break;
        }
        actions.addContinuousAction(armPositionAction);
        actions.addAction(new CloseClawAction(0, 0));
        actions.addAction(new ChangeArmTargetAction(0, 1, targetArmPos));
        actions.addAction(new WaitForActionsAction(1, 0, actions));
        actions.addAction(new OpenClawAction(1, 1));
        actions.addAction(new ChangeArmTargetAction(2, 0, -500));
        // Run at end to lower arm to 0 for drive
        actions.addAction(new ChangeArmTargetAction(4, 0, 0));
        actions.addAction(new CloseClawAction(4, 1));
        actions.addAction(new WaitForActionsAction(4, 2, actions));

        if (!Constants.IS_LEFT_OPMODE) {
//            actions.addTask(new FullStopAction(3, 0));
            actions.addAction(new SpinCarouselAction(3, 0));
        }
    }

    //Enter initial navigation waypoints here.
    private void registerNav(double initialX, double initialY, double initialTheta)
    {
        if (!Constants.IS_LEFT_OPMODE)
        {
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(910, 1520, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(910, 1520, initialTheta), new Position(610, 1500, initialTheta), new Position(450, 1330, initialTheta), new Position(530, 850, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(420, 760, initialTheta), new Position(490, 75, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(500, 75, initialTheta), new Position(1000, 125, initialTheta)));
        }
        else{
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(910, 1520, initialTheta)));
            waypointManager.addWaypoint(new Waypoint(new Position(910, 1520, initialTheta), new Position(720, 1520, initialTheta-Math.PI/2), false));
            waypointManager.addWaypoint(new Waypoint(new Position(720, 1520, initialTheta-Math.PI/2), new Position(10, 1790, initialTheta-Math.PI/2), new Position(160, 2770, initialTheta-Math.PI/2), new Position(230, 2850, initialTheta-Math.PI/2)));
            waypointManager.addWaypoint(new Waypoint(new Position(230,2850,initialTheta-Math.PI/2), new Position(230,2850, initialTheta), true));
        }
//          navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(429, 2283, initialTheta), new Position(629, 2483, initialTheta), new Position(829, 2683, initialTheta)));
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

            AutonCore.telem.addData("starting T", waypoint.startingPos.t);
            AutonCore.telem.addData("target T", waypoint.targetPos.t);

            // Ignore whether the waypoint is a spline for error correction
            loopToWaypoint(waypoint, true);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            loopToWaypoint(waypoint, false);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            actions.executeActions(taskIndex++);
        }
    }

    private void loopToWaypoint(Waypoint waypoint, boolean isErrorCorrectionMove) {
        while (!navigation.isTargetReached(waypoint, isErrorCorrectionMove) && !opMode.isStopRequested()) {
            actions.executeContinuousActions();
            if (isErrorCorrectionMove) {
                navigation.driveToTarget(waypoint.startingPos, false,  waypoint.onlyRotate);
            } else if (waypoint.isSpline) {
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
