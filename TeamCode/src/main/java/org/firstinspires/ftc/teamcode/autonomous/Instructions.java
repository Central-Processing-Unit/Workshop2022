package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ChangeArmTargetAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ColorWaypointJumpAction;
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
    public ObjectDetector objectDetector;
    public final WaypointManager waypointManager;
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
        registerNav(initialX, initialY, initialTheta, hardware);
    }

    //Enter initial navigation waypoints here.
    private void registerNav(double initialX, double initialY, double initialTheta, Hardware hardware)
    {
        actions = new Actions();
        WaypointBuilder waypointBuilder = new WaypointBuilder(waypointManager, actions, hardware, this);
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
        if (!Constants.IS_LEFT_OPMODE) {
            waypointBuilder
                    .runContinuously(armPositionAction)
                    .move(new Position(initialX, initialY, 0))
                        .run(new CloseClawAction()) // todo: would prefer to not have to pass hardware and this into every action constructor
                        .run(new ChangeArmTargetAction(targetArmPos))
                    .move(new Position(973, 1282, Math.PI/4))
                        .run(new WaitForActionsAction(actions))
                        .run(new OpenClawAction())
                    .move(new Position(973, 1282), new Position(564, 1250), new Position(340, 1040), new Position(345, 464, Math.PI/4))
                        .run(new ChangeArmTargetAction(-500))
                    .move(new Position(345, 345, 0))
                        .run(new ChangeArmTargetAction(0))
                        .run(new SpinCarouselAction())
                        .run(new WaitForActionsAction(actions))
                    .move(new Position(1000, 545, -Math.PI))
                        .run(new DriveToFreightAction(objectDetector, true))
                        .run(new CloseClawAction())
                        .run(new ColorWaypointJumpAction(6))
                        .run(new ChangeArmTargetAction(-4800))
                    .move(new Position(973, 1282, Math.PI/4))
                        .run(new WaitForActionsAction(actions))
                        .run(new OpenClawAction())
                    .move(new Position(911, 304, 0))
                        .run(new ChangeArmTargetAction(0))
                        .run(new CloseClawAction())
                        .run(new WaitForActionsAction(actions));
        } else{
            waypointBuilder
                    .runContinuously(armPositionAction)
                    .move(new Position(initialX, initialY, 0))
                        .run(new CloseClawAction())
                        .run(new ChangeArmTargetAction(targetArmPos))
                        .run(new WaitForActionsAction(actions))
                    .move(new Position(973, 1780, -Math.PI/4))
                        .run(new OpenClawAction())
                    .move(new Position(773, 1780, -Math.PI/2))
                        .run(new ChangeArmTargetAction(-500))
                    .move(new Position(773, 1780), new Position(-80, 1686), new Position(160, 2770), new Position(230, 2850))
                    .move(new Position(230, 2850, 0))
                        .run(new ChangeArmTargetAction(0))
                        .run(new CloseClawAction())
                        .run(new WaitForActionsAction(actions));
        }
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

            loopToWaypoint(waypoint);

            navigation.clear();

            if (opMode.isStopRequested())
                break;

            actions.executeActions(waypointManager.getIndex());
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
