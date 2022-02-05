package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.Constants.INITIAL_X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Action;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.OpenClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.RaiseArmAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SpinCarouselAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

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

    public Instructions(Hardware hardware, Localization localization, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode, double _initialX, double _initialY, double _initialTheta, ObjectDetector objectDetector)
    {
        this.objectDetector = objectDetector;
        initialX = _initialX;
        initialY = _initialY;
        initialTheta = _initialTheta;
        objectDetector.calculateState();
        registerActions(hardware, localization);
        registerNav(hardware, localization, runtime, actions, telemetry, opMode, initialX, initialY, initialTheta);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware, Localization localization)
    {
        actions = new Actions(hardware, localization);
        if (objectDetector.getTeamElementLocation() != ObjectDetector.TeamElementLocation.INDETERMINATE) {
            actions.addTask(new CloseClawAction(0, 0));
//            switch (objectDetector.getTeamElementLocation()) {
//                case LEFT:
//                    actions.addTask(new RaiseArmAction(10, 0, 1));
//                    break;
//                case CENTER:
//                    actions.addTask(new RaiseArmAction(40, 0, 1));
//                    break;
//                case RIGHT:
//                    actions.addTask(new RaiseArmAction(200, 0, 1));
//                    break;
//            }
            actions.addTask(new OpenClawAction(1, 0));

//            actions.addTask(new Action(3, 0) {
//                @Override
//                public void execute(Hardware hardware, Localization localization) {
//                    navigation.targetArmPos = 0;
//                }
//            });
        }
        if (!Constants.IS_LEFT_OPMODE) {
//            actions.addTask(new FullStopAction(3, 0));
            actions.addTask(new SpinCarouselAction(2, 0));
        }
    }

    //Enter initial navigation waypoints here.
    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode, double initialX, double initialY, double initialTheta)
    {
        navigation = new Navigation(hardware, localization, runtime, actions, telemetry, opMode, objectDetector);
        if (!Constants.IS_LEFT_OPMODE)
        {
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX+590, initialY+590, initialTheta)));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX+590, initialY+590, initialTheta), new Position(initialX, initialY-1000, initialTheta)));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY-800, initialTheta), new Position(initialX+680, initialY-900, initialTheta)));
        }
        else{
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX+590, initialY-630, initialTheta)));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX+590, initialY-630, initialTheta), new Position(initialX+590, initialY-630, initialTheta-Math.PI/2), true));
            navigation.addWayPointToQueue(new Waypoint(new Position(initialX+590, initialY-630, initialTheta-Math.PI/2), new Position(-310, 1190, initialTheta-Math.PI/2), new Position(-60, 3050, initialTheta-Math.PI/2), new Position(250, 2880, initialTheta-Math.PI/2)));
            navigation.addWayPointToQueue(new Waypoint(new Position(0,0,initialTheta-Math.PI/2), new Position(0,0,initialTheta), true));
        }

//          navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(429, 2283, initialTheta), new Position(629, 2483, initialTheta), new Position(829, 2683, initialTheta)));
    }

    public void runTasks()
    {
        navigation.executeTask();
    }


    public void reset() {
        navigation.reset();
        actions.reset();
    }
}
