package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.Constants.INITIAL_X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.actions.CloseClawAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.FullStopAction;
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
    private static ObjectDetector.TeamElementLocation elementLocation;

    public Instructions(Hardware hardware, Localization localization, ElapsedTime runtime, Telemetry telemetry, LinearOpMode opMode, double _initialX, double _initialY, double _initialTheta)
    {
        initialX = _initialX;
        initialY = _initialY;
        initialTheta = _initialTheta;
//        objectDetector = new ObjectDetector(hardware);
//        elementLocation = objectDetector.getTeamElementLocation();
//        while (elementLocation == ObjectDetector.TeamElementLocation.LOADING) {
//            elementLocation = objectDetector.getTeamElementLocation();
//        }
        registerActions(hardware, localization);
        registerNav(hardware, localization, runtime, actions, telemetry, opMode, initialX, initialY, initialTheta);
    }

    //Enter robot actions into this class.
    private void registerActions(Hardware hardware, Localization localization)
    {
        actions = new Actions(hardware, localization);
        /*if (!Constants.IS_LEFT_OPMODE) {
            if (elementLocation != ObjectDetector.TeamElementLocation.INDETERMINATE) {
                actions.addTask(new CloseClawAction(0, 0));
                switch (elementLocation) {
                    case LEFT:
                        actions.addTask(new RaiseArmAction(10, 0, 1));
                        break;
                    case CENTER:
                        actions.addTask(new RaiseArmAction(100, 0, 1));
                        break;
                    case RIGHT:
                        actions.addTask(new RaiseArmAction(200, 0, 1));
                        break;
                }
            }
            actions.addTask(new FullStopAction(1, 0));
            actions.addTask(new SpinCarouselAction(1, 1));
        }
        else{
            if (elementLocation != ObjectDetector.TeamElementLocation.INDETERMINATE) {
                actions.addTask(new CloseClawAction(0, 0));
                switch (elementLocation) {
                    case LEFT:
                        actions.addTask(new RaiseArmAction(10, 0, 1));
                        break;
                    case CENTER:
                        actions.addTask(new RaiseArmAction(100, 0, 1));
                        break;
                    case RIGHT:
                        actions.addTask(new RaiseArmAction(200, 0, 1));
                        break;
                }
            }
        }*/
        //actions.addTask(new PlaceCubeAction(3, navigation));

        //actions.addTask(new SpinCarouselAction(1));
    }

    //Enter initial navigation waypoints here.
    private void registerNav(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode, double initialX, double initialY, double initialTheta)
    {
        navigation = new Navigation(hardware, localization, runtime, actions, telemetry, opMode);
        navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta),
                new Position(initialX + 200,initialY + 400, initialTheta),
                new Position(initialX + 400,initialY - 400,initialTheta),
                new Position(initialX + 600, initialY, initialTheta)));
        if (!Constants.IS_LEFT_OPMODE) {

//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, 210, 0)));
//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, 210, 0), new Position(980, 210, initialTheta)));
        }
        else{
//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, initialY, initialTheta)));
//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, initialY, initialTheta), new Position(initialX, 2083, initialTheta)));
//            navigation.addWayPointToQueue(new Waypoint(new Position(initialX, 2803, initialTheta), new Position(initialX, 2803, initialTheta + Math.PI), true));
        }
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
