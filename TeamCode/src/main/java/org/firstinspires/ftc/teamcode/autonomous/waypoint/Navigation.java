package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.control.SplineController;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

import java.util.ArrayList;

public class Navigation {
    private final Hardware _hardware;
    private final Localization _localization;
    private final Actions _actions;
    private final ElapsedTime runtime;
    private final PID controller;
    private final PID thetaController;
	private final SplineController splineController;
    private final Telemetry telem;
    public Position position = new Position();
    public Position prevPosition = new Position();
    private org.firstinspires.ftc.robotcore.external.navigation.Velocity velocity = new org.firstinspires.ftc.robotcore.external.navigation.Velocity();
    private Acceleration acceleration = new Acceleration();

    private double time, oldTime, deltaTime;

    /*
    Holds waypoints that we can drive to. This allows for the robot to split a move up into
    multiple linear movements. This allows the robot to avoid obstacles and for movement to be planned.

    In the future, this feature may be used to add a GUI for motion planning.
     */
    private ArrayList<Waypoint> waypoints;
    private final LinearOpMode opMode;
    private static final double THETA_TOLERANCE = 0.025;
    private double distAlongCurve;
    private double arcLength;

    public Navigation(Hardware hardware, Localization localization, ElapsedTime runtime, Actions actions, Telemetry telemetry, LinearOpMode opMode)
    {
        telem = telemetry;
        this.runtime = runtime;
        _actions = actions;
        _hardware = hardware;
        _localization = localization;
        PIDCoefficients coefficients = new PIDCoefficients(0.005, 0.00001, 0);
        PIDCoefficients thetaCoefficients = new PIDCoefficients(0.07, 0.0001, 0);

        controller = new PID(coefficients);
        thetaController = new PID(thetaCoefficients);
		splineController = new SplineController();

        waypoints = new ArrayList<>();
        this.opMode = opMode;
    }

    public void reset() {
        waypoints.clear();
    }

    public void addWayPointToQueue(Waypoint waypoint)
    {
        if (!Constants.IS_BLUE_TEAM) {
            waypoint.startingPos.x *= -1;
            waypoint.targetPos.x *= -1;
            /*waypoint.startingPos.t = -1 * waypoint.startingPos.t + 2 * Math.PI;
            waypoint.targetPos.t = -1 * waypoint.targetPos.t + 2 * Math.PI; */
        }
        waypoints.add(waypoint);
    }

    public void executeTask()
    {
        for (int i = 0; i < waypoints.size(); i++)
        {
            Waypoint waypoint = waypoints.get(i);
            if (opMode.isStopRequested())
                break;

            double time = runtime.milliseconds();
            _hardware.setAllMotorPowers(0);
            while (runtime.milliseconds() - time < 500)
            {
                //nothing
            }

            telem.addData("starting T", waypoint.startingPos.t);
            telem.addData("target T", waypoint.targetPos.t);

            // TODO force this move to happen linearly regardless of isSpline
			if (!waypoint.isSpline)
	            driveToTarget(waypoint.startingPos, waypoint.isSpline, waypoint.onlyRotate);

            controller.resetSum();

            if (opMode.isStopRequested())
                break;

            driveToTarget(waypoint.startingPos, waypoint.splinePos1, waypoint.splinePos2, waypoint.targetPos, waypoint.isSpline, waypoint.onlyRotate);
            controller.resetSum();

            if (opMode.isStopRequested())
                break;

            _actions.executeTask(i);
        }

        waypoints.clear();
    }

    public void driveToTarget(Position destination) {
        driveToTarget(destination, false, false);
    }

    public void driveToTarget(Position destination, boolean isSpline, boolean onlyRotate) { driveToTarget(new Position(0,0,0), new Position(0,0,0), new Position(0,0,0), destination, isSpline, onlyRotate); }

	public void driveToTarget(Position start, Position control1, Position control2, Position destination, boolean isSpline, boolean onlyRotate)
    {
        boolean thetaFinished = false;
        distAlongCurve = 0;
        arcLength = splineController.getArcLength(start, control1, control2, destination);

        //Assume that starting position has been reached. Drive to target specified by waypoint.
        while(((Math.abs(destination.x - position.x) > 5) || (Math.abs(destination.y - position.y) > 5) || !thetaFinished) && !opMode.isStopRequested() && (!onlyRotate || !thetaFinished)) {
            thetaFinished = false;
            double thetaError = destination.t - position.t;
            boolean isCounterClockwise = false;
            if ((thetaError) > 0 && (thetaError < Math.PI) ) {
                isCounterClockwise = true;
            }

            if ((thetaError) < 0 && (thetaError < -Math.PI)) {
                isCounterClockwise = true;
                thetaError = destination.t - position.t + (2 * Math.PI);
            }

            if (Math.abs(thetaError) < THETA_TOLERANCE){
                thetaFinished = true;
            }

			if (isSpline)
            {
                splineToTarget(start, control1, control2, destination);
            } else
	            moveToTarget(destination, thetaError, isCounterClockwise, onlyRotate);
        }
    }

	public void splineToTarget(Position startPos, Position control1, Position control2, Position endPos)
    {
        if (opMode.isStopRequested())
            return;

        position = _localization.getRobotPosition(telem);
        _localization.increment(position);

        double orientation, negOutput, posOutput, t;

        distAlongCurve += _localization.getDeltaPosition();

        t = splineController.getT(distAlongCurve, arcLength);
        Position velocityVector = splineController.getVelocityVector(startPos, control1, control2, endPos, t);

        if (velocityVector.x > 0)
            orientation = Math.atan(velocityVector.y / velocityVector.x) - Math.PI / 4;
        else
            orientation = Math.atan(velocityVector.y / velocityVector.x) + Math.PI - Math.PI / 4;

        negOutput = 0.3 * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = 0.3 * Math.cos(orientation);

//        AutonCore.telem.addData("t: ", t);
//        AutonCore.telem.addData("VelocityVector.x: ", velocityVector.x);
//        AutonCore.telem.addData("VelocityVector.y: ", velocityVector.y);
//        AutonCore.telem.addData("orientation: ", orientation);
//        AutonCore.telem.addData("arcLength: ", arcLength);
//        AutonCore.telem.addData("distAlongCurve: ", distAlongCurve);
//        AutonCore.telem.addData("negOutput: ", negOutput);
//        AutonCore.telem.addData("posOutput: ", posOutput);
//        AutonCore.telem.update();

        if (t < 1)
            _hardware.setMotorValues(posOutput, negOutput);
        else
            _hardware.setMotorValues(0, 0);
    }

    public void moveToTarget(Position waypointPos, double thetaError, boolean isCounterClockwise, boolean onlyRotate)
    {
        if (opMode.isStopRequested())
            return;

        position = _localization.getRobotPosition(telem);
        _localization.increment(position);
        Velocity velocity = _localization.getRobotVelocity(runtime);

        double orientation, magnitude, negOutput, posOutput;

        if (waypointPos.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(waypointPos, position)) - Math.PI / 4 - position.t;
        else
            orientation = Math.atan(controller.getSlope(waypointPos, position)) + Math.PI - Math.PI / 4 - position.t;

        double error = Math.sqrt(Math.pow(waypointPos.y - position.y, 2) + Math.pow(waypointPos.x - position.x, 2));
        double speed = Math.sqrt(Math.pow(velocity.dy, 2) + Math.pow(velocity.dx, 2));

        magnitude = controller.getOutput(error, speed);
        if (error < 3) { // Make magnitude 0 if error is too low to matter
            magnitude = 0;
        }

        negOutput = magnitude * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = magnitude * Math.cos(orientation);
//
        telem.addData("X", position.x);
        telem.addData("Y", position.y);
        telem.addData("T", position.t);
        telem.addData("Orientation", orientation);
        telem.addData("target T", waypointPos.t);
        telem.addData("Theta Error", thetaError);
//        telem.addData("Raw Theta", _hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
//        telem.addData("Init Theta", Constants.INIT_THETA);
//        telem.addData("Velocity", Math.sqrt(Math.pow(velocity.dx, 2) + Math.pow(velocity.dy, 2)));
        telem.update();


        double thetaOutput = thetaController.getOutput(Math.abs(thetaError), 0);
        if (Math.abs(thetaError) < THETA_TOLERANCE) { // Set thetaOutput to 0 if thetaError is negligible
            thetaOutput = 0;
        }
        if (onlyRotate) {
            _hardware.setMotorValuesWithRotation(0, 0, (isCounterClockwise ? -1 : 1) * thetaOutput);
        } else {
            _hardware.setMotorValuesWithRotation(0.1 * posOutput, 0.1 * negOutput, (isCounterClockwise ? -1 : 1) * thetaOutput);
        }
    }
}
