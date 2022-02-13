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
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
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
    double t;

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

    public Navigation(Hardware hardware,
                      Localization localization,
                      ElapsedTime runtime,
                      Actions actions,
                      Telemetry telemetry,
                      LinearOpMode opMode,
                      ObjectDetector objectDetector)
    {

        telem = telemetry;
        this.runtime = runtime;
        _actions = actions;
        _hardware = hardware;
        _localization = localization;
        PIDCoefficients coefficients = new PIDCoefficients(0.01, 0.00001, 0);
        PIDCoefficients thetaCoefficients = new PIDCoefficients(0.2, 0.0003, 0);

        controller = new PID(coefficients);
        thetaController = new PID(thetaCoefficients);
		splineController = new SplineController();

        waypoints = new ArrayList<>();
        this.opMode = opMode;
    }

    public void reset() {
        waypoints.clear();
    }

    public void executeTask()
    {
        for (int i = 0; i < waypoints.size(); i++)
        {
        }

        waypoints.clear();
    }

    public void driveToTarget(Position destination) {
        driveToTarget(destination, false, false);
    }

    public void driveToTarget(Position destination, boolean isSpline, boolean onlyRotate) { driveToTarget(new Position(0,0,0), new Position(0,0,0), new Position(0,0,0), destination, isSpline, onlyRotate); }

    public boolean isTargetReached(Waypoint waypoint, boolean isErrorCorrectionMove) {
        if (waypoint.isSpline) {
            return (t > 1);
        } else {
            final Position target = isErrorCorrectionMove ? waypoint.startingPos : waypoint.targetPos;
            boolean thetaFinished = false;
            double thetaError = target.t - position.t;
            if ((thetaError) < 0 && (thetaError < -Math.PI)) {
                thetaError = target.t - position.t + (2 * Math.PI); // todo: might want to store theta error to an instance variable so we don't calculate it twice every iteration
            }

            if (Math.abs(thetaError) < THETA_TOLERANCE) {
                thetaFinished = true;
            }
            return !(((Math.abs(target.x - position.x) > 5) || (Math.abs(target.y - position.y) > 5) || !thetaFinished) && (!waypoint.onlyRotate || !thetaFinished));
        }
    }

	public void driveToTarget(Position start, Position control1, Position control2, Position destination, boolean isSpline, boolean onlyRotate)
    {
        arcLength = splineController.getArcLength(start, control1, control2, destination);
        double thetaError = destination.t - position.t;
        boolean isCounterClockwise = false;
        if ((thetaError) > 0 && (thetaError < Math.PI) ) {
            isCounterClockwise = true;
        }

        if ((thetaError) < 0 && (thetaError < -Math.PI)) {
            isCounterClockwise = true;
            thetaError = destination.t - position.t + (2 * Math.PI);
        }

        if (isSpline) {
            splineToTarget(start, control1, control2, destination);
        } else {
            moveToTarget(destination, thetaError, isCounterClockwise, onlyRotate);
        }

    }

	public void splineToTarget(Position startPos, Position control1, Position control2, Position endPos)
    {
        position = _localization.getRobotPosition(telem);
        _localization.increment(position);
		Velocity velocity = _localization.getRobotVelocity();

        double orientation, negOutput, posOutput;

        distAlongCurve += Math.sqrt(Math.pow(velocity.dx * (_localization.currentTime - _localization.previousTime), 2) + Math.pow(velocity.dy * (_localization.currentTime - _localization.previousTime), 2));

        t = splineController.getT(distAlongCurve, arcLength);
        Position velocityVector = splineController.getVelocityVector(startPos, control1, control2, endPos, t);

        if (velocityVector.x > 0)
            orientation = Math.atan(velocityVector.y / velocityVector.x) - Math.PI / 4 - position.t;
        else
            orientation = Math.atan(velocityVector.y / velocityVector.x) + Math.PI - Math.PI / 4 - position.t;

        negOutput = 0.2 * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = 0.2 * Math.cos(orientation);

        telem.addData("X", position.x);
        telem.addData("Y", position.y);
        telem.addData("T", position.t);
        AutonCore.telem.addData("t: ", t);
        AutonCore.telem.addData("VelocityVector.x: ", velocityVector.x);
        AutonCore.telem.addData("VelocityVector.y: ", velocityVector.y);
        AutonCore.telem.addData("orientation: ", orientation);
        AutonCore.telem.addData("arcLength: ", arcLength);
        AutonCore.telem.addData("distAlongCurve: ", distAlongCurve);
        AutonCore.telem.addData("negOutput: ", negOutput);
        AutonCore.telem.addData("posOutput: ", posOutput);
        AutonCore.telem.update();

        if (t < 1)
            _hardware.setMotorValues(posOutput, negOutput);
        else
            _hardware.setMotorValues(0, 0);
    }

    public void moveToTarget(Position waypointPos, double thetaError, boolean isCounterClockwise, boolean onlyRotate)
    {
        position = _localization.getRobotPosition(telem);
        _localization.increment(position);
        Velocity velocity = _localization.getRobotVelocity();

        double orientation, magnitude, negOutput, posOutput;

        if (waypointPos.x - position.x > 0)
            orientation = Math.atan(controller.getSlope(waypointPos, position)) - Math.PI / 4 - position.t;
        else if (waypointPos.x - position.x < 0)
            orientation = Math.atan(controller.getSlope(waypointPos, position)) + Math.PI - Math.PI / 4 - position.t;
        else
            orientation = Math.PI / 2;

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
        telem.addData("ARM", Constants.ARM_TARGET);
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

    public void clear() {
        _hardware.setAllMotorPowers(0);
        thetaController.resetSum();
        controller.resetSum();
        distAlongCurve = 0;
    }

}
