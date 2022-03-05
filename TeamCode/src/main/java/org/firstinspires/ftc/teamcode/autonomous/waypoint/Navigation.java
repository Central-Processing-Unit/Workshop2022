package org.firstinspires.ftc.teamcode.autonomous.waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.control.SplineController;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

import static org.firstinspires.ftc.teamcode.autonomous.AutonCore.telem;

import java.util.ArrayList;

public class Navigation {
    private final Hardware _hardware;
    public final Localization _localization;
    private final PID controller;
    private final PID thetaController;
	private final SplineController splineController;
    public Position position = new Position();
    double t;

    /*
    Holds waypoints that we can drive to. This allows for the robot to split a move up into
    multiple linear movements. This allows the robot to avoid obstacles and for movement to be planned.

    In the future, this feature may be used to add a GUI for motion planning.
     */
    private final ArrayList<Waypoint> waypoints;
    private static final double THETA_TOLERANCE = 0.025;
    private double distAlongCurve;
    private double arcLength;

    public Navigation(Hardware hardware,
                      Localization localization)
    {

        _hardware = hardware;
        _localization = localization;
        PIDCoefficients coefficients = new PIDCoefficients(/*0.019*/ 0.01, 0.00003, 0);
        PIDCoefficients thetaCoefficients = new PIDCoefficients(0.25, 0.0004, 0);
        controller = new PID(coefficients);
        thetaController = new PID(thetaCoefficients);
		splineController = new SplineController();

        waypoints = new ArrayList<>();
    }

    public void reset() {
        waypoints.clear();
    }

    public void driveToTarget(Position destination, boolean isForDuck) {
        driveToTarget(destination, false, false, isForDuck);
    }

    public void driveToTarget(Position destination, boolean isSpline, boolean onlyRotate, boolean isForDuck) { driveToTarget(new Position(0,0,0), new Position(0,0,0), new Position(0,0,0), destination, isSpline, onlyRotate, isForDuck); }

    public boolean isTargetReached(Waypoint waypoint) {
        boolean thetaFinished = false;
        double thetaError = waypoint.targetPos.t - position.t;
        if (Math.abs(waypoint.targetPos.t - (position.t - 2 * Math.PI)) < Math.abs(thetaError)) {
            thetaError = waypoint.targetPos.t - (position.t - 2 * Math.PI);
        }
        if (Math.abs(waypoint.targetPos.t - (position.t + 2 * Math.PI)) < Math.abs(thetaError)) {
            thetaError = waypoint.targetPos.t - (position.t + 2 * Math.PI);
        }
//        if ((thetaError) < 0 && (thetaError < -Math.PI)) {
//            thetaError = waypoint.targetPos.t - position.t + (2 * Math.PI); // todo: might want to store theta error to an instance variable so we don't calculate it twice every iteration
//        }

        if (Math.abs(thetaError) < THETA_TOLERANCE) {
            thetaFinished = true;
        }
        return !(((Math.abs(waypoint.targetPos.x - position.x) > 10) || (Math.abs(waypoint.targetPos.y - position.y) > 10) || !thetaFinished) && (!waypoint.onlyRotate || !thetaFinished)) && (!waypoint.isSpline || t > 1);
    }

	public void driveToTarget(Position start, Position control1, Position control2, Position destination, boolean isSpline, boolean onlyRotate, boolean isForDuck)
    {
        arcLength = splineController.getArcLength(start, control1, control2, destination);
        double thetaError = destination.t - position.t; // -6
        boolean isCounterClockwise = false;
        if (Math.abs(destination.t - (position.t - 2 * Math.PI)) < Math.abs(thetaError)) {
            thetaError = destination.t - (position.t - 2 * Math.PI);
            isCounterClockwise = true;
        }
        if (Math.abs(destination.t - (position.t + 2 * Math.PI)) < Math.abs(thetaError)) {
            thetaError = destination.t - (position.t + 2 * Math.PI);
            isCounterClockwise = true;
        }
        if ((thetaError) > 0 && (thetaError < Math.PI) ) {
            isCounterClockwise = true;
        }

        if ((thetaError) < 0 && (thetaError < -Math.PI)) {
            isCounterClockwise = true;
            thetaError = destination.t - position.t + (2 * Math.PI);
        }

        if (isSpline && t < 1) {
            splineToTarget(start, control1, control2, destination);
        } else {
            moveToTarget(destination, thetaError, isCounterClockwise, onlyRotate, isForDuck);
        }

    }

	public void splineToTarget(Position startPos, Position control1, Position control2, Position endPos)
    {
        position = _localization.getRobotPosition();
        _localization.increment(position);
		Velocity velocity = _localization.getRobotVelocity();

        double orientation, negOutput, posOutput, magnitude;

        distAlongCurve += Math.sqrt(Math.pow(velocity.dx * (_localization.currentTime - _localization.previousTime), 2) + Math.pow(velocity.dy * (_localization.currentTime - _localization.previousTime), 2));

        t = splineController.getT(distAlongCurve, arcLength);
        Position velocityVector = splineController.getVelocityVector(startPos, control1, control2, endPos, t);

        if (velocityVector.x > 0)
            orientation = Math.atan(velocityVector.y / velocityVector.x) - Math.PI / 4 - position.t;
        else
            orientation = Math.atan(velocityVector.y / velocityVector.x) + Math.PI - Math.PI / 4 - position.t;

        negOutput = 0.5 * Math.sin(orientation);
        if (orientation == 0)
            posOutput = negOutput;
        else
            posOutput = 0.5 * Math.cos(orientation);

//        telem.addData("X", position.x);
//        telem.addData("Y", position.y);
//        telem.addData("T", position.t);
//        telem.addData("t: ", t);
//        telem.addData("VelocityVector.x: ", velocityVector.x);
//        telem.addData("VelocityVector.y: ", velocityVector.y);
//        telem.addData("orientation: ", orientation);
//        telem.addData("arcLength: ", arcLength);
//        telem.addData("distAlongCurve: ", distAlongCurve);
//        telem.addData("negOutput: ", negOutput);
//        telem.addData("posOutput: ", posOutput);
//        telem.update();

        if (t < 1)
            _hardware.setMotorValues(posOutput, negOutput);
        else
            _hardware.setMotorValues(0, 0);
    }

    public void moveToTarget(Position waypointPos, double thetaError, boolean isCounterClockwise, boolean onlyRotate, boolean isForDuck)
    {
        position = _localization.getRobotPosition();
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
        if (isForDuck && error < 35) {
            _hardware.clawServo.setPosition(0.75);
        }
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

        telem.addData("X", position.x);
        telem.addData("Y", position.y);
        telem.addData("T", position.t);
        telem.addData("Orientation", orientation);
        telem.addData("target T", waypointPos.t);
        telem.addData("Theta Error", thetaError);
        telem.addData("ARM", _hardware.armMotor.getCurrentPosition());
        telem.addData("Raw Theta", _hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telem.addData("Init Theta", Constants.INIT_THETA);
        telem.addData("Velocity", Math.sqrt(Math.pow(velocity.dx, 2) + Math.pow(velocity.dy, 2)));
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
