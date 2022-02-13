package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class Encoder {
    private final Hardware hardware; //Contains robot hardware for measuring robot position using motor encoders.



    //Position of encoder on each motor respectively. We need to store this so that we can subtract these values from current position to get displacement.
    private int lastLfPos, lastRfPos, lastRbPos, lastLbPos;
    private double initialTheta;

    public Encoder(Hardware hardware, double initialTheta)
    {
        this.hardware = hardware;
        this.initialTheta = initialTheta;
    }

    public Position getRobotPosition(Position previousPosition)
    {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.
        int lfPos, rfPos, rbPos, lbPos;

        //Record encoder values.
        lfPos = hardware.leftFrontMotor.getCurrentPosition();
        rfPos = hardware.rightFrontMotor.getCurrentPosition();
        rbPos = hardware.rightBackMotor.getCurrentPosition();
        lbPos = hardware.leftBackMotor.getCurrentPosition();

        //Displacement values
        double lfDisp, rfDisp, rbDisp, lbDisp;

        //Calculate displacement values
        lfDisp = (lfPos - lastLfPos) * Constants.DISTANCE_PER_TICK;
        rfDisp = (rfPos - lastRfPos) * Constants.DISTANCE_PER_TICK;
        rbDisp = (rbPos - lastRbPos) * Constants.DISTANCE_PER_TICK;
        lbDisp = (lbPos - lastLbPos) * Constants.DISTANCE_PER_TICK;

        //Store encoder values so we can use them in calculating displacement.
        lastLfPos = lfPos;
        lastRfPos = rfPos;
        lastRbPos = rbPos;
        lastLbPos = lbPos;

        //Average of displacement values
        double dispAvg;

        //Calculate avg.
        dispAvg = (lfDisp + rfDisp + rbDisp + lbDisp) / 4;

        //Robot displacement
        double robotLfDisp, robotRfDisp, robotRbDisp, robotLbDisp;

        //Calculate robot displacement
        robotLfDisp = lfDisp - dispAvg;
        robotRfDisp = rfDisp - dispAvg;
        robotRbDisp = rbDisp - dispAvg;
        robotLbDisp = lbDisp - dispAvg;

        //Holonomic displacement in robot reference frame.
        double deltaX, deltaY;

        //Compute displacement in robot reference include a picture :)frame.
        deltaX = (robotLfDisp + robotRfDisp - robotRbDisp - robotLbDisp) / (2 * Math.sqrt(2));
        deltaY = (robotLfDisp - robotRfDisp - robotRbDisp + robotLbDisp) / (2 * Math.sqrt(2));

        //Robot theta
        double theta;

        //Compute robot theta
        theta = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialTheta;
        if (theta > 2 * Math.PI) {
            theta -= 2 * Math.PI;
        } else if (theta < 0) {
            theta += 2 * Math.PI;
        }
        AutonCore.telem.addData("T_e", theta);

        //Compute displacement in field reference frame.
        double deltaXf = deltaX * Math.cos(theta) - deltaY * Math.sin(theta);
        double deltaYf = deltaY * Math.cos(theta) + deltaX * Math.sin(theta);

        Position robotPosition = new Position();
        robotPosition.x = previousPosition.x - deltaXf;
        robotPosition.y = previousPosition.y - deltaYf;
        robotPosition.t = theta;

        if (robotPosition.x == 0) robotPosition.x = 0.0000001;

        return robotPosition;
    }

    public Position getRawPosition()
    {
        //Encoder values. These are in ticks. We will later convert this to a usable distance.
        int lfPos, rfPos, rbPos, lbPos;

        //Record encoder values.
        lfPos = hardware.leftFrontMotor.getCurrentPosition();
        rfPos = hardware.rightFrontMotor.getCurrentPosition();
        rbPos = hardware.rightBackMotor.getCurrentPosition();
        lbPos = hardware.leftBackMotor.getCurrentPosition();

        return new Position((lfPos + rbPos)/2d, (rfPos + lbPos)/2d, 0);
    }

    public double getDistanceTraveled(Position prevRawPosition, Position rawPosition){
        double posDelta = (rawPosition.x - prevRawPosition.x) * Constants.DISTANCE_PER_TICK;
        double negDelta = (rawPosition.y - prevRawPosition.y) * Constants.DISTANCE_PER_TICK;

        return Math.sqrt(Math.pow(posDelta, 2) + Math.pow(negDelta, 2));
    }

    public Velocity getRobotVelocity(Position previousPosition, Position currentPosition, double previousTime, double currentTime)
    {
        Velocity robotVelocity = new Velocity();

        robotVelocity.dx = (currentPosition.x - previousPosition.x) / (currentTime - previousTime);
        robotVelocity.dy = (currentPosition.y - previousPosition.y) / (currentTime - previousTime);
        robotVelocity.dt = (currentPosition.t - previousPosition.t) / (currentTime - previousTime);

        return robotVelocity;
    }
}