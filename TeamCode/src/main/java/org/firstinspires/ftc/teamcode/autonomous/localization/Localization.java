package org.firstinspires.ftc.teamcode.autonomous.localization;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

public class Localization {
    private Position newPosition; //Current robot position. This is used for comparing current robot position to previously recorded robot position.
    private Position previousRobotPosition;
    private final Encoder encoder; //Contains all logic for encoder based localization.
    private Vision vision; //Contains all logic for vision based localization.
    private final ElapsedTime runtime;
    private double currentTime;
    private double previousTime;

    public Localization(Hardware hardware, Telemetry telemetry, double xOffset, double yOffset, double initialTheta)
    {
        //Robot hardware for passing to encoder and vision classes.
        if (Constants.IS_BLUE_TEAM) {
            newPosition = new Position(xOffset, yOffset, initialTheta);
        } else {
            newPosition = new Position(-xOffset, yOffset, -initialTheta + 2 * Math.PI);
        }
        previousRobotPosition = new Position();
        encoder = new Encoder(hardware, initialTheta);
        //vision = new Vision(hardware);
        runtime = new ElapsedTime();
    }

    public void increment(Position _newPosition)
    {
        previousRobotPosition = newPosition;
        newPosition = _newPosition;
        previousTime = currentTime;
    }

    public Position getRobotPosition(Telemetry telem)
    {
        Position visionRobotPosition = null; //vision.getRobotPosition();

        if (visionRobotPosition != null)
        {
            newPosition = visionRobotPosition; //This will allow encoder localization to correct to these new values.
            return visionRobotPosition;
        }

        return encoder.getRobotPosition(newPosition, telem); //If we can't see vision targets, return encoder based location.
    }

    public Velocity getRobotVelocity(ElapsedTime runtime)
    {
        currentTime = runtime.milliseconds();
        return encoder.getRobotVelocity(previousRobotPosition, newPosition, previousTime, currentTime);
    }
}