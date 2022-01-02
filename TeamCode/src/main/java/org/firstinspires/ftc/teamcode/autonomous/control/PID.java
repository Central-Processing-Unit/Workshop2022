package org.firstinspires.ftc.teamcode.autonomous.control;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;

public class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double errorSum;

    public PID(PIDCoefficients coeffs)
    {
        kP = coeffs.p;
        kI = coeffs.i;
        kD = coeffs.d;
    }

    public double getOutput(double error, double dError) {
        errorSum += error;
        return (kP * error) + (kI * errorSum) - (kD * dError);
    }

    public double getSlope(Position target, Position position)
    {
        return (target.y - position.y) / (target.x - position.x);
    }

    public void resetSum() {
        errorSum = 0;
    }
}
