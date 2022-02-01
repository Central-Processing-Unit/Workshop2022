package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class RaiseArmAction extends Action{
    private double targetArmPos;
    private PID armPID = new PID(new PIDCoefficients(-0.05, 0, 0));
    private double prevArmPos, prevTime, lastPIDOutput;
    public RaiseArmAction(double _targetArmPos, int index, int priority)
    {
        super(index, priority);
        targetArmPos = _targetArmPos;
    }

    @Override
    public void execute(Hardware hardware, Localization localization)
    {
        double armPos = hardware.armMotor.getCurrentPosition();
        double armPosError = armPos - targetArmPos;
        while (Math.abs(armPosError) > 5)
        {
            armPos = hardware.armMotor.getCurrentPosition();
            armPosError = armPos - targetArmPos;
            double dArmPosError = (prevArmPos - targetArmPos) / (System.currentTimeMillis() - prevTime);
            double output = armPID.getOutput(armPosError, dArmPosError);
            lastPIDOutput = output;
            hardware.armMotor.setPower(output);
            prevArmPos = armPos;
            prevTime = System.currentTimeMillis();
        }
    }
}
