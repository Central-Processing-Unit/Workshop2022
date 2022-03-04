package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class ArmPositionAction extends ContinuousAction {

    public static double targetArmPos = 0;
    private double prevArmPos;
    private final PID controller = new PID(new PIDCoefficients(0.005, 0, 0));
    private long prevTime;

    public ArmPositionAction(Hardware hardware, Instructions instructions) {
        super(hardware, instructions);
    }

    @Override
    public void execute() {
        double armPos = hardware.armMotor.getCurrentPosition();
        double armPosError = targetArmPos - armPos;
        double dArmPosError = (armPos - prevArmPos) / (System.currentTimeMillis() - prevTime);
        double output = controller.getOutput(armPosError, dArmPosError);
        hardware.armMotor.setPower(output);
        prevArmPos = armPos;
        prevTime = System.currentTimeMillis();
//        AutonCore.telem.addLine("armOutput: " + output);
//        AutonCore.telem.addLine("armError: " + armPosError);
//        AutonCore.telem.addLine("armPos: " + armPos);
//        AutonCore.telem.update();
    }

    @Override
    public void initialize() {
        prevArmPos = hardware.armMotor.getCurrentPosition();
        prevTime = System.currentTimeMillis();
        targetArmPos = 0;
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetArmPos - prevArmPos) < 20;
    }
}
