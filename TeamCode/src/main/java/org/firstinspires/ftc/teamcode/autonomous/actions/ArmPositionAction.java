package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class ArmPositionAction extends ContinuousAction {

    private double targetArmPos = 0;
    private double prevArmPos;
    private PID controller = new PID(new PIDCoefficients(0.005, 0, 0));
    private long prevTime;

    public ArmPositionAction() {
        super();
    }

    @Override
    public void execute(Hardware hardware, Localization localization) {
        double targetArmPos = Constants.ARM_TARGET;
        double armPos = hardware.armMotor.getCurrentPosition();
        double armPosError = targetArmPos - armPos;
        double dArmPosError = (armPos - prevArmPos) / (System.currentTimeMillis() - prevTime);
        double output = controller.getOutput(armPosError, dArmPosError);
        hardware.armMotor.setPower(output);
        prevArmPos = armPos;
        prevTime = System.currentTimeMillis();
        AutonCore.telem.addLine("armOutput: " + output);
        AutonCore.telem.addLine("armError: " + armPosError);
        AutonCore.telem.addLine("armPos: " + armPos);
    }

    @Override
    public void initialize(Hardware hardware, Localization localization) {
        prevArmPos = hardware.armMotor.getCurrentPosition();
        prevTime = System.currentTimeMillis();
    }
}
