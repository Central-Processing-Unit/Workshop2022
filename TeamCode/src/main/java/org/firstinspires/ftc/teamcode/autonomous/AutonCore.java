package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class AutonCore {
    public static ElapsedTime runtime;
    public static Telemetry telem;
    public static Instructions instructions;

    public void runCore(double initialX, double initialY, double initialTheta, LinearOpMode opMode, Telemetry telemetry) {
        telem = telemetry;
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(opMode.hardwareMap);
        Localization localization = new Localization(hardware, initialX, initialY, initialTheta);

        opMode.waitForStart();
        runtime.reset();
        long t = System.currentTimeMillis();
        while (System.currentTimeMillis() - t < 6000) {}

        /*do {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        } while (runtime.milliseconds() < 500);*/

        runtime.reset();


        instructions = new Instructions(hardware, localization, opMode, initialX, initialY, initialTheta);

        instructions.runTasks();
        opMode.stop();
        instructions.reset();
    }
}
