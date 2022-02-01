package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;

import java.io.IOException;

public class AutonCore {
    public static ElapsedTime runtime;
    public static Telemetry telem;

    public void runCore(double initialX, double initialY, double initialTheta, LinearOpMode opMode, Telemetry telemetry) {
        telem = telemetry;
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(opMode.hardwareMap);
        try {
            ObjectDetector.tic = new TFICBuilder(opMode.hardwareMap, "model.tflite", "NoTeamElement", "TeamElement").setQuantized(true).build();
        } catch (IOException e) {
            e.printStackTrace();
        }
        Localization localization = new Localization(hardware, opMode.telemetry, initialX, initialY, initialTheta);
        ObjectDetector objectDetector = new ObjectDetector(hardware);

        opMode.waitForStart();
        runtime.reset();

        /*do {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        } while (runtime.milliseconds() < 500);*/

        runtime.reset();


        Instructions instructions = new Instructions(hardware, localization, runtime, opMode.telemetry, opMode, initialX, initialY, initialTheta, objectDetector);

        instructions.runTasks();
        opMode.stop();
        instructions.reset();
    }
}
