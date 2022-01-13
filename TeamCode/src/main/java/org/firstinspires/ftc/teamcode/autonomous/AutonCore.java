package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.actions.Actions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.localization.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Navigation;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

public class AutonCore {
    // IS SENSITIVE to the order in which actions are added
    public static ElapsedTime runtime;

    public void runCore(double initialX, double initialY, double initialTheta, LinearOpMode opMode) {
        runtime = new ElapsedTime();
        Hardware hardware = new Hardware(opMode.hardwareMap);
        Localization localization = new Localization(hardware, opMode.telemetry, initialX, initialY, initialTheta);
        Instructions instructions = new Instructions(hardware, localization, runtime, opMode.telemetry, opMode, initialX, initialY, initialTheta);

        opMode.waitForStart();

        runtime.reset();

        /*do {
            Constants.INIT_THETA = hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).secondAngle;
        } while (runtime.milliseconds() < 500);*/

        runtime.reset();

        instructions.runTasks();
        opMode.stop();
        instructions.reset();
    }
}
