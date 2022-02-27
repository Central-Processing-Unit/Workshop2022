package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.control.PID;

@TeleOp
public class Drive extends Core {
    double positive_power, negative_power, rot_power;
    double joystick_x, joystick_y, joystick_power;
    double orientation;
    int carouselDirection = 0;
    Orientation gyro_angles;
    long prevTime = System.currentTimeMillis();
    boolean isClawClosed;
    long bLastPressed = -1;
    long yLastPressed = -1;
    boolean turboMode;
    double clawPos;
    double armPower;
    PID armController = new PID(new PIDCoefficients(0.003, 0, 0));
    double armTarget;

    public void loop() {

        if (gamepad1.a) {
            spinCarousel(1);
        } else if (gamepad1.x) {
            spinCarousel(-1);
        }

        if (gamepad1.right_trigger > 0.5 && armMotor.getCurrentPosition() > -4800) {
            armPower = -1;
            armTarget = armMotor.getCurrentPosition();
        } else if (gamepad1.left_trigger > 0.5 && armMotor.getCurrentPosition() < 0){
            armPower = 1;
            armTarget = armMotor.getCurrentPosition();
        } else {
            if (armTarget < -4800) {
                armTarget = -4800;
            } else if (armTarget > 0) {
                armTarget = 0;
            }
            armPower = armController.getOutput(armTarget - armMotor.getCurrentPosition(), 0);
        }

        if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;
        }

        clawPos = isClawClosed ? 0.75 : 0.35;

//        if (gamepad1.y && System.currentTimeMillis() - yLastPressed > 250) {
//            yLastPressed = System.currentTimeMillis();
//            turboMode = !turboMode;
//        }

        turboMode = gamepad1.y;

        prevTime = System.currentTimeMillis();
        // Get all the info we from the gamepad
        joystick_y = gamepad1.left_stick_y > 0 ? Math.pow(gamepad1.left_stick_y, 2) :
                    -Math.pow(gamepad1.left_stick_y, 2);
        joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                (gamepad1.left_stick_x > 0 ? Math.pow(gamepad1.left_stick_x, 2) :
                -Math.pow(gamepad1.left_stick_x, 2));
        rot_power = 0.4 * (gamepad1.right_stick_x);

        // Find out the distance of the joystick from resting position to control speed
        joystick_power = Math.sqrt(Math.pow(joystick_x, 2) + Math.pow(joystick_y, 2));

        // Pull raw orientation values from the gyro
        gyro_angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double theta = gyro_angles.firstAngle; // Add pi for CPU's robot

        // Turn the joystick coordinates into an angle in radians
        orientation = (joystick_x > 0) ? (Math.atan(-joystick_y / joystick_x) - Math.PI / 4)  - theta :
                (Math.atan(-joystick_y/joystick_x) + Math.PI - Math.PI / 4) - theta ;

        telemetry.addData("theta", theta);
        telemetry.addData("orientation", orientation);
        telemetry.update();

        // Pass that angle through a pair of wave functions to get the power for each corresponding pair of parallel wheels
        negative_power = (joystick_power * Math.sin(orientation));
        positive_power = (orientation != 0) ? (joystick_power * Math.cos(orientation)) :
                negative_power;

        if (!turboMode)
        {
            negative_power *= 0.5;
            positive_power *= 0.5;
            rot_power *= 0.6;
        }

        // This is all we need to actually move the robot, method decs in Core.java
        move(positive_power, negative_power, rot_power);
        moveCarousel(carouselDirection * 0.18);
        setClawPos(clawPos);
        moveArm(armPower);
    }

    private void spinCarousel(int direction)
    {
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 2000)
        {
            moveCarousel(direction * (System.currentTimeMillis() - startTime < 1500 ? 0.18 : 0.5));
        }
        moveCarousel(0);
    }
}
