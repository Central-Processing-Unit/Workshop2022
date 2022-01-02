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
    double prevArmPos;
    double targetArmPos;
    boolean hasTargetArmPos;
    boolean isArmMoving;
    long prevTime = System.currentTimeMillis();
    private PID armPID = new PID(new PIDCoefficients(-0.05, 0.0, 0));
    double lastPIDOutput = 0;
    boolean isClawClosed;
    long clawOpeningTime = 0;
    long bLastPressed = -1;
    long yLastPressed = -1;
    boolean turboMode;

    public void loop() {
        double armPos = armMotor.getCurrentPosition();
        boolean usePID = Math.abs(armPos) > 5;
        if (gamepad1.right_trigger > 0) {
            armMotor.setPower(-0.2 * gamepad1.right_trigger);
            isArmMoving = true;
            hasTargetArmPos = false;
            prevArmPos = armPos;
        } else if (gamepad1.left_trigger > 0) {
            armMotor.setPower((usePID ? lastPIDOutput : 0) + -gamepad1.left_trigger);
            isArmMoving = true;
            hasTargetArmPos = false;
            prevArmPos = armPos;
        } else if (isArmMoving) {
            isArmMoving = false;
            hasTargetArmPos = true;
            targetArmPos = armPos;
        }
        if (hasTargetArmPos && usePID) {
            double armPosError = armPos - targetArmPos;
            double dArmPosError = (prevArmPos - targetArmPos) / (System.currentTimeMillis() - prevTime);
            double output = armPID.getOutput(armPosError, dArmPosError);
            lastPIDOutput = output;
            armMotor.setPower(output);
            prevArmPos = armPos;
        }

        if (gamepad1.a)
        {
            carouselDirection = 1;
        } else if (gamepad1.x)
        {
            carouselDirection = -1;
        } else
        {
            carouselDirection = 0;
        }

        moveCarousel(carouselDirection * 0.3);

        if (gamepad1.b == true && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;
            clawOpeningTime = 0;
            if (isClawClosed) {
                closeClaw();
            }
        }

        if (gamepad1.y == true && System.currentTimeMillis() - yLastPressed > 250) {
            yLastPressed = System.currentTimeMillis();
            turboMode = !turboMode;
        }

        if (!isClawClosed && clawOpeningTime < 250) {
            clawOpeningTime += System.currentTimeMillis() - prevTime;
            clawMotor.setPower(-0.2);
        } else if (!isClawClosed) {
            clawMotor.setPower(0);
        }


        prevTime = System.currentTimeMillis();
        // Get all the info we from the gamepad
        joystick_y = gamepad1.left_stick_y;
        joystick_x = (gamepad1.left_stick_x == 0) ? 0.000001 :
                gamepad1.left_stick_x;
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
            negative_power *= 0.45;
            positive_power *= 0.45;
            rot_power *= 0.4;
        }

        // This is all we need to actually move the robot, method decs in Core.java


        move(positive_power, negative_power, rot_power);
    }
}
