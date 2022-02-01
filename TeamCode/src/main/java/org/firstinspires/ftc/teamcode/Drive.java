package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class Drive extends Core {
    double positive_power, negative_power, rot_power;
    double joystick_x, joystick_y, joystick_power;
    double orientation;
    int carouselDirection = 0;
    Orientation gyro_angles;
    long prevTime = System.currentTimeMillis();
    boolean isClawClosed;
    long clawOpeningTime = 0;
    long bLastPressed = -1;
    long yLastPressed = -1;
    boolean turboMode;
    double clawPower;
    double armPower;
    double triggerLastPressed;
    boolean isArmRaised;

    public void loop()
    {
        carouselDirection = gamepad1.a ? 1 :
                            gamepad1.x ? -1 :
                                    0;

        if (gamepad1.right_trigger > 0.3 && System.currentTimeMillis() - triggerLastPressed > 250) {
            triggerLastPressed = System.currentTimeMillis();
            isArmRaised = !isArmRaised;
        }

        armPower = isArmRaised ? 0.6 : 0.0;

        if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
            bLastPressed = System.currentTimeMillis();
            isClawClosed = !isClawClosed;
            clawOpeningTime = 0;
            if (isClawClosed) {
                clawPower = 0.6;
            }
        }

        if (!isClawClosed && clawOpeningTime < 100) {
            clawOpeningTime += System.currentTimeMillis() - prevTime;
            clawPower = -0.2;
        } else if (!isClawClosed) {
            clawPower = 0;
        }

        if (gamepad1.y && System.currentTimeMillis() - yLastPressed > 250) {
            yLastPressed = System.currentTimeMillis();
            turboMode = !turboMode;
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
            negative_power *= 0.4;
            positive_power *= 0.4;
            rot_power *= 0.5;
        }

        // This is all we need to actually move the robot, method decs in Core.java
        move(positive_power, negative_power, rot_power);
        moveCarousel(carouselDirection * 0.25);
        moveClaw(clawPower);
        moveArm(armPower);
    }
}
