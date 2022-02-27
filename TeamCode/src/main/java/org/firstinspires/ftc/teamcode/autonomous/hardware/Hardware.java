package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Hardware {
    public DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    public DcMotor armMotor, carouselMotor;
    public Servo clawServo;
    public BNO055IMU gyro;
    public WebcamName camera;
    public OpenCvCamera cvCamera;
    public HardwareMap map;
    public ColorSensor colorSensor;

    public Hardware (HardwareMap hardware)
    {
        this.map = hardware;
        initializeGyro(hardware);
        initializeDriveMotors(hardware);
        initializeAccessoryMotors(hardware);
        initializeWebcam(hardware);
        initializeColorSensor(hardware);
    }

    private void initializeDriveMotors(HardwareMap hardware)
    {
        leftFrontMotor = hardware.dcMotor.get("leftFrontMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor = hardware.dcMotor.get("leftBackMotor");
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor = hardware.dcMotor.get("rightFrontMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackMotor = hardware.dcMotor.get("rightBackMotor");
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setMotorValues(double posinput, double neginput)
    {
        leftFrontMotor.setPower(-posinput);
        rightFrontMotor.setPower(neginput);
        leftBackMotor.setPower(-neginput);
        rightBackMotor.setPower(posinput);
    }

    public void setMotorValuesWithRotation(double posinput, double neginput, double rotinput)
    {
        leftFrontMotor.setPower(-posinput-rotinput);
        rightFrontMotor.setPower(neginput-rotinput);
        leftBackMotor.setPower(-neginput-rotinput);
        rightBackMotor.setPower(posinput-rotinput);
    }

    public void setAllMotorPowers(double input)
    {
        leftFrontMotor.setPower(input);
        rightFrontMotor.setPower(input);
        leftBackMotor.setPower(input);
        rightBackMotor.setPower(input);
    }

    private void initializeAccessoryMotors(HardwareMap hardware)
    {
        carouselMotor = hardware.dcMotor.get("carouselMotor");
        carouselMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor = hardware.dcMotor.get("armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clawServo = hardware.get(Servo.class, "clawServo");
    }

    private void initializeGyro(HardwareMap hardware)
    {
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro = hardware.get(BNO055IMU.class, "imu");
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 100);
        gyro.initialize(params);
    }

    private void initializeWebcam(HardwareMap hardware)
    {
        camera = hardware.get(WebcamName.class, "PrimaryWebcam");
        WebcamName webcamName = hardware.get(WebcamName.class, "PrimaryWebcam");
        cvCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
    }

    private void initializeColorSensor(HardwareMap hardware) {
        colorSensor = hardware.colorSensor.get("color");
    }

}
