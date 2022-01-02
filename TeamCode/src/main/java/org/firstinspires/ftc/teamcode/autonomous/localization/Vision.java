package org.firstinspires.ftc.teamcode.autonomous.localization;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class Vision {
    private Hardware _hardware;

    public VuforiaLocalizer vuforiaLocalizer; //Vuforia instance
    public VuforiaTrackables targets; //Vuforia image
    public List<VuforiaTrackable> trackables;

    public OpenGLMatrix location;

    private static final float mmTargetHeight = 152.4f;
    private static final float halfField = 1828.8f;
    private static final float halfTile = 304.8f;
    private static final float oneAndHalfTile = 914.4f;

    public Boolean targetVisible = false;

    public Vision(Hardware hardware)
    {
        _hardware = hardware;
        initialize();
    }

    public Position getRobotPosition()
    {
        for (VuforiaTrackable trackable : trackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                //Target is visible

                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    location = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {
            VectorF translation = location.getTranslation();
            Orientation orientation = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            Position position = new Position();

            position.x = translation.get(0) / 25.4;
            position.y = translation.get(1) / 25.4;
            position.t = orientation.thirdAngle - Constants.INIT_THETA;

            targetVisible = false;

            return position;
        }

        targetVisible = false;

        return null;
    }

    private void initialize() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        params.cameraName = _hardware.camera;
        params.useExtendedTracking = false;

        vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);
        targets = vuforiaLocalizer.loadTrackablesFromAsset("FreightFrenzy");
        trackables = new ArrayList<>();
        trackables.addAll(targets);

        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90, 0, 180);

        OpenGLMatrix cameraLocation = OpenGLMatrix //We need to describe where the camera is on the robot.
                .translation(4.0f /*Forward displacement from center*/, 0.0f /*Left displacement from center*/, 0f  /*Vertical displacement from ground*/)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : trackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(params.cameraName, cameraLocation);
        }
    }

    //Set location of targets and give the names for trackables
    private void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
