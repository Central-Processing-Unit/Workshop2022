package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

public class ObjectDetector {
    private Hardware _hardware;
    private VuforiaLocalizer _vuforia;
    private TFObjectDetector _tfod;
    private OpenCvCamera camera;

    public ObjectDetector(Hardware hardware)
    {
        _hardware = hardware;

        initializeObjectDetector();
    }

    private void initializeObjectDetector()
    {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        vuforiaParams.cameraName = _hardware.camera;
        _vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters();
        params.minResultConfidence = 0.7f;
        params.isModelTensorFlow2 = true;
        params.inputSize = 320;
        _tfod = ClassFactory.getInstance().createTFObjectDetector(params, _vuforia);
        _tfod.loadModelFromAsset("model.tflite", "TeamElement");
    }

    //To allow the robot to figure out the position of any element without training an AI model, we will figure out which barcode we can't see.
    public TeamElementLocation getTeamElementLocation() {

        List<Recognition> recognitions = _tfod.getUpdatedRecognitions();
//        camera.stopRecordingPipeline();
//        camera.closeCameraDevice();
        if (recognitions != null && recognitions.size() > 0)
        {
            Recognition box = null;
            for (Recognition recognition : recognitions) //Find right most element
            {
                if (recognition.equals("TeamElement"))
                {
                    box = recognition;
                    break;
                }
            }

            if (box == null) {
                return TeamElementLocation.INDETERMINATE; // fallback case, pretty much an error
            }

            if (box.getLeft() > 700) // Arbitrary number for detecting whether the team element is in the rightmost position
            {
                return TeamElementLocation.RIGHT;
            } else if (box.getLeft() > 400) {
                return TeamElementLocation.CENTER;
            } else {
                return TeamElementLocation.LEFT;
            }
        }

        return TeamElementLocation.CENTER;
    }

    public enum TeamElementLocation
    {
        LEFT, CENTER, RIGHT, INDETERMINATE
    }
}
