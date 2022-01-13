package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;
import org.tensorflow.lite.support.image.TensorImage;

import java.util.List;

public class ObjectDetector {
    private Hardware _hardware;
    private VuforiaLocalizer _vuforia;
    private TFObjectDetector _tfod;
    private OpenCvCamera camera;
    private TeamElementLocation teamElementLocation = TeamElementLocation.LOADING;

    public ObjectDetector(Hardware hardware)
    {
        _hardware = hardware;

        initializeObjectDetector();
    }

    private void handleMat(Mat mat) {
        TensorImageClassifier tic;
        try {
            tic = new TFICBuilder(_hardware.map, "model.tflite", "TeamElement", "NoTeamElement").build();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
        // todo: these threshold values probably need to be changed
        Mat leftMat = mat.submat(new Rect(0, 0, Constants.LEFT_TEAM_ELEMENT_LOCATION_THRESHOLD, Constants.WEBCAM_HEIGHT));
        Mat centerMat = mat.submat(new Rect(Constants.LEFT_TEAM_ELEMENT_LOCATION_THRESHOLD, 0, Constants.RIGHT_TEAM_ELEMENT_LOCATION_THRESHOLD, Constants.WEBCAM_HEIGHT));
        Mat rightMat = mat.submat(new Rect(Constants.RIGHT_TEAM_ELEMENT_LOCATION_THRESHOLD, 0, Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT));
        List<TensorImageClassifier.Recognition> leftOutput = tic.recognize(leftMat);
        List<TensorImageClassifier.Recognition> centerOutput = tic.recognize(centerMat);
        List<TensorImageClassifier.Recognition> rightOutput = tic.recognize(rightMat);
        float leftConfidence = leftOutput.get(0).getConfidence(); // If this doesn't work, it's because the labels are in random order
        float centerConfidence = centerOutput.get(0).getConfidence();
        float rightConfidence = rightOutput.get(0).getConfidence();
        System.out.println("Confidence: " + leftConfidence + " / " + centerConfidence + " / " + rightConfidence);
        if (Math.max(Math.max(leftConfidence, centerConfidence), rightConfidence) < 0.5f) {
            teamElementLocation = TeamElementLocation.INDETERMINATE;
            return;
        }
        if (leftConfidence > centerConfidence) {
            if (leftConfidence > rightConfidence) {
                teamElementLocation = TeamElementLocation.LEFT;
            } else {
                teamElementLocation = TeamElementLocation.RIGHT;
            }
        } else if (centerConfidence > rightConfidence) {
            teamElementLocation = TeamElementLocation.CENTER;
        } else {
            teamElementLocation = TeamElementLocation.RIGHT;
        }
    }

    private void initializeObjectDetector()
    {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        vuforiaParams.cameraName = _hardware.camera;
        _vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

//        TFObjectDetector.Parameters params = new TFObjectDetector.Parameters();
//        params.minResultConfidence = 0.7f;
//        params.isModelTensorFlow2 = true;
//        params.inputSize = 320;
//        _tfod = ClassFactory.getInstance().createTFObjectDetector(params, _vuforia);
//        _tfod.loadModelFromAsset("model.tflite", "TeamElement");

        WebcamPipeline webcamPipeline = new WebcamPipeline();
        webcamPipeline.setMatFunction(m -> {
            System.out.println("Mat function called");
            _hardware.cvCamera.stopStreaming();
            _hardware.cvCamera.closeCameraDevice();
            handleMat(m);
        });
        _hardware.cvCamera.setPipeline(webcamPipeline);
        _hardware.cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                _hardware.cvCamera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT);
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("bruh cv open failed with code " + errorCode);
            }
        });
    }

    //To allow the robot to figure out the position of any element without training an AI model, we will figure out which barcode we can't see.
    public TeamElementLocation getTeamElementLocation() {
        return teamElementLocation;
    }

    public enum TeamElementLocation
    {
        LEFT, CENTER, RIGHT, INDETERMINATE, LOADING
    }
}
