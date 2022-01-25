package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TFICBuilder;
import org.outoftheboxrobotics.tensorflowapi.ImageClassification.TensorImageClassifier;

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
            tic = new TFICBuilder(_hardware.map, "model.tflite", "TeamElement", "NoTeamElement").setQuantized(true).build();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }
        // todo: these threshold values probably need to be changed
        AutonCore.telem.addData("Beginning crop: %s", mat.width() + " x " + mat.height() + "; c: " + mat.channels());
        AutonCore.telem.update();
        Mat rawLeftMat = mat.submat(new Rect(0, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat leftMat = new Mat();
        Imgproc.resize(rawLeftMat, leftMat, new Size(224, 224));
        AutonCore.telem.addData("Finished first crop: %s", leftMat.width() + " x " + leftMat.height() + "; c: " + leftMat.channels() + "; l: " + (leftMat.total() * leftMat.channels()));
        AutonCore.telem.update();
        Mat rawCenterMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat centerMat = new Mat();
        Imgproc.resize(rawCenterMat, centerMat, new Size(224, 224));
        Mat rawRightMap = mat.submat(new Rect(2 * Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_WIDTH - 2 * Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat rightMat = new Mat();
        Imgproc.resize(rawRightMap, rightMat, new Size(224, 224));
        List<TensorImageClassifier.Recognition> leftOutput = tic.recognize(leftMat);
        AutonCore.telem.addLine("Recognized left mat");
        AutonCore.telem.update();
        List<TensorImageClassifier.Recognition> centerOutput = tic.recognize(centerMat);
        List<TensorImageClassifier.Recognition> rightOutput = tic.recognize(rightMat);
        float leftConfidence = leftOutput.get(0).getTitle().equals("TeamElement") ? leftOutput.get(0).getConfidence() : 1 - leftOutput.get(1).getConfidence();
        float centerConfidence = centerOutput.get(0).getTitle().equals("TeamElement") ? centerOutput.get(0).getConfidence() : 1 - centerOutput.get(1).getConfidence();
        float rightConfidence = rightOutput.get(0).getTitle().equals("TeamElement") ? rightOutput.get(0).getConfidence() : 1 - rightOutput.get(1).getConfidence();
        System.out.println("Confidence: " + leftConfidence + " / " + centerConfidence + " / " + rightConfidence);
        System.out.println("Left: " + leftOutput.size() + "; " + leftOutput.get(0).getTitle() + ": " + leftOutput.get(0).getConfidence() + "; " + leftOutput.get(1).getTitle() + ": " + leftOutput.get(1).getConfidence());
        System.out.println("Center: " + centerOutput.size() + "; " + centerOutput.get(0).getTitle() + ": " + centerOutput.get(0).getConfidence() + "; " + centerOutput.get(1).getTitle() + ": " + centerOutput.get(1).getConfidence());
        System.out.println("Right: " + rightOutput.size() + "; " + rightOutput.get(0).getTitle() + ": " + rightOutput.get(0).getConfidence() + "; " + rightOutput.get(1).getTitle() + ": " + rightOutput.get(1).getConfidence());
        AutonCore.telem.addLine("Confidence: " + leftConfidence + " / " + centerConfidence + " / " + rightConfidence);
        AutonCore.telem.update();
        if (Math.max(Math.max(leftConfidence, centerConfidence), rightConfidence) < 0.5f) {
            teamElementLocation = TeamElementLocation.INDETERMINATE;
            AutonCore.telem.addData("TeamElementLocation: INDETERMINATE %s", teamElementLocation.toString());
            AutonCore.telem.update();
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
        AutonCore.telem.addData("TeamElementLocation: %s", teamElementLocation.toString());
        AutonCore.telem.update();
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
            AutonCore.telem.addLine("MatFunction");
            AutonCore.telem.update();
            System.out.println("Mat function called");
            _hardware.cvCamera.stopStreaming();
            _hardware.cvCamera.closeCameraDevice();
            handleMat(m);
        });
        _hardware.cvCamera.setPipeline(webcamPipeline);
        _hardware.cvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                _hardware.cvCamera.startStreaming(Constants.WEBCAM_WIDTH, Constants.WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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
