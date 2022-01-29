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
    private OpenCvCamera camera;
    private TeamElementLocation teamElementLocation = TeamElementLocation.LOADING;
    public static TensorImageClassifier tic;
    private WebcamPipeline webcamPipeline;

    public ObjectDetector(Hardware hardware)
    {
        _hardware = hardware;
        initializeObjectDetector();
    }

    private static float getConfidence(List<TensorImageClassifier.Recognition> output) {
        if (output.size() < 2) {
            return -1;
        }
        return output.get(0).getTitle().equals("TeamElement") ? output.get(0).getConfidence() : output.get(1).getConfidence();
    }

    public void calculateState() {
        _hardware.cvCamera.stopStreaming();
        _hardware.cvCamera.closeCameraDevice();

        System.out.println("handleMat");
        if (tic == null || webcamPipeline.getLastMat() == null) {
            AutonCore.telem.addLine("Last mat: " + webcamPipeline.getLastMat());
            AutonCore.telem.update();
            return;
        }
        Mat mat = webcamPipeline.getLastMat();
        // todo: these threshold values probably need to be changed
        Mat rawLeftMat = mat.submat(new Rect(0, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat leftMat = new Mat();
        Imgproc.resize(rawLeftMat, leftMat, new Size(224, 224));
        List<TensorImageClassifier.Recognition> leftOutput = tic.recognize(leftMat);
        float leftConfidence = getConfidence(leftOutput);
        if (leftConfidence > 0.5) {
            teamElementLocation = TeamElementLocation.LEFT;
            AutonCore.telem.addLine("LEFT: " + leftConfidence);
            AutonCore.telem.update();
            return;
        }
        Mat rawCenterMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
        Mat centerMat = new Mat();
        Imgproc.resize(rawCenterMat, centerMat, new Size(224, 224));
        List<TensorImageClassifier.Recognition> centerOutput = tic.recognize(centerMat);
        float centerConfidence = getConfidence(centerOutput);
        if (centerConfidence > 0.5) {
            teamElementLocation = TeamElementLocation.CENTER;
            AutonCore.telem.addLine("CENTER: " + centerConfidence);
            AutonCore.telem.update();
            return;
        }
        if (leftConfidence > 0.2 || centerConfidence > 0.2) {
            Mat rawRightMat = mat.submat(new Rect(Constants.WEBCAM_SECTION_WIDTH, 0, Constants.WEBCAM_SECTION_WIDTH, Constants.WEBCAM_HEIGHT));
            Mat rightMat = new Mat();
            Imgproc.resize(rawRightMat, rightMat, new Size(224, 224));
            List<TensorImageClassifier.Recognition> rightOutput = tic.recognize(rightMat);
            float rightConfidence = getConfidence(rightOutput);
            if (rightConfidence > centerConfidence) {
                if (rightConfidence > leftConfidence) {
                    teamElementLocation = TeamElementLocation.RIGHT;
                    AutonCore.telem.addLine("RIGHT: " + rightConfidence);
                } else {
                    teamElementLocation = TeamElementLocation.CENTER;
                    AutonCore.telem.addLine("CENTER: " + centerConfidence);
                }
                AutonCore.telem.update();
            } else if (centerConfidence > leftConfidence) {
                teamElementLocation = TeamElementLocation.CENTER;
                AutonCore.telem.addLine("CENTER: " + centerConfidence);
                AutonCore.telem.update();
            } else {
                teamElementLocation = TeamElementLocation.LEFT;
                AutonCore.telem.addLine("LEFT: " + leftConfidence);
                AutonCore.telem.update();
            }
        } else {
            teamElementLocation = TeamElementLocation.RIGHT;
            AutonCore.telem.addLine("RIGHT: " + leftConfidence + "; " + centerConfidence);
            AutonCore.telem.update();
        }
    }

    private void initializeObjectDetector()
    {
        webcamPipeline = new WebcamPipeline();
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
