package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ObjectDetector {
    private Hardware _hardware;
    private TeamElementLocation teamElementLocation = TeamElementLocation.LOADING;
    private WebcamPipeline webcamPipeline;

    public ObjectDetector(Hardware hardware)
    {
        _hardware = hardware;
        initializeObjectDetector();
    }

    public int[] getFreightPixelPosition(boolean isDuck) {
        Mat mat = webcamPipeline.getLastMat();
        Mat sm = mat.submat(0, mat.rows(), mat.cols() / 3, 2 * mat.cols() / 3);
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(sm, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat output = new Mat();
        Core.inRange(hsvMat, new Scalar(20, isDuck ? 100 : 50, 100), new Scalar(150, 350, 500), output);
        int mi = -1, mj = -1;
        outer: for (int j = output.rows() - 1; j >= 10; j -= 5) {
            pixelloop: for (int i = 0; i < output.cols(); i += 5) {
                for (int k = 0; k < 25; k++) {
                    if (output.get(j, i - k)[0] == 0) {
                        continue pixelloop;
                    }
                }
                mi = i;
                mj = j;
                break outer;
            }
        }
        return new int[]{mi + mat.cols() / 3, mj};
    }

    public void calculateState() {
        System.out.println("handleMat");
        if (webcamPipeline.getLastMat() == null) {
            AutonCore.telem.addLine("Last mat is null");
            AutonCore.telem.update();
            return;
        }
        Mat mat = webcamPipeline.getLastMat();
        Mat hsvMat = new Mat();
        // todo: maybe change this to BGR2HSV
        Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat filtered = new Mat();
        Core.inRange(hsvMat, new Scalar(40, 50, 100), new Scalar(80, 75, 200), filtered);

        int leftMatchingPixels = 0;
        for (int i = 0; i < Constants.WEBCAM_SECTION_WIDTH; i++) {
            for (int j = 0; j < Constants.WEBCAM_HEIGHT; j += 5) {
                double[] pixelVals = filtered.get(j, i);
                if (pixelVals[0] > 0) { // If is in HSV range, the pixelVals will be {255, 255, 255}
                    leftMatchingPixels++;
                }
            }
        }
        int centerMatchingPixels = 0;
        for (int i = Constants.WEBCAM_SECTION_WIDTH; i < Constants.WEBCAM_WIDTH - Constants.WEBCAM_SECTION_WIDTH; i++) {
            for (int j = 0; j < Constants.WEBCAM_HEIGHT; j += 5) {
                double[] pixelVals = filtered.get(j, i);
                if (pixelVals[0] > 0) {
                    centerMatchingPixels++;
                }
            }
        }
        int rightMatchingPixels = 0;
        for (int i = Constants.WEBCAM_WIDTH - Constants.WEBCAM_SECTION_WIDTH; i < Constants.WEBCAM_WIDTH; i++) {
            for (int j = 0; j < Constants.WEBCAM_HEIGHT; j += 5) {
                double[] pixelVals = filtered.get(j, i);
                if (pixelVals[0] > 0) {
                    rightMatchingPixels++;
                }
            }
        }
        if (leftMatchingPixels >= centerMatchingPixels) {
            if (leftMatchingPixels >= rightMatchingPixels) {
                teamElementLocation = TeamElementLocation.LEFT;
            } else {
                teamElementLocation = TeamElementLocation.RIGHT;
            }
        } else if (centerMatchingPixels >= rightMatchingPixels) {
            teamElementLocation = TeamElementLocation.CENTER;
        } else {
            teamElementLocation = TeamElementLocation.RIGHT;
        }
        AutonCore.telem.addData("TeamElementLocation: ", teamElementLocation.toString());
        AutonCore.telem.update();
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
