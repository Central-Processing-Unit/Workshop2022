package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.function.BiFunction;
import java.util.function.Predicate;

public class WebcamPipeline extends OpenCvPipeline {

    private Mat lastMat;
    private OpenCvCamera openCvCamera;
    private MatFunction matFunction;

    @Override
    public Mat processFrame(Mat input) {
        lastMat = input;
        openCvCamera.stopRecordingPipeline();
        openCvCamera.stopStreaming();
        matFunction.handleMat(lastMat);
        System.out.println("new frame just dropped 0.0");
        return input;
    }

    public void setMatFunction(MatFunction m) {
        matFunction = m;
    }

    public Mat getLastMat() {
        return lastMat;
    }

    public void setOpenCvCamera(OpenCvCamera openCvCamera) {
        this.openCvCamera = openCvCamera;
    }

}
