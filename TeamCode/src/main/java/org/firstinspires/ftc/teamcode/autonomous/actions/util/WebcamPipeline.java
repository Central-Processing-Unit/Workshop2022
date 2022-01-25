package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private Mat lastMat;
    private MatFunction matFunction;

    @Override
    public Mat processFrame(Mat input) {
        lastMat = input;
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

}
