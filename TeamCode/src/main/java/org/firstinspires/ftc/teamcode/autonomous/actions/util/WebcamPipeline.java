package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private Mat lastMat;
    private final ObjectDetector objectDetector;

    public WebcamPipeline(ObjectDetector objectDetector) {
        super();
        this.objectDetector = objectDetector;
    }

    @Override
    public Mat processFrame(Mat input) {
        lastMat = input;
        if (objectDetector.isDetectingFreight()) {
            objectDetector.findFreightLocation(input);
        } else {
            AutonCore.telem.addLine("Webcam ready");
            AutonCore.telem.update();
        }
        System.out.println("new frame just dropped 0.0");
        return input;
    }

    public Mat getLastMat() {
        return lastMat;
    }

}
