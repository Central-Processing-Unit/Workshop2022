package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.opencv.core.Mat;

@FunctionalInterface
public interface MatFunction {

    void handleMat(Mat mat);

}
