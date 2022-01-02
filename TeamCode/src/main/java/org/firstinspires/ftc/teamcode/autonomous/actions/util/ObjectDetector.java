package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;

import java.util.List;

public class ObjectDetector {
    private Hardware _hardware;
    private VuforiaLocalizer _vuforia;
    private TFObjectDetector _tfod;


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
    }

    //To allow the robot to figure out the position of any element without training an AI model, we will figure out which barcode we can't see.
    public BarcodeLocation getRecognition()
    {
        List<Recognition> recognitions = _tfod.getUpdatedRecognitions();

        if (recognitions != null && recognitions.size() > 0)
        {
            double leftMost = 0.0;
            double rightMost = 0.0;

            for (Recognition recognition : recognitions) //Find left most element
            {
                if (recognition.getLeft() < leftMost)
                {
                    leftMost = recognition.getLeft();
                }
            }

            for (Recognition recognition : recognitions) //Find right most element
            {
                if (recognition.getLeft() > rightMost)
                {
                    rightMost = recognition.getLeft();
                }
            }

            double bardcodeDist = rightMost - leftMost;

            if (bardcodeDist > 450) //Distance between rightmost and leftmost barcode. If this margin is around 500mm, we know object is in center.
            {
                return BarcodeLocation.CENTER;
            } else
            {
                if (Constants.IS_LEFT_OPMODE)
                {
                    if (leftMost > 230)
                        return BarcodeLocation.LEFT;
                    else
                        return BarcodeLocation.RIGHT;
                } else if (leftMost > 100)
                    return BarcodeLocation.LEFT;
                else
                    return BarcodeLocation.RIGHT;

                //FIGURE OUT IF ITS ON RIGHT OR LEFT SIDE.
            }
        }

        return BarcodeLocation.CENTER;
    }

    public enum BarcodeLocation
    {
        LEFT, CENTER, RIGHT;
    }
}
