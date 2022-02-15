package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public class DetectFreightAction extends Action {
    private final ObjectDetector objectDetector;

    public DetectFreightAction(int index, int priority, ObjectDetector objectDetector) {
        super(index, priority);
        this.objectDetector = objectDetector;
    }

    @Override
    public void execute(Hardware hardware, Localization localization) {
        objectDetector.startFreightDetection();
    }
}
