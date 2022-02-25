package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.Homography;
import org.firstinspires.ftc.teamcode.autonomous.actions.util.ObjectDetector;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;
import org.firstinspires.ftc.teamcode.autonomous.waypoint.Waypoint;

public class DriveToFreightAction extends Action {
    private final ObjectDetector objectDetector;
    public DriveToFreightAction(Hardware hardware, Instructions instructions, int index, int priority, ObjectDetector objectDetector) {
        super(hardware, instructions, index, priority);
        this.objectDetector = objectDetector;
    }

    @Override
    public void execute() {
        int[] pixelVals = objectDetector.getFreightPixelPosition();
        AutonCore.telem.addData("px: ", pixelVals[0]);
        AutonCore.telem.addData("py: ", pixelVals[1]);
        AutonCore.telem.update();
        Position targetPos = Homography.convertCameraPointToWorldPoint(pixelVals[0], pixelVals[1], instructions.navigation._localization.getRobotPosition());
        instructions.loopToWaypoint(new Waypoint(targetPos));
    }
}
