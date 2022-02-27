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
    private final boolean isDuck;

    public DriveToFreightAction(Hardware hardware, Instructions instructions, int index, int priority, ObjectDetector objectDetector, boolean isDuck) {
        super(hardware, instructions, index, priority);
        this.objectDetector = objectDetector;
        this.isDuck = isDuck;
    }

    @Override
    public void execute() {
        int[] pixelVals = objectDetector.getFreightPixelPosition(isDuck);
        Position targetPos = Homography.convertCameraPointToWorldPoint(pixelVals[0], pixelVals[1], instructions.navigation._localization.getRobotPosition());
        AutonCore.telem.addData("tx: ", pixelVals[0]);
        AutonCore.telem.addData("ty: ", pixelVals[1]);
        AutonCore.telem.update();
        if (pixelVals[0] != -1 && pixelVals[1] != -1) {
            instructions.loopToWaypoint(new Waypoint(targetPos));
        }
    }
}
