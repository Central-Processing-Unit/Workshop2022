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

    public DriveToFreightAction(ObjectDetector objectDetector, boolean isDuck) {
        super();
        this.objectDetector = objectDetector;
        this.isDuck = isDuck;
    }

    public DriveToFreightAction(Hardware hardware, Instructions instructions, int index, int priority, ObjectDetector objectDetector, boolean isDuck) {
        super(hardware, instructions, index, priority);
        this.objectDetector = objectDetector;
        this.isDuck = isDuck;
    }

    @Override
    public void execute() {
        int[] pixelVals = objectDetector.getFreightPixelPosition(isDuck);
        Position targetPos = Homography.convertCameraPointToWorldPoint(pixelVals[0], pixelVals[1], instructions.navigation._localization.getRobotPosition());
        long t = System.currentTimeMillis();
//        while (System.currentTimeMillis() - t < 10000) {}
        if (pixelVals[0] != -1 && pixelVals[1] != -1) {
            instructions.loopToWaypoint(new Waypoint(targetPos), true);
            AutonCore.telem.addLine("drove to freight");
            AutonCore.telem.update();
        }
    }
}
