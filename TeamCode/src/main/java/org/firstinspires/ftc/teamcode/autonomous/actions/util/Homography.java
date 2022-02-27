package org.firstinspires.ftc.teamcode.autonomous.actions.util;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class Homography {

    public static final double CAM_PIXEL_HEIGHT = 720;
    public static final double CAM_PIXEL_WIDTH = 1280;

    private static final double FY = -2.3;
    private static final double PYS;
    private static final double PYE;
    private static final double PZ_PYS;
    private static final double PZ_PYE;
    private static final double CAM_OFFSET_Y = 6;
    private static final double CAM_OFFSET_Z = 12.5;

    private static final double QY;
    private static final double RL;
    private static final double FZ;

    static {
        double imageBottom = 16.5;
        double imageTop = 81;
        double imageTopWidth = 63;
        double imageBottomWidth = 17.5;
        double mf = -CAM_OFFSET_Z / (50 - CAM_OFFSET_Y);
        double mf_neg_reciprocal = -(1 / mf);
        FZ = mf * (FY - CAM_OFFSET_Y) + CAM_OFFSET_Z;
        PYS = (mf_neg_reciprocal * CAM_OFFSET_Y + FZ + (FY * FZ) / (imageBottom - FY) - CAM_OFFSET_Z) / ((FZ / (imageBottom - FY)) + mf_neg_reciprocal);
        PYE = (FZ + (FY * FZ) / (imageTop - FY) - CAM_OFFSET_Z + mf_neg_reciprocal * CAM_OFFSET_Y) / ((FZ / (imageTop - FY)) + mf_neg_reciprocal);
        PZ_PYS = mf_neg_reciprocal * (PYS - CAM_OFFSET_Y) + CAM_OFFSET_Z;
        PZ_PYE = mf_neg_reciprocal * (PYE - CAM_OFFSET_Y) + CAM_OFFSET_Z;
        QY = (imageBottomWidth / 2) * ((imageTop - imageBottom) / ((imageBottomWidth / 2) - (imageTopWidth / 2))) + imageBottom;
        RL = 2 * (((imageTopWidth/2 - imageBottomWidth/2) / (imageTop - imageBottom)) * (CAM_OFFSET_Y - imageBottom) + (imageBottomWidth / 2));
    }

    public static Position convertCameraPointToWorldPoint(double x, double y, Position robotPos) {
        double p = 1 - (y / CAM_PIXEL_HEIGHT);
        double ppz = PZ_PYS + (PZ_PYE - PZ_PYS) * p;
        double ppy = PYS + (PYE - PYS) * p;
        double worldYRelativeToRobot = -FZ * ((FY - ppy) / (FZ - ppz)) + FY;
        double rx = RL * (((x/CAM_PIXEL_WIDTH)) - (1/2d));
        double worldXRelativeToRobot = ((-rx) / (QY - CAM_OFFSET_Y)) * (worldYRelativeToRobot - QY);
        AutonCore.telem.addData("wxf: ", worldXRelativeToRobot);
        AutonCore.telem.addData("wyf: ", worldYRelativeToRobot);
        AutonCore.telem.addData("x", x);
        AutonCore.telem.addData("y", y);
        AutonCore.telem.update();
        long t = System.currentTimeMillis();
        while (System.currentTimeMillis() - t < 5000) {

        }
        worldYRelativeToRobot += 7;
        if (worldXRelativeToRobot < 0) {
            worldXRelativeToRobot += 3;
        }
        // todo: when the bobit has to move to the left, it undershoots the x amount. it seems to work fine when it moves right - probs bc of the camera offset
        // (worldYRelativeToRobot, worldXRelativeToRobot) is a coordinate relative to the robot, with the y-axis being in line with the direction that the robot is facing (aka theta is forward)
        // in this coordinate space, (0, 0) is the position of the camera

        // Thus, convert to absolute world space using the algorithm from Encoder
        double theta = robotPos.t;
        double deltaXf = worldXRelativeToRobot * Math.cos(theta) - worldYRelativeToRobot * Math.sin(theta);
        double deltaYf = worldYRelativeToRobot * Math.cos(theta) + worldXRelativeToRobot * Math.sin(theta);

        AutonCore.telem.addData("dxf: ", deltaXf);
        AutonCore.telem.addData("dyf: ", deltaYf);
        AutonCore.telem.addData("x", x);
        AutonCore.telem.addData("y", y);
        AutonCore.telem.update();
        t = System.currentTimeMillis();
        while (System.currentTimeMillis() - t < 5000) {

        }
        return new Position(robotPos.x + 25.4 * deltaXf, robotPos.y + 25.4 * deltaYf, theta);
    }

}

