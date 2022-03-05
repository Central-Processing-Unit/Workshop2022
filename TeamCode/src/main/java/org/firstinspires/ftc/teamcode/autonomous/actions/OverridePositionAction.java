package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class OverridePositionAction extends Action {
    private final Double tx;
    private final Double ty;

    public OverridePositionAction(Double x, Double y)
    {
        super();
        this.tx = x;
        this.ty = y;
    }

    public OverridePositionAction(Hardware hardware, Instructions instructions, int index, int priority, Double x, Double y)
    {
        super(hardware, instructions, index, priority);
        this.tx = x;
        this.ty = y;
    }
    @Override
    public void execute() {
        instructions.navigation._localization.increment(new Position(tx != null ? tx : instructions.navigation._localization.getRobotPosition().x, ty != null ? ty : instructions.navigation._localization.getRobotPosition().y, instructions.navigation._localization.getRobotPosition().t));
        instructions.navigation._localization.increment(new Position(tx != null ? tx : instructions.navigation._localization.getRobotPosition().x, ty != null ? ty : instructions.navigation._localization.getRobotPosition().y, instructions.navigation._localization.getRobotPosition().t));
    }
}
