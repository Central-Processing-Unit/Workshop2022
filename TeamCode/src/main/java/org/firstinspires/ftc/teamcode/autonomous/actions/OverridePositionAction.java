package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.Instructions;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Position;

public class OverridePositionAction extends Action
{
    Position pos;

    public OverridePositionAction(Position pos)
    {
        super();
        this.pos = pos;
    }
    public OverridePositionAction(Hardware hardware, Instructions instructions, int index, int priority, Position pos)
    {
        super(hardware, instructions, index, priority);
        this.pos = pos;
    }

    @Override
    public void execute()
    {
        instructions.navigation.position = pos;
    }
}
