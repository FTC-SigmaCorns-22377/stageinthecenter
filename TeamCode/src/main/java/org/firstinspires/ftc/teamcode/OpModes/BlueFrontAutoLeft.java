package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.RandomizationSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class BlueFrontAutoLeft extends PerseveranceAuto{
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
    public Side getSide() {
        return Side.FRONT;
    }

    @Override
    public RandomizationSide getRandomization() {
        return RandomizationSide.LEFT;
    }
}
