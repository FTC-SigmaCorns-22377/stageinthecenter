package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.RandomizationSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class RedFrontAutoRight extends FrontAuto {
    @Override
    public Team getTeam() {
        return Team.RED;
    }
    public Side getSide() {
        return Side.FRONT;
    }

    @Override
    public RandomizationSide getRandomization() {
        return RandomizationSide.RIGHT;
    }
}
