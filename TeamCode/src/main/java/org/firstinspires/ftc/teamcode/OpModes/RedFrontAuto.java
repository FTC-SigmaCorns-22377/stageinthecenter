package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.Side;

@Autonomous
public class RedFrontAuto extends FrontAuto {
    @Override
    public Team getTeam() {
        return Team.RED;
    }
    public Side getSide() {
        return Side.FRONT;
    }
}
