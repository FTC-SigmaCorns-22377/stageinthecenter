package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class BlueFrontAuto extends FrontAuto {
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
    public Side getSide() {
        return Side.FRONT;
    }
}
