package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.teamcode.Utils.Side;

@Autonomous
public class BlueFrontAuto extends PerseveranceAuto{
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
    public Side getSide() {
        return Side.FRONT;
    }
}
