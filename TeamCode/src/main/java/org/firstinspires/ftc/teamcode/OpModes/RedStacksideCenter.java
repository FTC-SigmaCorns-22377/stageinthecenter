package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Park;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class RedStacksideCenter extends AutonomousParent {
    @Override
    public Team getTeam() {
        return Team.RED;
    }
    public Side getSide() {
        return Side.STACKSIDE;
    }
    public Park getPark() {
        return Park.CENTER;
    }
}