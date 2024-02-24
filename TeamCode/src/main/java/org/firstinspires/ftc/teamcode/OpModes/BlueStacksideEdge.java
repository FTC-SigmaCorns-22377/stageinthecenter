package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Park;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous(preselectTeleOp="TeleOp")
public class BlueStacksideEdge extends AutonomousParent {
    @Override
    public Team getTeam() {
        return Team.BLUE;
    }
    public Side getSide() {
        return Side.STACKSIDE;
    }
    public Park getPark() {
        return Park.EDGE;
    }
}