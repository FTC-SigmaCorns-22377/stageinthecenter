package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Hang;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class MoveHang extends Command {
    Hang hang;
    Hang.HangState hangState;

    public MoveHang(Hang hang, Hang.HangState hangState) {
        this.hang = hang;
        this.hangState = hangState;
    }

    @Override
    public void init() {
        hang.setHang(hangState);
    }

    @Override
    public void periodic() {}

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
