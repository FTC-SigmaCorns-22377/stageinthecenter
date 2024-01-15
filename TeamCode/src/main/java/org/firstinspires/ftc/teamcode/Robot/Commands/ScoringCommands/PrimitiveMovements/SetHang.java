package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Hang;

public class SetHang extends Command {
    Hang hang;
    Double power;
    boolean hasFinished = false;

    public SetHang(Hang hang, Double power) {
        this.hang = hang;
        this.power = power;
    }

    @Override
    public void init() {
        hang.setHang(power);
    }

    @Override
    public void periodic() {
        hang.setHang(power);
        hasFinished = true;
    }

    @Override
    public boolean completed() {
        return hasFinished;
    }

    @Override
    public void shutdown() {

    }
}
