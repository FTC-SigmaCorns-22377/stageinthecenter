package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Hang;

public class SetHang extends Command {
    Hang hang;
    Hang.Hanging hanging;
    //Double power;
    //boolean hasFinished = false;

    public SetHang(Hang hang, Hang.Hanging hanging) {
        this.hang = hang;
        this.hanging = hanging;
    }

    @Override
    public void init() {
        //hang.setHang(hanging);
        hang.setHanging(hanging);
    }


    @Override
    public void periodic() {
//        hang.setHang(hanging);
//        hasFinished = true;
    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}