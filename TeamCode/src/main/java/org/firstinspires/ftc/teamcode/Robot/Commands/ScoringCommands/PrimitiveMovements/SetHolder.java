package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class SetHolder extends Command {
    Intake intake;
    Intake.HolderStates holderStates;
    boolean hasFinished = false;

    public SetHolder(Intake intake, Intake.HolderStates holderStates) {
        this.intake = intake;
        this.holderStates = holderStates;
    }

    @Override
    public void init() {
        intake.setHolder(holderStates);
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
