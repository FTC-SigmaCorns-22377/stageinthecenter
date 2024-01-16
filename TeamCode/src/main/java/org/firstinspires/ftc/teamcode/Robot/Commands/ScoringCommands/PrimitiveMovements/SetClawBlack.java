package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetClawBlack extends Command {
    Output output;
    Output.ClawState clawBlackStates;
    boolean hasFinished = false;

    public SetClawBlack(Output output, Output.ClawState clawBlackStates) {
        this.output = output;
        this.clawBlackStates = clawBlackStates;
    }

    @Override
    public void init() {
        output.setClawBlack(clawBlackStates);
    }

    @Override
    public void periodic() {
        output.setClawBlack(clawBlackStates);
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
