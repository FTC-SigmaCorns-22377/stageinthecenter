package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetClaw extends Command {
    Output output;
    Output.ClawState clawStates;
    boolean hasFinished = false;

    public SetClaw(Output output, Output.ClawState clawStates) {
        this.output = output;
        this.clawStates = clawStates;
    }

    @Override
    public void init() {
        output.setClawBlack(clawStates);
        output.setClawPurple(clawStates);
    }

    @Override
    public void periodic() {
        output.setClawBlack(clawStates);
        output.setClawPurple(clawStates);
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
