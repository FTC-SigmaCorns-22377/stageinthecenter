package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetClawPurple extends Command {
    Output output;
    Output.ClawState clawState;
    boolean hasFinished = false;

    public SetClawPurple(Output claw, Output.ClawState clawState) {
        this.output = output;
        this.clawState = clawState;
    }

    @Override
    public void init() {
        output.setClawPurple(clawState);
    }

    @Override
    public void periodic() {
        output.setClawPurple(clawState);
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
