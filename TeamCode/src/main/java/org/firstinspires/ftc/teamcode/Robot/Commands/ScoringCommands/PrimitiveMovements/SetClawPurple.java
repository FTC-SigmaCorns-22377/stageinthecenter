package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetClawPurple extends Command {
    Output output;
    Output.ClawPurpleStates clawPurpleStates;
    boolean hasFinished = false;

    public SetClawPurple(Output claw, Output.ClawPurpleStates clawPurpleStates) {
        this.output = output;
        this.clawPurpleStates = clawPurpleStates;
    }

    @Override
    public void init() {
        output.setClawPurple(clawPurpleStates);
    }

    @Override
    public void periodic() {
        output.setClawPurple(clawPurpleStates);
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
