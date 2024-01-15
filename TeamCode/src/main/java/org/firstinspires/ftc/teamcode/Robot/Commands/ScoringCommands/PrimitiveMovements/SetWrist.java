package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetWrist extends Command {
    Output output;
    Output.WristStates wristStates;
    boolean hasFinished = false;

    public SetWrist(Output output, Output.WristStates wristStates) {
        this.output = output;
        this.wristStates = wristStates;
    }

    @Override
    public void init() {
        output.setWrist(wristStates);
    }

    @Override
    public void periodic() {
        output.setWrist(wristStates);
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
