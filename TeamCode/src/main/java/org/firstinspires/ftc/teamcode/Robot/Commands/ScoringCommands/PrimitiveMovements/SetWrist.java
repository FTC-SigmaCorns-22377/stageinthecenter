package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetWrist extends Command {
    Output output;
    Output.WristState wristState;
    boolean hasFinished = false;

    public SetWrist(Output output, Output.WristState wristState) {
        this.output = output;
        this.wristState = wristState;
    }

    @Override
    public void init() {
        output.setWrist(wristState);
    }

    @Override
    public void periodic() {
        output.setWrist(wristState);
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
