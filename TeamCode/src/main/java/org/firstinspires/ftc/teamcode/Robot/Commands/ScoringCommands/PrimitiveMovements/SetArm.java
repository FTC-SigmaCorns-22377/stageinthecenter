package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetArm extends Command {
    Output output;
    Output.ArmStates armStates;
    boolean hasFinished = false;

    public SetArm(Output output, Output.ArmStates armStates) {
        this.output = output;
        this.armStates = armStates;
    }

    @Override
    public void init() {
        output.setArm(armStates);
    }

    @Override
    public void periodic() {
        output.setArm(armStates);
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
