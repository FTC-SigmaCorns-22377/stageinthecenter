package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetArm extends Command {
    Output output;
    Output.ArmState armState;

    public SetArm(Output output, Output.ArmState armState) {
        this.output = output;
        this.armState = armState;
    }

    @Override
    public void init() {
        output.setArmState(armState);
    }

    @Override
    public void periodic() {}

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
