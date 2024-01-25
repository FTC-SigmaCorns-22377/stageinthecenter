package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class ToggleTransfer extends Command {
    Intake intake;

    public ToggleTransfer(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void init() {
        intake.toggleTransferState();
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
