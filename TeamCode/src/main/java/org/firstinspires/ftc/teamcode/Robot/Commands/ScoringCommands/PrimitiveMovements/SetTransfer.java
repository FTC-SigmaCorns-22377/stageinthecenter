package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class SetTransfer extends Command {
    Intake intake;
    Intake.TransferState transferState;

    public SetTransfer(Intake intake, Intake.TransferState transferState) {
        this.intake = intake;
        this.transferState = transferState;
    }

    @Override
    public void init() {
        intake.setTransferState(transferState);
    }

    @Override
    public void periodic() {
//        intake.setTransferState(transferState);
    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
