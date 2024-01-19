package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RaceAction;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

public class SetTransferSafe extends RaceAction {
    Intake intake;
    Intake.TransferState transferState;

    Output output;

    Output.ArmState armState;

    public SetTransferSafe(Intake intake, Intake.TransferState transferState, Output output, Output.ArmState armState) {
        this.intake = intake;
        this.transferState = transferState;
        this.output = output;
        this.armState = armState;
    }

    @Override
    public void init() {
        intake.setTransferState(transferState);
    }

    @Override
    public void periodic() {
        if(transferState == Intake.TransferState.TRANSFER){
            output.setArmState(Output.ArmState.TRANSFER);
        }

        intake.setTransferState(transferState);
    }

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {

    }
}
