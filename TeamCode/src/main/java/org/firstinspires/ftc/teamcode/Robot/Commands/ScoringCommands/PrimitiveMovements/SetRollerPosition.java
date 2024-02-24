package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;

public class SetRollerPosition extends Command {
	Intake intake;
	Intake.TransferState transferState;
	public SetRollerPosition(Intake intake, Intake.TransferState transferState) {
		this.intake = intake;
		this.transferState = transferState;
	}

	@Override
	public void init() {
		intake.rawSetServos(transferState);
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
