package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Drone;

public class RetractDrone extends Command {
	Drone drone;

	public RetractDrone(Drone drone) {
		this.drone = drone;
	}

	@Override
	public void init() {
		drone.retract();
	}

	@Override
	public void periodic() {}

	@Override
	public boolean completed() {
		return true;
	}

	@Override
	public void shutdown() {}
}
