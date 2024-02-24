package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Drone;

public class FireDroneSequence extends Command {
    Drone drone;

    public FireDroneSequence(Drone drone) {
        this.drone = drone;
    }

    @Override
    public void init() {
        if (drone.getState() == Drone.DroneState.DOWN) {
            drone.ready();
        }
        else if (drone.getState() == Drone.DroneState.UP) {
            drone.fire();
        }
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
