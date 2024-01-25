package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Drone;

public class SetDrone extends Command {
    Drone drone;
    Drone.DroneStates droneStates;
    boolean hasFinished = false;

    public SetDrone(Drone drone, Drone.DroneStates droneStates) {
        this.drone = drone;
        this.droneStates = droneStates;
    }

    @Override
    public void init() {
        drone.setDrone(droneStates);
    }

    @Override
    public void periodic() {
        drone.setDrone(droneStates);
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
