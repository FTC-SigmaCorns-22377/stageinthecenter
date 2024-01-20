package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Drone extends Subsystem {
    Servo drone;

    public void initCommon(HardwareMap hwMap) {
        drone = hwMap.get(Servo.class, "drone");
        setDrone(DroneStates.IN);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        initCommon(hwMap);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        initCommon(hwMap);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }

    // TUNE
    public void setDrone(DroneStates droneStates) {
        switch (droneStates) {
            case IN:
                drone.setPosition(0);
            case OUT:
                drone.setPosition(1);
        }
    }

    public enum DroneStates {
        IN,
        OUT
    }
}