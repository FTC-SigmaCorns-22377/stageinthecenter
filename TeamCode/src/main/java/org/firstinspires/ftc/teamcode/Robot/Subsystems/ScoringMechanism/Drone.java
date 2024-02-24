package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Drone extends Subsystem {
    private static final double HELD = 0.225; // 950
    private static final double RELEASED = 0.3; // 1100
    private static final double DOWN = 0.225; // 950
    private static final double UP = 0.3; // 1100

    Servo droneAngle;
    Servo droneRelease;

    public void initCommon(HardwareMap hwMap) {
        droneAngle = hwMap.get(Servo.class, "droneAngle");
        droneRelease = hwMap.get(Servo.class, "droneRelease");
//        setDrone(DroneStates.IN);
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
//        switch (droneStates) {
//            case IN:
//                drone.setPosition(0);
//            case OUT:
//                drone.setPosition(1);
//        }
    }

    public enum DroneStates {
        DOWN,
        UP,
        RELEASED
    }
}