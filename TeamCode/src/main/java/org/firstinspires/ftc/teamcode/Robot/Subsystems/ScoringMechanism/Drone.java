package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Drone extends Subsystem {
    private static final double HELD = 0.225; // 950
    private static final double RELEASED = 0.3; // 1100
    private static final double DOWN = 0.7; // 1900
    private static final double UP = 0.55; // 1600
    private static final double RETRACT_DELAY = 0.67;

    private DroneState state;
    private ElapsedTime retractTimer;
    private ServoImplEx droneAngle;
    private ServoImplEx droneRelease;

    public void initCommon(HardwareMap hwMap) {
        droneAngle = hwMap.get(ServoImplEx.class, "droneAngle");
        droneRelease = hwMap.get(ServoImplEx.class, "droneRelease");

        droneAngle.setPwmRange(new PwmControl.PwmRange(500, 2500));
        droneRelease.setPwmRange(new PwmControl.PwmRange(500, 2500));

        droneAngle.setPosition(DOWN);
        droneRelease.setPosition(HELD);

        state = DroneState.DOWN;

        retractTimer = new ElapsedTime();
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
        if (state == DroneState.RELEASED && retractTimer.seconds() > RETRACT_DELAY) {
            retract();
        }
    }

    @Override
    public void shutdown() {

    }



    public void ready() {
        if (state == DroneState.DOWN) {
            droneAngle.setPosition(UP);
            state = DroneState.UP;
        }
    }

    public void retract() {
        if (state == DroneState.UP) {
            droneAngle.setPosition(DOWN);
            state = DroneState.DOWN;
        }
        else if (state == DroneState.RELEASED) {
            droneAngle.setPosition(DOWN);
            state = DroneState.RETRACTED;
        }
    }

    public void fire() {
        if (state == DroneState.UP) {
            droneRelease.setPosition(RELEASED);
            state = DroneState.RELEASED;
            retractTimer.reset();
        }
    }

    public DroneState getState() {
        return state;
    }

    public enum DroneState {
        DOWN,
        UP,
        RELEASED,
        RETRACTED
    }
}