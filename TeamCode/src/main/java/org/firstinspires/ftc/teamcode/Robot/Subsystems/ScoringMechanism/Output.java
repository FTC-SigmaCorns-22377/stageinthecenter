package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.ArmStates.TRANSFER;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.WristStates.DEG0;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Output extends Subsystem {
    Servo armLeft;
    Servo armRight;
    Servo wrist;
    Servo clawPurple;
    Servo clawBlack;

    public void initCommon(HardwareMap hwMap) {
        armLeft = hwMap.get(Servo.class, "armLeft");
        armRight = hwMap.get(Servo.class, "armRight");
        wrist = hwMap.get(Servo.class, "wrist");
        clawPurple = hwMap.get(Servo.class, "clawPurple");
        clawBlack = hwMap.get(Servo.class, "clawBlack");
        setArm(TRANSFER);
        setWrist(DEG0);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        initCommon(hwMap);
        setClawPurple(ClawPurpleStates.CLOSED);
        setClawBlack(ClawBlackStates.CLOSED);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        initCommon(hwMap);
        setClawPurple(ClawPurpleStates.OPEN);
        setClawBlack(ClawBlackStates.OPEN);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }

    // TUNE
    public void setArm(ArmStates armStates) {
        switch (armStates) {
            case TRANSFER:
                armLeft.setPosition(0);
                armRight.setPosition(1);
                break;
            case SCORE:
                armLeft.setPosition(0.5);
                armRight.setPosition(0.5);
                break;
        }
    }

    // TUNE
    public void setWrist(WristStates wristStates) {
        switch (wristStates) {
            case DEG0:
                wrist.setPosition(0);
                break;
            case DEG60:
                wrist.setPosition(0.167);
                break;
            case DEG120:
                wrist.setPosition(0.333);
                break;
            case DEG180:
                wrist.setPosition(0.5);
                break;
            case DEG240:
                wrist.setPosition(0.666);
                break;
            case DEG300:
                wrist.setPosition(0.833);
                break;
        }
    }

    // TUNE
    public void setClawPurple(ClawPurpleStates clawPurpleState) {
        switch (clawPurpleState) {
            case OPEN:
                clawPurple.setPosition(0.5);
                break;
            case CLOSED:
                clawPurple.setPosition(0);
                break;
        }
    }

    // TUNE
    public void setClawBlack(ClawBlackStates clawBlackState) {
        switch (clawBlackState) {
            case OPEN:
                clawBlack.setPosition(0.5);
                break;
            case CLOSED:
                clawBlack.setPosition(0);
                break;
        }
    }

    public enum ArmStates {
        TRANSFER,
        SCORE
    }

    public enum WristStates {
        DEG0,
        DEG60,
        DEG120,
        DEG180,
        DEG240,
        DEG300
    }

    public enum ClawPurpleStates {
        OPEN,
        CLOSED
    }

    public enum ClawBlackStates {
        OPEN,
        CLOSED
    }
}