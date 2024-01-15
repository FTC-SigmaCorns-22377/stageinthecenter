package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.HolderStates.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.HolderStates.TRANSFER;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.RollerStates.OFF;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Intake extends Subsystem {
    DcMotorEx roller;
    Servo rollerDropLeft;
    Servo rollerDropRight;
    Servo holder;
    Servo linkage;

    public void initCommon(HardwareMap hwMap) {
        roller = hwMap.get(DcMotorEx.class, "roller");
        rollerDropLeft = hwMap.get(Servo.class, "rollerDropLeft");
        rollerDropRight = hwMap.get(Servo.class, "rollerDropRight");
        holder = hwMap.get(Servo.class, "holder");
        linkage = hwMap.get(Servo.class, "linkage");
        setRoller(OFF);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        initCommon(hwMap);
        setHolder(TRANSFER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        initCommon(hwMap);
        setHolder(INTAKE);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }

    // TUNE
    public void setRoller(RollerStates rollerStates) {
        switch (rollerStates) {
            case OFF:
                roller.setPower(0);
            case ON:
                roller.setPower(0.5);
        }
    }

    // TUNE
    public void setHolder(HolderStates holderStates) {
        switch (holderStates) {
            case INTAKE:
                rollerDropLeft.setPosition(0.72);
                rollerDropRight.setPosition(0.28);
                holder.setPosition(1);
                linkage.setPosition(0.93);
                break;
            case TRANSFER:
                rollerDropLeft.setPosition(0.4);
                rollerDropRight.setPosition(0.6);
                holder.setPosition(0.3);
                linkage.setPosition(0.8);
                break;
        }
    }

    public enum RollerStates {
        OFF,
        ON
    }

    public enum HolderStates {
        INTAKE,
        TRANSFER
    }
}