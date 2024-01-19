package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.TransferState.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.TransferState.TRANSFER;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.RollerState.OFF;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Intake extends Subsystem {
    public static double INTAKE_POWER = 0.7;
    public static double TRANSFER_DROP_INTAKE_VALUE = 0.22;
    public static double TRANSFER_DROP_TRANSFER_VALUE = -0.1;
    public static double TRANSFER_ANGLE_INTAKE_VALUE = 1;
    public static double TRANSFER_ANGLE_TRANSFER_VALUE = 0.2;
    DcMotorEx roller;
    Servo rollerDropLeft;
    Servo rollerDropRight;
    Servo transferAngle;
    Servo linkage;

    RollerState rollerState;
    TransferState transferState;
    ElapsedTime transferTimer = new ElapsedTime();

    public void initCommon(HardwareMap hwMap) {
        roller = hwMap.get(DcMotorEx.class, "roller");
        rollerDropLeft = hwMap.get(Servo.class, "rollerDropLeft");
        rollerDropRight = hwMap.get(Servo.class, "rollerDropRight");
        transferAngle = hwMap.get(Servo.class, "holder");
        linkage = hwMap.get(Servo.class, "linkage");

        roller.setDirection(DcMotorSimple.Direction.REVERSE);

        setRoller(OFF);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        initCommon(hwMap);
        setTransferState(TRANSFER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        initCommon(hwMap);
        setTransferState(INTAKE);
    }

    @Override
    public void periodic() {
        System.out.println("running roller set " + rollerState.name());
        System.out.println("running holder set " + transferState.name());
        switch (rollerState) {
            case OFF:
                roller.setPower(0);
                break;
            case INTAKE:
                roller.setPower(INTAKE_POWER);
                break;
            case OUTTAKE:
                roller.setPower(-INTAKE_POWER*0.7);
                break;
        }
        switch (transferState) {
            case INTAKE:
                if (transferTimer.seconds() > 0.2) {
                    linkage.setPosition(0.95);
                    transferAngle.setPosition(TRANSFER_ANGLE_INTAKE_VALUE);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(0.85);
                } else {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_DROP_INTAKE_VALUE);
                    rollerDropRight.setPosition(0.5 - TRANSFER_DROP_INTAKE_VALUE);
                }
                break;
            case TRANSFER:
                if (transferTimer.seconds() > 0.2) {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_DROP_TRANSFER_VALUE);
                    rollerDropRight.setPosition(0.5 - TRANSFER_DROP_TRANSFER_VALUE);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(0.63);

                } else {
                    linkage.setPosition(0.85);
                    transferAngle.setPosition(TRANSFER_ANGLE_TRANSFER_VALUE);
                }
                break;
        }
    }

    @Override
    public void shutdown() {

    }

    // TUNE
    public void setRoller(RollerState rollerState) {
        System.out.println("roller state set to: "+ rollerState.name());

        this.rollerState = rollerState;
    }

    // TUNE
    public TransferState getTransferState() {
        return transferState;
    }

    public void setTransferState(TransferState transferState) {
        this.transferState = transferState;
        transferTimer.reset();
    }

    public void toggleTransferState() {
        if (transferState == INTAKE) {
            setTransferState(TRANSFER);
        } else {
            setTransferState(INTAKE);
        }
    }

    public enum RollerState {
        OFF,
        INTAKE,
        OUTTAKE,
    }

    public enum TransferState {
        INTAKE,
        TRANSFER
    }
}