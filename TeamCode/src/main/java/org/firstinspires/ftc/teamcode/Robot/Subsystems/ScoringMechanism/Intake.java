package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.TransferState.INTAKE;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.TransferState.TRANSFER;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake.RollerState.OFF;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utils.ProfiledServo;


import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

@Config
public class Intake extends Subsystem {
    public static double INTAKE_POWER = 0.7;
    public static double TRANSFER_DROP_INTAKE_VALUE_LEFT = 0.03;
    public static double TRANSFER_DROP_INTAKE_VALUE_RIGHT = 0.009;

    public static double TRANSFER_DROP_TRANSFER_VALUE_LEFT = -0.25;
    public static double TRANSFER_DROP_TRANSFER_VALUE_RIGHT = -0.275;


    public static double TRANSFER_ANGLE_INTAKE_VALUE = 1;
    public static double TRANSFER_ANGLE_TRANSFER_VALUE = 0.215;
    public static double TRANSFER_FIVE_DROP_VALUE_LEFT = -0.03;
    public static double TRANSFER_FIVE_DROP_VALUE_RIGHT = -0.05;

    public static double TRANSFER_FIVE_ANGLE_VALUE = 0.92;
    public static double TRANSFER_THREE_DROP_VALUE_LEFT = -0.16;
    public static double TRANSFER_THREE_DROP_VALUE_RIGHT = 0.16;

    public static double TRANSFER_THREE_ANGLE_VALUE = 0.97;
    public static double TRANSFER_ANGLE_TRAVEL_VALUE = 0.1;




    public static double LINKAGE_FINAL_VALUE_TRANSFER = 0.615;

    public static double ARM_IN_COLLECT = 0;
    DcMotorEx roller;
    Servo rollerDropLeft;
    Servo rollerDropRight;
//    ProfiledServo transferAngle;
    Servo transferAngle;
    Servo linkage;

    RollerState rollerState;
    TransferState transferState;
    ElapsedTime transferTimer = new ElapsedTime();

    public void initCommon(HardwareMap hwMap) {
        roller = hwMap.get(DcMotorEx.class, "roller");
        rollerDropLeft = hwMap.get(Servo.class, "rollerDropLeft");
        rollerDropRight = hwMap.get(Servo.class, "rollerDropRight");
        double velocityForward = 0.8; //percent/s
        double accelForward = 1; // percent/s^2

        double velocityBackward = 0.5;
        double accelBackward = 0.7;

//        transferAngle = new ProfiledServo(hwMap, "holder",velocityForward,accelForward * 1.5,accelForward / 2,velocityBackward,accelBackward * 1.5,accelBackward / 2,ARM_IN_COLLECT);
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
                if (transferTimer.seconds() > 0.25) {
                    linkage.setPosition(0.95);
                    rollerDropLeft.setPosition(0.5 + TRANSFER_DROP_INTAKE_VALUE_LEFT);
                    rollerDropRight.setPosition(0.5 - TRANSFER_DROP_INTAKE_VALUE_RIGHT);
                } else if (transferTimer.seconds() > 0.15) {
                    linkage.setPosition(0.85);
                } else {
                    transferAngle.setPosition(TRANSFER_ANGLE_INTAKE_VALUE);
                }
                break;
            case TRANSFER:
                if (transferTimer.seconds() > 0.2) {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_DROP_TRANSFER_VALUE_LEFT);
                    rollerDropRight.setPosition(0.5 - TRANSFER_DROP_TRANSFER_VALUE_RIGHT);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(LINKAGE_FINAL_VALUE_TRANSFER);

                } else {
                    linkage.setPosition(0.85);
                    transferAngle.setPosition(TRANSFER_ANGLE_TRANSFER_VALUE);
                }
                break;
            case FIVE:
                if (transferTimer.seconds() > 0.2) {
                    linkage.setPosition(0.95);
                    transferAngle.setPosition(TRANSFER_FIVE_ANGLE_VALUE);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(0.85);
                } else {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_FIVE_DROP_VALUE_LEFT);
                    rollerDropRight.setPosition(0.5 - TRANSFER_FIVE_DROP_VALUE_RIGHT);
                }
                break;

            case THREE:
                if (transferTimer.seconds() > 0.2) {
                    linkage.setPosition(0.95);
                    transferAngle.setPosition(TRANSFER_THREE_ANGLE_VALUE);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(0.85);
                } else {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_THREE_DROP_VALUE_LEFT);
                    rollerDropRight.setPosition(0.5 - TRANSFER_THREE_DROP_VALUE_RIGHT);
                }
                break;
            case TRAVEL:
                if (transferTimer.seconds() > 0.2) {
                    rollerDropLeft.setPosition(0.5 + TRANSFER_DROP_TRANSFER_VALUE_LEFT);
                    rollerDropRight.setPosition(0.5 - TRANSFER_DROP_TRANSFER_VALUE_RIGHT);
                } else if (transferTimer.seconds() > 0.1) {
                    linkage.setPosition(0.8);

                } else {
                    linkage.setPosition(0.8);
                    transferAngle.setPosition(TRANSFER_ANGLE_TRAVEL_VALUE);
                }
                break;
        }
        updateProfiledServos();
    }

    protected void updateProfiledServos() {
//        transferAngle.periodic();
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
        TRANSFER,
        THREE,
        FIVE,
        TRAVEL
    }
}