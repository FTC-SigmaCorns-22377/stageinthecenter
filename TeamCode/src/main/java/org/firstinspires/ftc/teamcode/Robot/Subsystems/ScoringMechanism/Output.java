package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.ArmState.TRANSFER;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.WristState.DEG0;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.WristState.DEG180;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.WristState.DEG240;
import static org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output.WristState.DEG60;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

@Config
public class Output extends Subsystem {

    ArmState armState = TRANSFER;
    WristState wristState = DEG0;
    ClawState clawPurpleState = ClawState.OPEN;
    ClawState clawBlackState = ClawState.OPEN;
    Servo armLeft;
    Servo armRight;
    Servo wrist;
    Servo clawPurple;
    Servo clawBlack;
    ElapsedTime scoreTime = new ElapsedTime();


    public static double WRIST_ZERO = 0.73;
    public static double WRIST_ONE = 0.52;
    public static double WRIST_TWO = 0.9;
    public static double WRIST_THREE = 0.17;


    public static double ARM_SCORE_VALUE = 0.77;
    public static double ARM_TRANSFER_VALUE = 0.245;
    public static double ARM_TRAVEL_VALUE = 0.2;

    public static double ARM_POST_VALUE = 0.78;
    public static double LEFT_OFFSET = 0.006;

    public void initCommon(HardwareMap hwMap) {
        armLeft = hwMap.get(Servo.class, "armLeft");
        armRight = hwMap.get(Servo.class, "armRight");
        wrist = hwMap.get(Servo.class, "wrist");
        clawPurple = hwMap.get(Servo.class, "clawPurple");
        clawBlack = hwMap.get(Servo.class, "clawBlack");
        setWrist(DEG0);
        setArmState(TRANSFER);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        initCommon(hwMap);
        setClawPurple(ClawState.CLOSED);
        setClawBlack(ClawState.CLOSED);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        initCommon(hwMap);
        setClawPurple(ClawState.OPEN);
        setClawBlack(ClawState.OPEN);
    }

    @Override
    public void periodic() {}


    @Override
    public void shutdown() {

    }

    // TUNE
    public void setArmState(ArmState armState) {
        this.armState = armState;
        switch (armState) {
            case TRANSFER:
                armLeft.setPosition(ARM_TRANSFER_VALUE+LEFT_OFFSET);
                armRight.setPosition(1-ARM_TRANSFER_VALUE);
                break;
            case SCORE:
                armLeft.setPosition(ARM_SCORE_VALUE+LEFT_OFFSET);
                armRight.setPosition(1-ARM_SCORE_VALUE);
                break;
            case POST:
                if (scoreTime.seconds() > 0.4) {
                    armLeft.setPosition(ARM_POST_VALUE+LEFT_OFFSET);
                    armRight.setPosition(1-ARM_POST_VALUE);
                    break;
                }
            case TRAVEL:
                armLeft.setPosition(ARM_TRAVEL_VALUE+LEFT_OFFSET);
                armRight.setPosition(1-ARM_TRAVEL_VALUE);
                break;
        }


    }

    public ArmState getArmState(){
        return this.armState;
    }
    // TUNE
    public void setWrist(WristState wristState) {
        this.wristState = wristState;
        switch (wristState) {
            case DEG0:
                wrist.setPosition(WRIST_ZERO);
                break;
            case DEG60:
                wrist.setPosition(WRIST_ONE);
                break;
            case DEG120:
                wrist.setPosition(WRIST_TWO);
                break;
            case DEG180:
                wrist.setPosition(WRIST_THREE);
                break;
            case DEG240:
                wrist.setPosition(WRIST_ZERO - 4./6);
                break;
            case DEG300:
                wrist.setPosition(WRIST_ZERO - 5./6);
                break;
            case DEG360:
                wrist.setPosition(0);
                break;

        }
    }

    // TUNE
    public void setClawPurple(ClawState clawPurpleState) {
        this.clawPurpleState = clawPurpleState;
        switch (clawPurpleState) {
            case OPEN:
                clawPurple.setPosition(0.732);
                break;
            case CLOSED:
                clawPurple.setPosition(0.8);
                break;
            case BOTHPOSTSCORE:
                clawPurple.setPosition(0.68);
                break;
            case PURPLEPOSTSCORE:
                clawPurple.setPosition(0.68);
        }
    }

    // TUNE
    public void setClawBlack(ClawState clawBlackState) {
        this.clawBlackState = clawBlackState;
        switch (clawBlackState) {
            case OPEN:
                clawBlack.setPosition(0.269);
                break;
            case CLOSED:
                clawBlack.setPosition(0.225);
                break;
            case BOTHPOSTSCORE:
                clawBlack.setPosition(0.292);
                break;
            case BLACKPOSTSCORE:
                clawBlack.setPosition(0.292);
        }
    }

    public enum ArmState {
        TRANSFER,
        SCORE,
        POST,
        TRAVEL
    }

    public enum WristState {
        DEG0,
        DEG60,
        DEG120,
        DEG180,
        DEG240,
        DEG300,
        DEG360
    }

    public enum ClawState {
        OPEN,
        CLOSED,
        BOTHPOSTSCORE,
        PURPLEPOSTSCORE,
        BLACKPOSTSCORE
    }

}