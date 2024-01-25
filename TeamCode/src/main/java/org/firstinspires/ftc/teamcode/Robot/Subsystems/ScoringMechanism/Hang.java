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

public class Hang extends Subsystem {
    public static double HANG_POWER = 0.9;
    DcMotorEx hang;

    HangState hangState;

    public void initCommon(HardwareMap hwMap) {
        hang = hwMap.get(DcMotorEx.class, "hang");

        hang.setDirection(DcMotorSimple.Direction.REVERSE);

        setHang(HangState.OFF);

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
        switch (hangState) {
            case OFF:
                hang.setPower(0);
                break;
            case INTAKE:
                hang.setPower(HANG_POWER);
                break;
            case OUTTAKE:
                hang.setPower(-HANG_POWER*0.8);
                break;
        }

    }

    @Override
    public void shutdown() {

    }

    // TUNE
    public void setHang(HangState hangState) {
        this.hangState = hangState;
    }


    public enum HangState {
        OFF,
        INTAKE,
        OUTTAKE,
    }


}