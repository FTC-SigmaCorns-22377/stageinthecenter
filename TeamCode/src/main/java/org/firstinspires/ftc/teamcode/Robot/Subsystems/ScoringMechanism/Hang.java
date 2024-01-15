package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Hang extends Subsystem {
    DcMotorEx hang;

    public void commonInit(HardwareMap hwMap) {
        hang = hwMap.get(DcMotorEx.class, "hang");
        hang.setPower(0);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    public void setHang(Double power) {
        hang.setPower(power);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }
}