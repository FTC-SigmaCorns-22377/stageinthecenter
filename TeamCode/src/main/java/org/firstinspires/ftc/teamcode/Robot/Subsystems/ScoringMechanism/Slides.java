package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Slides extends Subsystem {
    DcMotorEx slides1;
    DcMotorEx slides2;

    public void commonInit(HardwareMap hwMap) {
        slides1 = hwMap.get(DcMotorEx.class, "slides1");
        slides2 = hwMap.get(DcMotorEx.class, "slides2");
        slides2.setDirection(DcMotorSimple.Direction.REVERSE); // CHECK
        slides1.setPower(0);
        slides2.setPower(0);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlides(int time) throws InterruptedException {
        slides1.setPower(0.5);
        slides2.setPower(0.5);
        Thread.sleep(time);
        slides1.setPower(0);
        slides2.setPower(0);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }
}