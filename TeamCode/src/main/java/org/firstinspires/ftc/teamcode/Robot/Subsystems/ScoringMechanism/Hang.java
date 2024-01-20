package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Hang extends Subsystem {
    DcMotorEx hangMotor;

    private Hang.Hanging hanging = Hang.Hanging.DOWN;

    public void commonInit(HardwareMap hwMap) {
        hangMotor = hwMap.get(DcMotorEx.class, "hang");
        hangMotor.setPower(0);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
    }

//    public void setHang(Hanging hanging) {
//        this.hanging = hanging;
//        //hangMotor.setPower(power);
//    }

    @Override
    public void periodic() {
        updateTarget();
    }

    @Override
    public void shutdown() {
        hangMotor.setPower(0);
    }

    private void updateTarget(){
        switch (hanging) {
            case UP:
                //TODO: Check if this is the right direction
                hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                hangMotor.setPower(0.5);
                break;
            case DOWN:
                hangMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                hangMotor.setPower(0.5);
                break;
        }
    }
    public void setHanging(Hang.Hanging hanging) { this.hanging = hanging; }

    public enum Hanging {
        UP,
        DOWN
    }
}