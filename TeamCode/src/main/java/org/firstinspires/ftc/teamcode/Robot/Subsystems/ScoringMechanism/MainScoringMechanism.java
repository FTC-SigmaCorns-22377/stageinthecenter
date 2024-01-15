package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class MainScoringMechanism extends Subsystem {

    public Drone drone = new Drone();
    public Hang hang = new Hang();
    public Intake intake = new Intake();
    public Slides slides = new Slides();
    public Output output = new Output();

    public void commonInit(HardwareMap hwMap) {
        drone.initAuto(hwMap);
        hang.initAuto(hwMap);
        intake.initAuto(hwMap);
        slides.initAuto(hwMap);
        output.initAuto(hwMap);
    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void initTeleop(HardwareMap hwMap) {
        commonInit(hwMap);
    }

    @Override
    public void periodic() {
        updateMechanisms();
    }

    @Override
    public void shutdown() {

    }

    private void updateMechanisms() {
        drone.periodic();
        hang.periodic();
        intake.periodic();
        slides.periodic();
        output.periodic();
    }
}