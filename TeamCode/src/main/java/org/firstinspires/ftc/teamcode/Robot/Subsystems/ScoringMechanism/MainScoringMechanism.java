package org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

import java.util.ArrayList;

public class MainScoringMechanism extends Subsystem {

//    public Drone drone = new Drone();
    public Intake intake = new Intake();
    public Slides slides = new Slides();
    public Output output = new Output();
    public Hang hang = new Hang();
    public ArrayList<Subsystem> subsystems = new ArrayList<>(List.of(intake, slides, output, hang));

    public void commonInit(HardwareMap hwMap) {
//        drone.initAuto(hwMap);
        for (Subsystem subsystem : subsystems)
            subsystem.initAuto(hwMap);
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
        for (Subsystem subsystem : subsystems)
            subsystem.periodic();
    }

    @Override
    public void shutdown() {

    }

}