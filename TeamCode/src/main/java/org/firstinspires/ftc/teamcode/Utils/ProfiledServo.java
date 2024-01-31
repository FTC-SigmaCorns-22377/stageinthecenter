package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Math.AsymmetricProfile.MotionConstraint;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;

public class ProfiledServo {
    public Servo servo;
    protected double endPosition;
    protected double previousEndPosition;
    protected double currentPosition;
    protected String name;

    public AsymmetricMotionProfile profile_m;
    public MotionConstraint forwardConstraint;
    public MotionConstraint backwardContraint;
    public ElapsedTime timer = new ElapsedTime();

    public ProfiledServo(HardwareMap hwmap, String name, double Forwardvelo, double Forwardaccel, double Forwarddecel, double BackwardVelo, double BackwardAccel, double backwawrdDecel, double initialPosition) {
        servo = hwmap.get(Servo.class, name);
        this.name = name;
        this.endPosition = initialPosition;
        this.currentPosition = initialPosition;
        this.previousEndPosition = initialPosition + 100; // just guarantee that they are not equal
        this.forwardConstraint = new MotionConstraint(Forwardvelo,Forwardaccel,Forwarddecel);
        this.backwardContraint = new MotionConstraint(BackwardVelo,BackwardAccel, backwawrdDecel);
        setPosition(initialPosition);
    }

    protected void regenerate_profile() {
        profile_m = new AsymmetricMotionProfile(this.currentPosition,this.endPosition,this.forwardConstraint);
        timer.reset();
    }


    public void periodic() {
        if (endPosition != previousEndPosition) {
            regenerate_profile();
        }
        previousEndPosition = endPosition;
        double current_target = profile_m.calculate(timer.seconds()).getX();
        servo.setPosition(current_target);
        Dashboard.packet.put(name + "profiled_servo: ", current_target);
    }

    public boolean isBusy() {
        return timer.seconds() < profile_m.getProfileDuration();
    }


    public void setPosition(double endPosition) {
        this.endPosition = endPosition;
    }
}