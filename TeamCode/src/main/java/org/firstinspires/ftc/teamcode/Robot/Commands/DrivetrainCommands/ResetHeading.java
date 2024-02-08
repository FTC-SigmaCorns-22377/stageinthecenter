package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class ResetHeading extends Command {
    Drivetrain drivetrain;

    public ResetHeading(Robot robot) {
        super(robot.drivetrain);
        this.drivetrain = robot.drivetrain;
    }

    @Override
    public void init() {
        Pose2d currentPose = drivetrain.drive.getPoseEstimate();
        Pose2d newPose = new Pose2d(currentPose.getX(), currentPose.getY(), 0);
        drivetrain.drive.setPoseEstimate(newPose);
    }

    @Override
    public void periodic() {}

    @Override
    public boolean completed() {
        return true;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }
}