package org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

public class FieldRelative extends Command {
    Drivetrain drivetrain;
    Gamepad game_pad1;
    double strafe_dead_band = 0.1;

    public FieldRelative(Robot robot, Gamepad game_pad1) {
        super(robot.drivetrain, game_pad1);
        this.drivetrain = robot.drivetrain;
        this.game_pad1 = game_pad1;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {


        double scalar = 1;

        double x;
        double y;
        double turn;
        y = game_pad1.getStrafeJoystick();
        x = game_pad1.getForwardJoystick();
        turn = game_pad1.getTurnJoystick();

        double heading = -drivetrain.getPose().getHeading();

        y = MathUtils.applyDeadBand(y, strafe_dead_band);

        Vector2d input = new Vector2d(x, y).rotated(heading);



        Pose2d powers = new Pose2d(input.getX() * scalar, input.getY() * scalar, turn * scalar);


        drivetrain.applyPowers(powers);


    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
        drivetrain.shutdown();
    }
}