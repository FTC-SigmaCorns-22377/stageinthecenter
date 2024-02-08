package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Gamepad;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

@Config
public class SlideOverride extends Command {

    public static double SCALAR = 2;
    Slides slides;
    Gamepad gamepad2;
    double strafe_dead_band = 0.1;

    public SlideOverride(Robot robot, Gamepad gamepad2) {
        super(robot.drivetrain, gamepad2);
        this.slides = robot.scoringMechanism.slides;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void init() {

    }

    @Override
    public void periodic() {
        double amount = gamepad2.getForwardJoystick();
        Dashboard.packet.put("gamepad2 val", amount);
        slides.setSlideOverrideModifier(amount * SCALAR);
    }

    @Override
    public boolean completed() {
        return false;
    }

    @Override
    public void shutdown() {
//        slides.setSlideOverrideModifier(0);
    }
}