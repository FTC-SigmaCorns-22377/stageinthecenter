package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

@Autonomous
public class TestingAuto extends BaseAuto {
    Pose2d startPose = new Pose2d(12, 59, Math.toRadians(-90));

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }


    @Override
    public Command setupAuto(CommandScheduler scheduler) {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);


        Trajectory center = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .forward(28)
                .build();

        Trajectory toBackboard = robot.drivetrain.getBuilder().trajectoryBuilder(center.end())
                .back(1)
                .splineTo(new Vector2d(48, 35), Math.toRadians(0))
                .build();

        Trajectory cycleForward = robot.drivetrain.getBuilder().trajectoryBuilder(toBackboard.end())
                .forward(3)
                .splineToConstantHeading(new Vector2d(30, 0), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, 0), Math.toRadians(180))
                .build();
        Trajectory cycleBack = robot.drivetrain.getBuilder().trajectoryBuilder(cycleForward.end())
                .splineToConstantHeading(new Vector2d(30, 0), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(46, 35), Math.toRadians(0))
                .build();

        Trajectory recycleForward = robot.drivetrain.getBuilder().trajectoryBuilder(cycleBack.end())
                .forward(3)
                .splineToConstantHeading(new Vector2d(30, 0), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-53, 0), Math.toRadians(180))
                .build();



        Command auto = followRR(center);
        auto.addNext(followRR(toBackboard));
        auto.addNext(commandGroups.scorePos());
        auto.addNext(wait(0.5));
        auto.addNext(commandGroups.score());
        auto.addNext(commandGroups.postScore());
        auto.addNext(followRR(cycleForward));
        auto.addNext(commandGroups.setClaw(Output.ClawState.OPEN));
        auto.addNext(commandGroups.postScore());
        auto.addNext(commandGroups.rollerOn());
        auto.addNext(wait(2.0));
        auto.addNext(commandGroups.rollerOff());
        auto.addNext(commandGroups.transferPos());
        auto.addNext(followRR(cycleBack));
        auto.addNext(commandGroups.scorePos());
        auto.addNext(wait(0.5));
        auto.addNext(commandGroups.score());
        auto.addNext(commandGroups.postScore());
        auto.addNext(followRR(recycleForward));
        auto.addNext(commandGroups.setClaw(Output.ClawState.OPEN));
        auto.addNext(commandGroups.postScore());
        auto.addNext(commandGroups.rollerOn());
        auto.addNext(wait(2.0));
        auto.addNext(commandGroups.rollerOff());
        auto.addNext(commandGroups.transferPos());
        auto.addNext(followRR(cycleBack));
        auto.addNext(commandGroups.scorePos());
        auto.addNext(wait(0.5));
        auto.addNext(commandGroups.score());

        return auto;
    }
}
