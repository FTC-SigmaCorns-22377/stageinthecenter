package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

import java.util.ArrayList;
import java.util.List;

public class BackdropParent extends BaseAuto {

    // Robot Parameters
    public static double robotLength = 17.008;
    public static double distToRollerTip = 17.0;
    public static double distToBackdropBase = 0.75;

    // Calculated Field Parameters
    public static double startY = 71.25 - 0.5 * robotLength;
    public static double intakeX = -69.75 + distToRollerTip;
    public static double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

    // Dynamic Points
    Pose2d startPose = new Pose2d(0, 0, 0);
    int spikeDelta = 0;
    Vector2d spikeMark = new Vector2d(0, 0);
    double spikeHeading = 0;
    Vector2d intakeJunction = new Vector2d(0, 0);
    Vector2d stack1 = new Vector2d(0, 0);
    Vector2d stack2 = new Vector2d(0, 0);
    Vector2d outputJunction = new Vector2d(0, 0);
    List<Vector2d> backdropSlots = new ArrayList<>();
    int randomizationSlot = 0;
    int outputSlot1 = 0;
    int outputSlot2 = 0;
    int outputSlot3 = 0;
    Vector2d parkPos = new Vector2d(0, 0);

    @Override
    public void setRobotPosition() {
        switch (getTeam()) {
            default:
                startPose = new Pose2d(11.875, startY, -0.5 * Math.PI);
                break;
            case RED:
                startPose = new Pose2d(11.875, -startY, 0.5 * Math.PI);
                break;
        }
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        switch (getTeam()) {
            case BLUE:
                for (int i = -5; i <= 5; i++) {
                    backdropSlots.add(new Vector2d(outputX, 35.625 + 1.5 * i));
                }
                switch (getRandomization()) {
                    case LEFT:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(25, 35.75 + 0.5 * robotLength);
                        spikeHeading = -0.5 * Math.PI;
                        intakeJunction = new Vector2d(-29.688, 5.3125);
                        stack1 = new Vector2d(intakeX, 5.3125);
                        outputJunction = new Vector2d(35.625, 5.3125);
                        randomizationSlot = 9;
                        outputSlot1 = 5;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    default:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(11.875, 24.75 + 0.5 * robotLength);
                        spikeHeading = -0.5 * Math.PI;
                        intakeJunction = new Vector2d(-35.625, 35.625);
                        stack1 = new Vector2d(intakeX, 35.625);
                        stack2 = new Vector2d(intakeX, 15);
                        outputJunction = new Vector2d(11.875, 35.625);
                        randomizationSlot = 5;
                        outputSlot1 = 9;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    case RIGHT:
                        spikeDelta = 5;
                        spikeMark = new Vector2d(1 + 0.5 * robotLength, 35.75);
                        spikeHeading = Math.PI;
                        intakeJunction = new Vector2d(-29.688, 11.875);
                        stack1 = new Vector2d(intakeX, 11.875);
                        outputJunction = new Vector2d(23.75, 11.875);
                        randomizationSlot = 1;
                        outputSlot1 = 5;
                        outputSlot2 = 9;
                        outputSlot3 = 0;
                        break;
                }
                switch (getPark()) {
                    default:
                        parkPos = new Vector2d(outputX, 10);
                        break;
                    case EDGE:
                        parkPos = new Vector2d(outputX, 59.375);
                        break;
                }
                break;
            case RED:
                for (int i = -5; i <= 5; i++) {
                    backdropSlots.add(new Vector2d(outputX, -35.625 + 1.5 * i));
                }
                switch (getRandomization()) {
                    case RIGHT:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(25, -35.75 - 0.5 * robotLength);
                        spikeHeading = 0.5 * Math.PI;
                        intakeJunction = new Vector2d(-29.688, -5.3125);
                        stack1 = new Vector2d(intakeX, -5.3125);
                        outputJunction = new Vector2d(35.625, -5.3125);
                        randomizationSlot = 1;
                        outputSlot1 = 5;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    default:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(11.875, -24.75 - 0.5 * robotLength);
                        spikeHeading = 0.5 * Math.PI;
                        intakeJunction = new Vector2d(-35.625, -35.625);
                        stack1 = new Vector2d(intakeX, -35.625);
                        stack2 = new Vector2d(intakeX, -15);
                        outputJunction = new Vector2d(11.875, -35.625);
                        randomizationSlot = 5;
                        outputSlot1 = 9;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    case LEFT:
                        spikeDelta = 5;
                        spikeMark = new Vector2d(1 + 0.5 * robotLength, -35.75);
                        spikeHeading = Math.PI;
                        intakeJunction = new Vector2d(-29.688, -11.875);
                        stack1 = new Vector2d(intakeX, -11.875);
                        outputJunction = new Vector2d(23.75, -11.875);
                        randomizationSlot = 9;
                        outputSlot1 = 5;
                        outputSlot2 = 9;
                        outputSlot3 = 0;
                        break;
                }
                switch (getPark()) {
                    default:
                        parkPos = new Vector2d(outputX, -10);
                        break;
                    case EDGE:
                        parkPos = new Vector2d(outputX, -59.375);
                        break;
                }
                break;
        }

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Trajectory purple = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .forward(spikeDelta)
                .splineTo(spikeMark, spikeHeading)
                .build();

        Trajectory yellow = robot.drivetrain.getBuilder().trajectoryBuilder(purple.end())
                .back(1)
                .splineTo(backdropSlots.get(randomizationSlot), 0)
                .build();

        Trajectory intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(yellow.end())
                .forward(1)
                .splineToConstantHeading(outputJunction, Math.PI)
                .lineTo(stack1)
                .build();

        Trajectory output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
                .lineTo(outputJunction)
                .splineToConstantHeading(backdropSlots.get(outputSlot1), 0)
                .build();

        Trajectory intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
                .forward(1)
                .splineToConstantHeading(outputJunction, Math.PI)
                .lineTo(stack1)
                .build();

        Trajectory output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
                .lineTo(outputJunction)
                .splineToConstantHeading(backdropSlots.get(outputSlot2), 0)
                .build();

        Trajectory intake3 = robot.drivetrain.getBuilder().trajectoryBuilder(output2.end())
                .forward(1)
                .splineToConstantHeading(outputJunction, Math.PI)
                .lineTo(intakeJunction)
                .splineToConstantHeading(stack2, Math.PI)
                .build();

        Trajectory output3 = robot.drivetrain.getBuilder().trajectoryBuilder(intake3.end())
                .back(1)
                .splineToConstantHeading(intakeJunction, 0)
                .lineTo(outputJunction)
                .splineToConstantHeading(backdropSlots.get(outputSlot3), 0)
                .build();

        Trajectory back = robot.drivetrain.getBuilder().trajectoryBuilder(yellow.end())
                .forward(3)
                .build();

        Trajectory park = robot.drivetrain.getBuilder().trajectoryBuilder(back.end())
                .lineTo(parkPos)
                .build();

        Command auto = followRR(purple);
        auto.addNext(new MultipleCommand(followRR(yellow),
                new DelayedCommand(0.5, new MultipleCommand(
                        commandGroups.setArm(Output.ArmState.SCORE),
                        commandGroups.setSlides(Slides.SlideHeight.L0)
                ))));
        auto.addNext(commandGroups.score());
        auto.addNext(followRR(back));
        auto.addNext(commandGroups.postScore());
        auto.addNext(followRR(park));

        return auto;
    }
}