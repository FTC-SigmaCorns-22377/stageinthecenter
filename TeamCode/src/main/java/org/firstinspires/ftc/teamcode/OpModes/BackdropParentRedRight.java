package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

import java.util.ArrayList;
import java.util.List;

public class BackdropParentRedRight extends BaseAuto {

    // Robot Parameters
    public static double robotLength = 17.008;
    public static double distToRollerTip = 17.0;
    public static double distToBackdropBase = 0.75;

    // Calculated Field Parameters
    public static double startY = 71.25 - 0.5 * robotLength;
    public static double intakeX = -68 + distToRollerTip;
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
                intakeJunction = new Vector2d(-29.688, 8);
                stack1 = new Vector2d(intakeX, 8);
                outputJunction = new Vector2d(35.625, 8);
                switch (getRandomization()) {
                    case LEFT:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(23, 35.75 + 0.5 * robotLength);
                        spikeHeading = -0.5 * Math.PI;
                        randomizationSlot = 9;
                        outputSlot1 = 5;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    default:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(11.875, 24.75 + 0.5 * robotLength);
                        spikeHeading = -0.5 * Math.PI;
                        randomizationSlot = 5;
                        outputSlot1 = 9;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    case RIGHT:
                        spikeDelta = 5;
                        spikeMark = new Vector2d(0.5 * robotLength, 35.75);
                        spikeHeading = Math.PI;
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
                intakeJunction = new Vector2d(-29.688, -12);
                stack1 = new Vector2d(intakeX+4, -12);
                outputJunction = new Vector2d(35.625, -6.5);
                switch (getRandomization()) {
                    case RIGHT:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(21, -35.75 - 0.5 * robotLength);
                        spikeHeading = 0.5 * Math.PI;
                        randomizationSlot = 1;
                        outputSlot1 = 5;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    default:
                        spikeDelta = 1;
                        spikeMark = new Vector2d(11.875, -24.75 - 0.5 * robotLength);
                        spikeHeading = 0.5 * Math.PI;
                        randomizationSlot = 5;
                        outputSlot1 = 9;
                        outputSlot2 = 1;
                        outputSlot3 = 0;
                        break;
                    case LEFT:
                        spikeDelta = 5;
                        spikeMark = new Vector2d( 0.5 * robotLength+2, -35.75);
                        spikeHeading = Math.PI;
                        randomizationSlot = 9;
                        outputSlot1 = 5;
                        outputSlot2 = 9;
                        outputSlot3 = 0;
                        break;
                }
                switch (getPark()) {
                    default:
                        parkPos = new Vector2d(outputX, -19);
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

//        Trajectory yellow = robot.drivetrain.getBuilder().trajectoryBuilder(purple.end())
//                .back(1)
//                .splineTo(backdropSlots.get(randomizationSlot), 0)
//                .build();
//
//        Trajectory back0 = robot.drivetrain.getBuilder().trajectoryBuilder(yellow.end())
//                .forward(4)
//                .build();
//
//        Trajectory intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(back0.end())
//                .forward(1)
//                .splineToConstantHeading(outputJunction.minus(new Vector2d(0,-4)), Math.PI)
//                .splineToConstantHeading(stack1.plus(new Vector2d(-1, 1)), Math.PI)
//                .build();
//
//        Trajectory output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
//                .lineTo(outputJunction)
//                .splineToConstantHeading(backdropSlots.get(outputSlot1).minus(new Vector2d(16, 16)), 0)
//                .build();
//
//        Trajectory back1 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
//                .forward(6)
//                .build();

//        Trajectory intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(back1.end())
//                .forward(1)
//                .splineToConstantHeading(outputJunction, Math.PI)
//                .lineTo(stack1)
//                .build();

//        Trajectory output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
//                .lineTo(outputJunction)
//                .splineToConstantHeading(backdropSlots.get(outputSlot2), 0)
//                .build();
//
//        Trajectory back2 = robot.drivetrain.getBuilder().trajectoryBuilder(output2.end())
//                .forward(6)
//                .build();
//
//        Trajectory intake3 = robot.drivetrain.getBuilder().trajectoryBuilder(output2.end())
//                .forward(1)
//                .splineToConstantHeading(outputJunction, Math.PI)
//                .lineTo(intakeJunction)
//                .splineToConstantHeading(stack2, Math.PI)
//                .build();
//
//        Trajectory output3 = robot.drivetrain.getBuilder().trajectoryBuilder(intake3.end())
//                .back(1)
//                .splineToConstantHeading(intakeJunction, 0)
//                .lineTo(outputJunction)
//                .splineToConstantHeading(backdropSlots.get(outputSlot3), 0)
//                .build();
//
//        Trajectory driftOffset = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
//                .forward(5)
//                .build();
//
//        Trajectory park = robot.drivetrain.getBuilder().trajectoryBuilder(back1.end())
//                .splineTo(parkPos, Math.PI)
//                .build();

        Command auto = followRR(purple);
//        auto.addNext(commandGroups.scorePos());
//        auto.addNext(commandGroups.setSlides(Slides.SlideHeight.HALF));
//        auto.addNext(followRR(yellow));
//        auto.addNext(commandGroups.score());
//        auto.addNext(followRR(back0));
//        auto.addNext(commandGroups.postScore());
//        auto.addNext(followRR(intake1));
//        auto.addNext(followRR(driftOffset));
//        auto.addNext(commandGroups.postScore5());
//        auto.addNext(wait(1.0));
//        auto.addNext(commandGroups.rollerOn());
//        auto.addNext(wait(2.0));
//        auto.addNext(followRR(output1));
//        auto.addNext(commandGroups.newSetTransfer(Intake.TransferState.TRANSFER));
//        auto.addNext(commandGroups.rollerOff());
//        auto.addNext(wait(1.75));
//        auto.addNext(commandGroups.scorePos());
//        auto.addNext(commandGroups.setSlides(Slides.SlideHeight.L4));
//        auto.addNext(wait(1.0));
//        auto.addNext(commandGroups.score());
//        auto.addNext(followRR(back1));
//        auto.addNext(commandGroups.postScore());
      //  auto.addNext(followRR(intake2));
      //  auto.addNext(followRR(driftOffset));
      //  auto.addNext(commandGroups.postScore3());
      //  auto.addNext(wait(1.0));
      //  auto.addNext(commandGroups.rollerOn());
      //  auto.addNext(wait(2.0));
      //  auto.addNext(commandGroups.rollerOff());
      //  auto.addNext(commandGroups.newSetTransfer(Intake.TransferState.TRANSFER));
      //  auto.addNext(followRR(output2));
      //  auto.addNext(commandGroups.scorePos());
      //  auto.addNext(wait(1.0));
      //  auto.addNext(commandGroups.score());
      //  auto.addNext(followRR(back2));
      //  auto.addNext(followRR(park));

        return auto;
    }
}