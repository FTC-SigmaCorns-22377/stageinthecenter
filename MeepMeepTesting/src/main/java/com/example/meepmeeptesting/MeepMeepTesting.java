package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;


enum Randomization {
    LEFT, CENTER, RIGHT
}

enum Team {
    BLUE, RED
}

enum Side {
    FRONT, BACK
}


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Randomization randomization = Randomization.RIGHT;
        Team team = Team.BLUE;
        Side side = Side.FRONT;

        // Robot Parameters
        final double robotLength = 17.008;
        final Double distToPixelSlot = (8.5 + 10 + 9.75) / 3;
        final Double distToRollerTip = 18.5;
        final Double distToBackdropBase = 1.5;

        // Calculated Field Parameters
        final Double startY = 71.25 - 0.5 * robotLength;
        final Double intakeX = -69.75 + distToRollerTip;
        final Double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

        // Dynamic Points
        Pose2d startPose = new Pose2d(0, 0, 0);
        Pose2d spikeMark = new Pose2d(0, 0, 0);
        Vector2d intakeJunction = new Vector2d(0, 0);
        Vector2d stack1 = new Vector2d(0, 0);
        Vector2d stack2 = new Vector2d(intakeX, 23.75);
        Vector2d outputJunction = new Vector2d(0, 0);
        List<Vector2d> backdropSlots = new ArrayList<>();
        int randomizationSlot = 0;
        int outputSlot1 = 0;
        int outputSlot2 = 0;
        int outputSlot3 = 0;



        switch (team) {
            case BLUE:
                for (int i = -5; i <= 5; i++) {
                    backdropSlots.add(new Vector2d(outputX, 35.625 + 1.5 * i));
                }
                switch (side) {
                    case FRONT:
                        startPose = new Pose2d(11.875, startY, -0.5 * Math.PI);
                        switch (randomization) {
                            case LEFT:
                                spikeMark = new Pose2d(22.75, 35.75 + distToPixelSlot, -0.5 * Math.PI);
                                intakeJunction = new Vector2d(-29.688, 11.875);
                                stack1 = new Vector2d(intakeX, 11.875);
                                outputJunction = new Vector2d(23.75, 11.875);
                                randomizationSlot = 8;
                                outputSlot1 = 5;
                                outputSlot2 = 1;
                                outputSlot3 = 0;
                                break;
                            default:
                                spikeMark = new Pose2d(11.875, 24.75 + distToPixelSlot, -0.5 * Math.PI);
                                intakeJunction = new Vector2d(-30, 35.625);
                                stack1 = new Vector2d(intakeX, 35.625);
                                outputJunction = new Vector2d(31.25, 35.625);
                                randomizationSlot = 4;
                                outputSlot1 = 9;
                                outputSlot2 = 1;
                                outputSlot3 = 0;
                                break;
                            case RIGHT:
                                spikeMark = new Pose2d(1 + distToPixelSlot, 28.125, Math.PI);
                                intakeJunction = new Vector2d(-29.688, 11.875);
                                stack1 = new Vector2d(intakeX, 11.875);
                                outputJunction = new Vector2d(23.75, 11.875);
                                randomizationSlot = 0;
                                outputSlot1 = 5;
                                outputSlot2 = 9;
                                outputSlot3 = 0;
                                break;
                        }
                        break;
                    case BACK:
                        startPose = new Pose2d(-35.625, startY, -0.5 * Math.PI);
                        switch (randomization) {
                            case LEFT:
                                break;
                            case CENTER:
                                break;
                            case RIGHT:
                                break;
                        }
                        break;
                }
                break;
            case RED:
                break;
        }

//        Trajectory purple = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
//                .lineToLinearHeading(spikeMark)
//                .build();
//
//        Trajectory yellow = robot.drivetrain.getBuilder().trajectoryBuilder(purple.end())
//                .back(1)
//                .splineTo(backdropSlots.get(randomizationSlot), 0)
//                .build();
//
//        Trajectory intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(yellow.end())
//                .forward(1)
//                .splineToConstantHeading(outputJunction, Math.PI)
//                .lineTo(stack1)
//                .build();
//
//        Trajectory output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
//                .lineTo(outputJunction)
//                .splineToConstantHeading(backdropSlots.get(outputSlot1), 0)
//                .build();
//
//        Trajectory intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
//                .forward(1)
//                .splineToConstantHeading(outputJunction, Math.PI)
//                .lineTo(stack1)
//                .build();
//
//        Trajectory output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
//                .lineTo(outputJunction)
//                .splineToConstantHeading(backdropSlots.get(outputSlot2), 0)
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
//        TrajectorySequence trajectorySequence = new TrajectorySequence(trajectories);
//
//        Command auto = followRR(purple);
//        auto.addNext(followRR(yellow));
//        auto.addNext(followRR(intake1));
//        auto.addNext(followRR(output1));
//        auto.addNext(followRR(intake2));
//        auto.addNext(followRR(output2));
//        auto.addNext(followRR(intake3));
//        auto.addNext(followRR(output3));
//
//        return auto;


        Pose2d finalStartPose = startPose;
        Pose2d finalSpikeMark = spikeMark;
        int finalRandomizationSlot = randomizationSlot;
        Vector2d finalOutputJunction = outputJunction;
        Vector2d finalStack = stack1;
        int finalOutputSlot = outputSlot1;
        int finalOutputSlot1 = outputSlot2;
        Vector2d finalIntakeJunction = intakeJunction;
        int finalOutputSlot2 = outputSlot3;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(finalStartPose)
                                .lineToLinearHeading(finalSpikeMark) // End of purple trajectory
                                .back(1)
                                .splineTo(backdropSlots.get(finalRandomizationSlot), 0) // End of yellow trajectory
                                .forward(1)
                                .splineToConstantHeading(finalOutputJunction, Math.PI)
                                .lineTo(finalStack) // End of intake1 trajectory
                                .lineTo(finalOutputJunction)
                                .splineToConstantHeading(backdropSlots.get(finalOutputSlot), 0) // End of output1 trajectory
                                .forward(1)
                                .splineToConstantHeading(finalOutputJunction, Math.PI)
                                .lineTo(finalStack) // End of intake2 trajectory
                                .lineTo(finalOutputJunction)
                                .splineToConstantHeading(backdropSlots.get(finalOutputSlot1), 0) // End of output2 trajectory
                                .forward(1)
                                .splineToConstantHeading(finalOutputJunction, Math.PI)
                                .lineTo(finalIntakeJunction)
                                .splineToConstantHeading(stack2, Math.PI) // End of intake3 trajectory
                                .back(1)
                                .splineToConstantHeading(finalIntakeJunction, 0)
                                .lineTo(finalOutputJunction)
                                .splineToConstantHeading(backdropSlots.get(finalOutputSlot2), 0) // End of output3 trajectory
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}