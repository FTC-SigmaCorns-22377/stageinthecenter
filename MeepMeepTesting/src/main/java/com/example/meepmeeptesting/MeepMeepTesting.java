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

public class MeepMeepTesting {
    public static void main(String[] args) {
        // Robot Parameters
        final double robotLength = 17.008;
        final double robotWidth = 14.358;
        final double distToRollerTip = 17;
        final double distToBackdropBase = 1.5;

        // Calculated Field Parameters
        final double intakeX = -69.75 + distToRollerTip;
        final double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

        // Dynamic Field Parameters
        double startX = 0;
        double startY = 0;
        double startHeading = 0;
        Pose2d startPose = null;
        double startDelay = 0;
        Trajectory randomization = null;
        Vector2d scoreVec = null;
        double parkY = 11.875;

        startX = 23.75 - 0.5 * robotWidth;
        startY = 71.25 - 0.5 * robotLength;
        startHeading = -0.5 * Math.PI;
        startPose = new Pose2d(startX, startY, startHeading);
        final Vector2d outputJunction = new Vector2d(35.625, 35.625);
        final Vector2d stack1 = new Vector2d(intakeX, 35.625);
        final Vector2d intakeJunction = new Vector2d(-35.625, 35.625);
        final Vector2d stack2 = new Vector2d(intakeX, 23.75);

        final Pose2d finalStartPose = startPose;
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.75 * 73.17330064499293, 0.75 * 73.17330064499293, Math.toRadians(356.8103234042553), Math.toRadians(356.8103234042553), 15)
                .setDimensions(14.348, 17.008)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(finalStartPose)
                                .lineToSplineHeading(new Pose2d(15 + 0.5 * robotLength, 29.5, Math.PI))
                                .splineToConstantHeading(new Vector2d(25 + 0.5 * robotLength, 24.75), 0)
                                .splineToConstantHeading(new Vector2d(outputX, 35.625 - 4), 0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(stack1)

                                .lineTo(outputJunction)
                                .splineToConstantHeading(new Vector2d(outputX, 35.625 - 4), -0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(stack1)

                                .lineTo(outputJunction)
                                .splineToConstantHeading(new Vector2d(outputX, 35.625 - 4), -0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(intakeJunction)
                                .splineToConstantHeading(stack2, -0.5 * Math.PI)

                                .setTangent(0.5 * Math.PI)
                                .splineToConstantHeading(intakeJunction, 0)
                                .lineTo(outputJunction)
                                .splineToConstantHeading(new Vector2d(outputX, 35.625 - 4), -0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(intakeJunction)
                                .splineToConstantHeading(stack2, -0.5 * Math.PI)

                                .setTangent(0.5 * Math.PI)
                                .splineToConstantHeading(intakeJunction, 0)
                                .lineTo(outputJunction)
                                .splineToConstantHeading(new Vector2d(outputX, 35.625 - 4), -0.5 * Math.PI)
                                .splineToConstantHeading(new Vector2d(55, parkY), 0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}