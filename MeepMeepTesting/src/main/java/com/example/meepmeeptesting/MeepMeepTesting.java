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
        final double parkX = 55;

        // Dynamic Field Parameters
        final double startX = 23.75 - 0.5 * robotWidth;
        final double startY = 71.25 - 0.5 * robotLength;
        final double startHeading = -0.5 * Math.PI;
        final Pose2d startPose = new Pose2d(startX, startY, startHeading);
        double startDelay = 0;
        Vector2d stack2 = new Vector2d(intakeX, 23.75);
        double scoreDelay0 = 0;
        double scoreDelay1 = 0;
        double scoreDelay3 = 0;
        double parkY = 11.875;
        final Vector2d stack1 = new Vector2d(intakeX, 35.625);
        final Vector2d intakeJunction = new Vector2d(-35.625, 35.625);
        final Vector2d outputJunction = new Vector2d(35.625, 35.625);
        scoreDelay1 = 2.6;
        scoreDelay3 = 3;
        final Vector2d scoreVec = new Vector2d(outputX, 35.625 - 4);
        final Vector2d scoreOffsetVec = new Vector2d(5, -1.5);

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.75 * 73.17330064499293, 0.75 * 73.17330064499293, Math.toRadians(356.8103234042553), Math.toRadians(356.8103234042553), 15)
                .setDimensions(14.348, 17.008)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(0.5 * (startX + 15 + 0.5 * robotLength), 0.5 * (startY + 29.5)))
                                .lineToSplineHeading(new Pose2d(15 + 0.5 * robotLength, 29.5, Math.PI))
                                .splineToConstantHeading(new Vector2d(25 + 0.5 * robotLength, 24.75), 0)
                                .splineToConstantHeading(scoreVec, 0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(stack1)

                                .lineTo(outputJunction)
                                .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(stack1)

                                .lineTo(outputJunction)
                                .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                .splineToConstantHeading(outputJunction, Math.PI)
                                .lineTo(intakeJunction)
                                .splineToConstantHeading(stack2, -0.5 * Math.PI)

                                .strafeRight(1)
                                .splineToConstantHeading(intakeJunction, 0)
                                .lineTo(outputJunction)
                                .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}