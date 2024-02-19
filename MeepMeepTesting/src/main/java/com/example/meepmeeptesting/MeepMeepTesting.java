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

        final Pose2d finalStartPose = startPose;
        MeepMeep meepMeep = new MeepMeep(600);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(0.75 * 73.17330064499293, 0.75 * 73.17330064499293, Math.toRadians(356.8103234042553), Math.toRadians(356.8103234042553), 15)
                .setDimensions(14.348, 17.008)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(finalStartPose)
                                .lineToSplineHeading(new Pose2d(1 + 0.5 * robotLength, 42.5, Math.PI))
                                .splineToConstantHeading(new Vector2d(outputX - 5, 35.625 - 4.5), 0)
                                .splineToConstantHeading(new Vector2d(23.75, 11.875), Math.PI)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}