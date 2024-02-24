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
        MeepMeep meepMeep = new MeepMeep(600);
        final Pose2d startPose = new Pose2d(-47.5 + 0.5 * 14.348, -71.25 + 0.5 * 17.008, 0.5 * Math.PI);
        final double startX = -47.5 + 0.5 * 14.348;
        final double startY = -71.25 + 0.5 * 17.008;
        final double robotLength = 17.008;
        final double outputX = 60 - 2 - 0.5 * 17.008;
        final double parkX = 55;
        final double parkY = -10;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14.348, 17.008)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .lineTo(new Vector2d(-41.625, -24.75 - 0.5 * robotLength))
                                .back(1)
                                .splineTo(new Vector2d(-30, -57.5), 0)
                                .lineTo(new Vector2d(23.75, -57.5))
                                .splineToConstantHeading(new Vector2d(outputX, -35.625 + 3), 0)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}