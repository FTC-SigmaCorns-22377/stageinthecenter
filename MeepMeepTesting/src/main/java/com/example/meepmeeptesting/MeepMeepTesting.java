package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


enum Randomization {
    LEFT, CENTER, RIGHT
}
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Randomization randomization = Randomization.CENTER;





        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 59, Math.toRadians(-90)))
//                                center
                                .forward(25)
                                .back(1)
                                .splineTo(new Vector2d(50, 35), Math.toRadians(0))


//                                left
//                                .splineTo(new Vector2d(12+8, 59-25), Math.toRadians(-75))
//                                .back(10)
//                                .splineTo(new Vector2d(50, 35), Math.toRadians(0))



//                                right
//                                .splineTo(new Vector2d(12-8, 59-25), Math.toRadians(-90 - 15))

                                .forward(3)
                                .splineToConstantHeading(new Vector2d(30, 12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-50, 12), Math.toRadians(180))
                                .back(80)
                                .splineToConstantHeading(new Vector2d(50, 35), Math.toRadians(0))
//                                .forward(40)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}