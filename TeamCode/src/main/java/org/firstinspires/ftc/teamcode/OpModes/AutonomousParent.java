package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;
import org.firstinspires.ftc.teamcode.Utils.RandomizationSide;

public class AutonomousParent extends BaseAuto {

    // Robot Parameters
    final double robotLength = 17.008;
    final double robotWidth = 14.358;
    final double distToRollerTip = 16.5;
    final double distToBackdropBase = 1.75;

    // Calculated Field Parameters
    final double intakeX = -69 + distToRollerTip;
    final double outputX = 60 - distToBackdropBase - 0.5 * robotLength + 1;
    final double parkX = 53;

    final double going_through_gate_delay = 1.75;

    // Dynamic Field Parameters
    double startX = 0;
    double startY = 0;
    double startHeading = 0;
    Pose2d startPose = new Pose2d(0,0,0);
    double startDelay = 0;
    Vector2d stack1 = new Vector2d(0,0);
    Vector2d stack1_1 = new Vector2d(0,0);

    Vector2d stack2 = new Vector2d(0,0);
    Vector2d intakeJunction = new Vector2d(0,0);
    Vector2d outputJunction = new Vector2d(0,0);
    double parkY = 0;

    // Trajectories
    Trajectory randomization;
    Trajectory randomization2;
    Trajectory intake1;
    Trajectory intake1_1;

    Trajectory output1;
    Trajectory intake2;
    Trajectory output2;
    Trajectory park;

    @Override
    public void setRobotPosition() {
        switch (getSide()) {
            case STACKSIDE:
                startX = -47.5 + 0.5 * robotWidth;
                break;
            default:
                startX = 23.75 - 0.5 * robotWidth;
                break;
        }
        switch (getTeam()) {
            case BLUE:
                startY = 71.25 - 0.5 * robotLength;
                startHeading = -0.5 * Math.PI;
                break;
            default:
                startY = -71.25 + 0.5 * robotLength;
                startHeading = 0.5 * Math.PI;
                break;
        }
        startPose = new Pose2d(startX, startY, startHeading);
        robot.drivetrain.setPose(startPose);
    }

    private void initializeTrajectories() {
        randomization = robot.drivetrain.getBuilder().trajectoryBuilder(new Pose2d(0,0,0))
                .forward(1)
                .build();
        randomization2 = randomization;
        intake1 = randomization;
        output1 = randomization;
        intake2 = randomization;
        output2 = randomization;
        park = randomization;
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        initializeTrajectories();
        switch (getTeam()) {
            case BLUE:
                break;
            case RED:
                stack2 = new Vector2d(intakeX+0.5, -30);
                switch (getPark()) {
                    case EDGE:
                        parkY = -61;
                        break;
                    case CENTER:
                        parkY = -10;
                        break;
                }
                switch (getSide()) {
                    case STACKSIDE:
                        startDelay = 5;
                        switch (getRandomization()) {
                            case RIGHT:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(-24.75 - 0.5 * robotLength, -33, 0.25 * Math.PI))
                                        .build();
                                randomization2 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                        .back(1)
                                        .splineTo(new Vector2d(-24.75 - 0.5 * robotLength, -58), 0)
                                        .lineTo(new Vector2d(23.75, -58))
                                        .splineToConstantHeading(new Vector2d(outputX, -35.625 - 2 * 3 + 3), 0)
                                        .build();
                                break;
                            case LEFT:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineTo(new Vector2d(-46.5, -35.75 - 0.5 * robotLength))
                                        .build();
                                randomization2 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                        .back(1)
                                        .splineTo(new Vector2d(-11.875, -60), 0)
                                        .lineTo(new Vector2d(23.75, -60))
                                        .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3 + 3), 0)
                                        .build();
                                break;
                            case CENTER:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineTo(new Vector2d(-35.625, -24.75 - 0.5 * robotLength))
                                        .build();
                                randomization2 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                        .back(1)
                                        .splineTo(new Vector2d(-11.875, -60), 0)
                                        .lineTo(new Vector2d(23.75, -60))
                                        .splineToConstantHeading(new Vector2d(outputX, -35.625 + 3), 0)
                                        .build();
                                break;
                        }
                        park = robot.drivetrain.getBuilder().trajectoryBuilder(randomization2.end())
                                .forward(1)
                                .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                .build();
                        break;
                    case BACKDROP:
                        if (getRandomization() == RandomizationSide.CENTER) {
                            stack1 = new Vector2d(intakeX, -13);
                            stack1_1 = new Vector2d(intakeX-0.4, -12.5);

                            intakeJunction = new Vector2d(-29.688, -12.5);
                            outputJunction = new Vector2d(23.75, -12.5);
                            randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                    .lineToSplineHeading(new Pose2d(14, -30, Math.PI))
                                    .splineToConstantHeading(new Vector2d(outputX, -35.625), 0)
                                    .build();
//                            intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
//                                    .lineTo(stack1)
//                                    .build();
                            intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .splineToConstantHeading(stack1, Math.PI)
                                    .build();
                            intake1_1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .splineToConstantHeading(stack1_1, Math.PI)
                                    .build();

//                            output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
//                                    .lineTo(new Vector2d(outputX, -35.125))
//                                    .build();
//                            intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
//                                    .lineTo(intakeJunction)
//                                    .splineToConstantHeading(stack2, 0.5 * Math.PI)
//                                    .build();
//                            output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
//                                    .strafeLeft(1)
//                                    .splineToConstantHeading(intakeJunction, 0)
//                                    .lineTo(new Vector2d(outputX, -35.625))
//                                    .build();
                            output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3), 0)
                                    .build();
                            intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(intakeJunction)
                                    .splineToConstantHeading(stack2, -0.5 * Math.PI)
                                    .forward(0.3)
                                    .build();
                            output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
                                    .strafeRight(1)
                                    .splineToConstantHeading(intakeJunction, 0)
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3), 0)
                                    .build();

                        } else {
                            double y_coord = -12.5;
                            stack1 = new Vector2d(intakeX-0.2, y_coord);
                            stack1_1 = new Vector2d(intakeX-0.3, y_coord-1);
                            intakeJunction = new Vector2d(-29.688, y_coord);
                            outputJunction = new Vector2d(23.75, y_coord);
                            if (getRandomization() == RandomizationSide.RIGHT) {
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineTo(new Vector2d(0.2 * startX + 0.8 * (23.75 + 0.5 * robotLength), 0.2 * startY - 0.8 * 35.625))
                                        .lineToSplineHeading(new Pose2d(23.75 + 0.5 * robotLength, -35.625, Math.PI))
                                        .splineToConstantHeading(new Vector2d(outputX, -35.625 - 2 * 3), 0)
                                        .build();
                            } else {
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToSplineHeading(new Pose2d(1 + 0.5 * robotLength, -40, Math.PI))
                                        .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3), 0)
                                        .build();
                            }
                            intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .splineToConstantHeading(stack1, Math.PI)
                                    .build();
                            intake1_1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .splineToConstantHeading(stack1_1, Math.PI)
                                    .build();
                            output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3), 0)
                                    .build();
                            intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(intakeJunction)
                                    .splineToConstantHeading(stack2, -0.5 * Math.PI)
                                    .build();
                            output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
                                    .strafeRight(1)
                                    .splineToConstantHeading(intakeJunction, 0)
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(new Vector2d(outputX, -35.625 + 2 * 3), 0)
                                    .build();
                        }
                        park = robot.drivetrain.getBuilder().trajectoryBuilder(output2.end())
                                .forward(1)
                                .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                .build();
                }
                break;
        }

        ScoringCommandGroups cmd = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Command auto = wait(startDelay);
        switch (getSide()) {
            case STACKSIDE:
                auto.addNext(followRR(randomization))
                        .addNext(followRR(randomization2))
                        .addNext(cmd.scorePos())
                        .addNext(cmd.setSlides(Slides.SlideHeight.L1))
                        .addNext(wait(1.5))
                        .addNext(cmd.score())
                        .addNext(followRR(park));
                break;
            case BACKDROP:
                auto.addNext(new MultipleCommand(followRR(randomization),
                                new DelayedCommand(0.5, new MultipleCommand(cmd.scorePos(), cmd.setSlides(Slides.SlideHeight.HALF)))))
                        .addNext(cmd.score())
                        .addNext(new MultipleCommand(followRR(intake1),
                                 new MultipleCommand(cmd.postScore().addNext(new Delay(going_through_gate_delay).addNext(cmd.newSetTransfer(Intake.TransferState.FIVE))).addNext(cmd.rollerOn()))))
                        .addNext(wait(0.25))
                        .addNext(cmd.newSetTransfer(Intake.TransferState.FOUR))
                        .addNext(wait(0.5))
                        .addNext(cmd.rollerOff())
                        .addNext(cmd.newSetTransfer(Intake.TransferState.TRANSFER))
                        .addNext(new MultipleCommand(followRR(output1),
                                new DelayedCommand(0.5, new MultipleCommand(cmd.scorePos(), cmd.slide4()))))
                        .addNext(cmd.score())
                        .addNext(new MultipleCommand(followRR(intake1_1),
                                new MultipleCommand(cmd.postScore().addNext(new Delay(going_through_gate_delay).addNext(cmd.newSetTransfer(Intake.TransferState.THREE))).addNext(cmd.rollerOn()))))
                        .addNext(wait(0.25))
                        .addNext(cmd.newSetTransfer(Intake.TransferState.INTAKE))
                        .addNext(wait(0.5))
                        .addNext(cmd.rollerOff())
                        .addNext(cmd.newSetTransfer(Intake.TransferState.TRANSFER))
                        .addNext(new MultipleCommand(followRR(output1),
                                new DelayedCommand(0.5, new MultipleCommand(cmd.scorePos(), cmd.slide4()))))
                        .addNext(cmd.score())
                        .addNext(new MultipleCommand(followRR(intake2),
                                new MultipleCommand(cmd.postScore().addNext(new Delay(going_through_gate_delay).addNext(cmd.newSetTransfer(Intake.TransferState.FIVE))).addNext(cmd.rollerOn()))))
                        .addNext(wait(0.25))
                        .addNext(cmd.newSetTransfer(Intake.TransferState.THREE))
                        .addNext(wait(0.3))
                        .addNext(cmd.rollerOff())
                        .addNext(cmd.newSetTransfer(Intake.TransferState.TRANSFER))
                        .addNext(new MultipleCommand(followRR(output2),
                                new DelayedCommand(0.5, new MultipleCommand(cmd.scorePos(), cmd.slide4()))))
                        .addNext(cmd.score())
                        .addNext(followRR(park));
                break;
        }

        return auto;
    }
}