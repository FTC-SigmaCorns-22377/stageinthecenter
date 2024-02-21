package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
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
    final double distToRollerTip = 17;
    final double distToBackdropBase = 1.5;

    // Calculated Field Parameters
    final double intakeX = -69.75 + distToRollerTip;
    final double outputX = 60 - distToBackdropBase - 0.5 * robotLength;
    final double parkX = 55;

    // Dynamic Field Parameters
    double startX = 0;
    double startY = 0;
    double startHeading = 0;
    Pose2d startPose = new Pose2d(0,0,0);
    double startDelay = 0;
    Vector2d stack1 = new Vector2d(0,0);
    Vector2d stack2 = new Vector2d(0,0);
    Vector2d intakeJunction = new Vector2d(0,0);
    Vector2d outputJunction = new Vector2d(0,0);
    double scoreDelay0 = 0;
    double scoreDelay1 = 0;
    double scoreDelay3 = 0;
    Vector2d scoreVec = new Vector2d(0,0);
    Vector2d scoreOffsetVec = new Vector2d(0,0);
    double parkY = 0;

    // Trajectories
    Trajectory randomization;
    Trajectory cycle1;
    Trajectory cycle2;
    Trajectory cycle3;
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
        cycle1 = randomization;
        cycle2 = randomization;
        cycle3 = randomization;
        park = randomization;
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        initializeTrajectories();
        switch (getTeam()) {
            case BLUE:
                stack2 = new Vector2d(intakeX, 23.75);
                switch (getPark()) {
                    case EDGE:
                        parkY = 59.375;
                        break;
                    default:
                        parkY = 10;
                        break;
                }
                switch (getSide()) {
                    case STACKSIDE:
                        startDelay = 15;
                        switch (getRandomization()) {
                            case LEFT:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .splineToConstantHeading(new Vector2d(-28, 34.8), 0)
                                        .lineToSplineHeading(new Pose2d(-6, 34.8, Math.PI))
                                        .lineTo(new Vector2d(2, 34.8))
                                        .splineToConstantHeading(new Vector2d(22.5, 53), 0)
                                        .splineToConstantHeading(new Vector2d(outputX, 35.625 + 2 * 3), 0)
                                        .build();
                            case RIGHT:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToSplineHeading(new Pose2d(-38.996, 37.5, Math.PI))
                                        .splineToConstantHeading(new Vector2d(-15, 7.174), 0)
                                        .splineToConstantHeading(new Vector2d(outputX, 35.625 - 2 * 3), 0)
                                        .build();
                            default:
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .splineToConstantHeading(new Vector2d(-42.5, 33.254), 0)
                                        .splineToSplineHeading(new Pose2d(-11.875, 35.625, Math.PI), 0)
                                        .lineTo(new Vector2d(outputX, 35.625))
                                        .build();
                        }
                        park = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                .forward(1)
                                .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                .build();
                        break;
                    default:
                        if (getRandomization() == RandomizationSide.CENTER) {
                            stack1 = new Vector2d(intakeX, 35.625);
                            intakeJunction = new Vector2d(-35.625, 35.625);
                            outputJunction = new Vector2d(35.625, 35.625);
                            scoreDelay1 = 2.25;
                            scoreDelay3 = 2.5;
                            scoreVec = new Vector2d(outputX, 35.625 - 4);
                            scoreDelay0 = 2.5;
                            randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                    .lineTo(new Vector2d(0.5 * (startX + 15 + 0.5 * robotLength), 0.5 * (startY + 29.5)))
                                    .lineToSplineHeading(new Pose2d(15 + 0.5 * robotLength, 29.5, Math.PI))
                                    .splineToConstantHeading(new Vector2d(25 + 0.5 * robotLength, 24.75), 0)
                                    .splineToConstantHeading(scoreVec, 0.5 * Math.PI)
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(stack1)
                                    .build();
                            cycle1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(stack1)
                                    .build();
                            cycle2 = robot.drivetrain.getBuilder().trajectoryBuilder(cycle1.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(intakeJunction)
                                    .splineToConstantHeading(stack2, -0.5 * Math.PI)
                                    .build();
                            cycle3 = robot.drivetrain.getBuilder().trajectoryBuilder(cycle2.end())
                                    .strafeRight(1)
                                    .splineToConstantHeading(intakeJunction, 0)
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, -0.5 * Math.PI)
                                    .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                    .build();
                        } else {
                            stack1 = new Vector2d(intakeX, 11.875);
                            intakeJunction = new Vector2d(-29.688, 11.875);
                            outputJunction = new Vector2d(23.75, 11.875);
                            scoreDelay1 = 2.6;
                            scoreDelay3 = 3;
                            scoreVec = new Vector2d(outputX, 35.625 - 2 * 3 - 3);
                            scoreOffsetVec = new Vector2d(5, -1.5);
                            if (getRandomization() == RandomizationSide.LEFT) {
                                scoreDelay0 = 2.1;
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToSplineHeading(new Pose2d(23.75 + 0.5 * robotLength, 35.75, Math.PI))
                                        .splineToConstantHeading(new Vector2d(outputX, 35.625 + 2 * 3), 0.5 * Math.PI)
                                        .splineToConstantHeading(new Vector2d(outputX - 5, 35.625 + 2 * 3 + 2), Math.PI)
                                        .splineToConstantHeading(outputJunction, Math.PI)
                                        .lineTo(stack1)
                                        .build();
                            } else {
                                scoreDelay0 = 2.3;
                                randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToSplineHeading(new Pose2d(1 + 0.5 * robotLength, 37.5, Math.PI))
                                        .splineToConstantHeading(new Vector2d(outputX - 5, 35.625 - 4.5), 0)
                                        .splineToConstantHeading(outputJunction, Math.PI)
                                        .lineTo(stack1)
                                        .build();
                            }
                            cycle1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, 0.5 * Math.PI)
                                    .splineToConstantHeading(scoreVec.minus(scoreOffsetVec), Math.PI)
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(stack1)
                                    .build();
                            cycle2 = robot.drivetrain.getBuilder().trajectoryBuilder(cycle1.end())
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, 0.5 * Math.PI)
                                    .splineToConstantHeading(scoreVec.minus(scoreOffsetVec), Math.PI)
                                    .splineToConstantHeading(outputJunction, Math.PI)
                                    .lineTo(intakeJunction)
                                    .splineToConstantHeading(stack2, 0.5 * Math.PI)
                                    .build();
                            cycle3 = robot.drivetrain.getBuilder().trajectoryBuilder(cycle2.end())
                                    .strafeLeft(1)
                                    .splineToConstantHeading(intakeJunction, 0)
                                    .lineTo(outputJunction)
                                    .splineToConstantHeading(scoreVec, 0.5 * Math.PI)
                                    .splineToConstantHeading(scoreVec.minus(new Vector2d(5, -2)), Math.PI)
                                    .splineToConstantHeading(new Vector2d(parkX, parkY), 0)
                                    .build();
                        }
                }
                break;
            default:
                break;
        }

        ScoringCommandGroups cmd = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Command auto = wait(startDelay);
        switch (getSide()) {
                    case STACKSIDE:
                        auto.addNext(followRR(randomization))
                                .addNext(cmd.scorePos())
                                .addNext(cmd.setSlides(Slides.SlideHeight.HALF))
                                .addNext(cmd.score())
                                .addNext(followRR(park))
                                .addNext(cmd.scorePos());
                        break;
                    default:
                        auto.addNext(new MultipleCommand(followRR(randomization)
//                                        new DelayedCommand(scoreDelay0 - 1, new MultipleCommand(cmd.scorePos(), cmd.setSlides(Slides.SlideHeight.HALF))),
//                                        new DelayedCommand(scoreDelay0, cmd.score()),
//                                        new DelayedCommand(scoreDelay0 + 1, cmd.postScore())
                                ))
//                                .addNext(cmd.newSetTransfer(Intake.TransferState.FIVE))
//                                .addNext(cmd.rollerOn())
//                                .addNext(wait(1.5))
//                                .addNext(cmd.rollerOff())
                                .addNext(new MultipleCommand(followRR(cycle1)
//                                        cmd.newSetTransfer(Intake.TransferState.TRANSFER),
//                                        new DelayedCommand(scoreDelay1 - 1, cmd.scorePos()),
//                                        new DelayedCommand(scoreDelay1, cmd.score()),
//                                        new DelayedCommand(scoreDelay1 + 1, cmd.postScore())
                                ))
//                                .addNext(cmd.newSetTransfer(Intake.TransferState.THREE))
//                                .addNext(cmd.rollerOn())
//                                .addNext(wait(1.5))
//                                .addNext(cmd.rollerOff())
                                .addNext(new MultipleCommand(followRR(cycle2)
//                                        cmd.newSetTransfer(Intake.TransferState.TRANSFER),
//                                        new DelayedCommand(scoreDelay1 - 1, cmd.scorePos()),
//                                        new DelayedCommand(scoreDelay1, cmd.score()),
//                                        new DelayedCommand(scoreDelay1 + 1, cmd.postScore())
                                ))
//                                .addNext(cmd.newSetTransfer(Intake.TransferState.FIVE))
//                                .addNext(cmd.rollerOn())
//                                .addNext(wait(1.5))
//                                .addNext(cmd.rollerOff())
                                .addNext(new MultipleCommand(followRR(cycle3)
//                                        cmd.newSetTransfer(Intake.TransferState.TRANSFER),
//                                        new DelayedCommand(scoreDelay3 - 1, cmd.scorePos()),
//                                        new DelayedCommand(scoreDelay3, cmd.score()),
//                                        new DelayedCommand(scoreDelay3 + 1, cmd.postScore())
                                ));
                        break;
        }

        return auto;
    }
}
