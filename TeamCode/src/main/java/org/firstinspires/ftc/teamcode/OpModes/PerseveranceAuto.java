package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class PerseveranceAuto extends BaseAuto {

    // Robot Parameters
    final Double robotLength = 17.008;
    final Double distToRollerTip = 17.0;
    final Double distToBackdropBase = 1.75;

    // Calculated Field Parameters
    final Double startY = 71.25 - 0.5 * robotLength;
    final Double intakeX = -69.75 + distToRollerTip;
    final Double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

    // Dynamic Points
    Pose2d startPose = new Pose2d(0, 0, 0);
    Pose2d spikeMark = new Pose2d(0, 0, 0);
    Vector2d intakeJunction = new Vector2d(0, 0);
    Vector2d stack1 = new Vector2d(0, 0);
    Vector2d stack2 = new Vector2d(0, 0);
    Vector2d outputJunction = new Vector2d(0, 0);
    List<Vector2d> backdropSlots = new ArrayList<>();
    int randomizationSlot = 0;
    int outputSlot1 = 0;
    int outputSlot2 = 0;
    int outputSlot3 = 0;

    @Override
    public void setRobotPosition() {
        switch (getTeam()) {
            default:
                switch (getSide()) {
                    default:
                        startPose = new Pose2d(11.875, startY, -0.5 * Math.PI);
                        break;
                }
                break;
            case RED:
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
                switch (getSide()) {
                    case FRONT:
                        switch (randomizationSide) {
                            case LEFT:
                                spikeMark = new Pose2d(23.75, 35.75 + 0.5 * robotLength, -0.5 * Math.PI);
                                intakeJunction = new Vector2d(-29.688, 5.3125);
                                stack1 = new Vector2d(intakeX, 5.3125);
                                outputJunction = new Vector2d(35.625, 5.3125);
                                randomizationSlot = 8;
                                outputSlot1 = 5;
                                outputSlot2 = 1;
                                outputSlot3 = 0;
                                break;
                            default:
                                spikeMark = new Pose2d(11.875, 24.75 + 0.5 * robotLength, -0.5 * Math.PI);
                                intakeJunction = new Vector2d(-35.625, 35.625);
                                stack1 = new Vector2d(intakeX, 35.625);
                                stack2 = new Vector2d(intakeX, 15);
                                outputJunction = new Vector2d(11.875, 35.625);
                                randomizationSlot = 4;
                                outputSlot1 = 9;
                                outputSlot2 = 1;
                                outputSlot3 = 0;
                                break;
                            case RIGHT:
                                spikeMark = new Pose2d(1 + 0.5 * robotLength, 28.125, Math.PI);
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
                        switch (randomizationSide) {
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

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Trajectory purple = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .lineToLinearHeading(spikeMark)
                .build();

        Trajectory yellow = robot.drivetrain.getBuilder().trajectoryBuilder(purple.end())
                .back(1)
                .splineToConstantHeading(backdropSlots.get(randomizationSlot), 0)
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

        Command auto = followRR(purple);
        auto.addNext(new MultipleCommand(followRR(yellow),
                new DelayedCommand(0.5, new MultipleCommand(
                        commandGroups.setArm(Output.ArmState.SCORE),
                        commandGroups.setSlides(Slides.SlideHeight.L1)
                ))
        ));
        auto.addNext(commandGroups.score());
        auto.addNext(commandGroups.postScore());
        auto.addNext(followRR(intake1));
        auto.addNext(commandGroups.postScore());
        auto.addNext(commandGroups.rollerOn());
        auto.addNext(wait(3.0));
        auto.addNext(commandGroups.rollerOff());
        auto.addNext(commandGroups.transferPos());
        auto.addNext(new MultipleCommand(followRR(output1),
                new DelayedCommand(1.0, commandGroups.setClaw(Output.ClawState.CLOSED)),
                new DelayedCommand(1.5, commandGroups.setArm(Output.ArmState.SCORE)),
                new DelayedCommand(1.5, commandGroups.setSlides(Slides.SlideHeight.L1))));
        auto.addNext(commandGroups.score());
//        auto.addNext(commandGroups.postScore());
//        auto.addNext(followRR(intake2));
//        auto.addNext(commandGroups.postScore());
//        auto.addNext(commandGroups.rollerOn());
//        auto.addNext(wait(3.0));
//        auto.addNext(commandGroups.rollerOff());
//        auto.addNext(commandGroups.transferPos());
//        auto.addNext(new MultipleCommand(followRR(output2),
//                new DelayedCommand(0.5, commandGroups.setClaw(Output.ClawState.CLOSED)),
//                new DelayedCommand(1.5, commandGroups.setArm(Output.ArmState.SCORE)),
//                new DelayedCommand(1.5, commandGroups.setSlides(Slides.SlideHeight.LOWMID))));
//        auto.addNext(commandGroups.score());
//        auto.addNext(wait(0.5));
//        auto.addNext(commandGroups.postScore());
//        auto.addNext(followRR(intake3));
//        auto.addNext(commandGroups.postScore());
//        auto.addNext(commandGroups.rollerOn());
//        auto.addNext(wait(3.0));
//        auto.addNext(commandGroups.rollerOff());
//        auto.addNext(commandGroups.transferPos());
//        auto.addNext(new MultipleCommand(followRR(output3),
//                new DelayedCommand(0.5, commandGroups.setClaw(Output.ClawState.CLOSED)),
//                new DelayedCommand(1.5, commandGroups.setArm(Output.ArmState.SCORE)),
//                new DelayedCommand(1.5, commandGroups.setSlides(Slides.SlideHeight.LOWMID))));
//        auto.addNext(commandGroups.score());

        return auto;
    }
}