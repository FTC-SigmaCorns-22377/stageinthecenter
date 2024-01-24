package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class PerseveranceAuto extends BaseAuto {

    // Robot Parameters
    final Double robotLength = 17.008;
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
    Vector2d junction = new Vector2d(0, 0);
    Vector2d stack = new Vector2d(0, 0);
    List<Vector2d> backdropSlots = new ArrayList<>();
    int randomizationSlot = 0;
    int outputSlot1 = 0;
    int outputSlot2 = 0;

    @Override
    public void setRobotPosition() {
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
                        startPose = new Pose2d(11.875, startY, -0.5 * Math.PI);
                        switch (randomizationSide) {
                            case LEFT:
                                spikeMark = new Pose2d(22.75, 35.75 + distToPixelSlot, -0.5 * Math.PI);
                                junction = new Vector2d(23.75, 11.875);
                                stack = new Vector2d(intakeX, 11.875);
                                randomizationSlot = 8;
                                outputSlot1 = 5;
                                outputSlot2 = 1;
                                break;
                            default:
                                spikeMark = new Pose2d(11.875, 24.75 + distToPixelSlot, -0.5 * Math.PI);
                                junction = new Vector2d(31.25, 35.625);
                                stack = new Vector2d(intakeX, 35.625);
                                randomizationSlot = 4;
                                outputSlot1 = 9;
                                outputSlot2 = 1;
                                break;
                            case RIGHT:
                                spikeMark = new Pose2d(1 + distToPixelSlot, 28.125, Math.PI);
                                junction = new Vector2d(23.75, 11.875);
                                stack = new Vector2d(intakeX, 11.875);
                                randomizationSlot = 0;
                                outputSlot1 = 5;
                                outputSlot2 = 9;
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

        Trajectory yellow = robot.drivetrain.getBuilder().trajectoryBuilder(purple.end(),true)
                .splineToLinearHeading(new Pose2d(backdropSlots.get(randomizationSlot), Math.PI), 0)
                .build();

        Trajectory intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(yellow.end(), true)
                .splineToConstantHeading(junction, Math.PI)
                .lineTo(stack)
                .build();

        Trajectory output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
                .lineTo(junction)
                .splineToConstantHeading(backdropSlots.get(outputSlot1), 0)
                .build();

        Trajectory intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end(), true)
                .splineToConstantHeading(junction, Math.PI)
                .lineTo(stack)
                .build();

        Trajectory output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
                .lineTo(junction)
                .splineToConstantHeading(backdropSlots.get(outputSlot2), 0)
                .build();

        Command auto = followRR(purple);
        auto.addNext(followRR(yellow));
        auto.addNext(followRR(intake1));
        auto.addNext(followRR(output1));
        auto.addNext(followRR(intake2));
        auto.addNext(followRR(output2));

        return auto;
    }
}