package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
// import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;

// import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class PerseveranceAuto extends BaseAuto {

    // Robot Parameters -- TUNE
    final Double robotLength = 17.008;
    final Double distToPixelSlot = (8.5 + 10 + 9.75) / 3;
    final Double distToRollerTip = 18.5;
    final Double distToBackdropBase = 1.5;

    // Calculated Field Parameters
    final Double startY = 71.25 - 0.5 * robotLength;
    final Double intakeX = -69.75 + distToRollerTip;
    final Double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

    // Dynamic Points
    List<Vector2d> backdropSlots = new ArrayList<>();
    Vector2d stack = new Vector2d(0, 0);
    Pose2d startPose = new Pose2d(0, 0, 0);
    Pose2d spikeMark = new Pose2d(0, 0, 0);
    Vector2d junction = new Vector2d(0, 0);
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
                for (int i = -2; i <= 2; i++) {
                    backdropSlots.add(new Vector2d(outputX, 35.625 + 3 * i));
                }
                switch (getSide()) {
                    case FRONT:
                        startPose = new Pose2d(11.875, startY, -0.5 * Math.PI);
                        switch (randomizationSide) {
                            case LEFT:
                                stack = new Vector2d(intakeX, 11.875);
                                spikeMark = new Pose2d(22.75 + distToPixelSlot, 29.75, Math.PI);
                                junction = new Vector2d(23.75, 11.875);
                                randomizationSlot = 4;
                                outputSlot1 = 3;
                                outputSlot2 = 1;
                                break;
                            case CENTER:
                                stack = new Vector2d(intakeX, 35.625);
                                spikeMark = new Pose2d(11.875, 24.75 + distToPixelSlot, -0.5 * Math.PI);
                                junction = new Vector2d(31.25, 35.625);
                                randomizationSlot = 3;
                                outputSlot1 = 4;
                                outputSlot2 = 0;
                                break;
                            case RIGHT:
                                stack = new Vector2d(intakeX, 11.875);
                                spikeMark = new Pose2d(1 + distToPixelSlot, 30.25, Math.PI);
                                junction = new Vector2d(23.75, 11.875);
                                randomizationSlot = 0;
                                outputSlot1 = 1;
                                outputSlot2 = 3;
                                break;
                            default:
                                stack = new Vector2d(intakeX, 35.625);
                                spikeMark = new Pose2d(11.875, 24.75 + distToPixelSlot, -0.5 * Math.PI);
                                junction = new Vector2d(31.25, 35.625);
                                randomizationSlot = 3;
                                outputSlot1 = 4;
                                outputSlot2 = 0;
                                break;
                        }
                    case BACK:
                        startPose = new Pose2d(-35.625, startY, -0.5 * Math.PI);
                        switch (randomizationSide) {
                            case LEFT:
                            case CENTER:
                            case RIGHT:
                        }
                }
            case RED:
        }

        Trajectory trajectory = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .lineToLinearHeading(spikeMark)
                .back(5)
                .splineToLinearHeading(new Pose2d(backdropSlots.get(randomizationSlot), Math.PI), 0)
                .forward(5)
                .splineToConstantHeading(junction, Math.PI)
                .lineTo(stack)
                .lineTo(junction)
                .splineToConstantHeading(backdropSlots.get(outputSlot1), 0)
                .forward(5)
                .splineToConstantHeading(junction, Math.PI)
                .lineTo(stack)
                .lineTo(junction)
                .splineToConstantHeading(backdropSlots.get(outputSlot2), 0)
                .build();

        return followRR(trajectory);
    }
}