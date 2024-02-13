package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

import java.util.ArrayList;
import java.util.List;

public class AutonomousParent extends BaseAuto {

    // Robot Parameters
    final double robotLength = 17.008;
    final double robotWidth = 14.358;
    final double distToRollerTip = 17;
    final double distToBackdropBase = 1;

    // Calculated Field Parameters
    final double startY = 71.25 - 0.5 * robotLength;
    final double intakeX = -69.75 + distToRollerTip;
    final double outputX = 60 - distToBackdropBase - 0.5 * robotLength;

    // Dynamic Field Parameters
    Pose2d startPose = new Pose2d(0, 0, 0);
    List<Vector2d> backdropVecs = new ArrayList<>();
    Vector2d parkVec = new Vector2d(0, 0);

    @Override
    public void setRobotPosition() {
        switch (getSide()) {
            default:
                break;
            case STACKSIDE:
                switch (getTeam()) {
                    default:
                        break;
                    case BLUE:
                        startPose = new Pose2d(-47.5 + 0.5 * robotLength, startY, -0.5 * Math.PI);
                        break;
                }
                break;
        }
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        ScoringCommandGroups cmd = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);
        Trajectory traj = null;
        Trajectory park = null;

        switch (getTeam()) {
            default:
                break;
            case BLUE:
                for (int i = -2; i <= 2; i++) {
                    backdropVecs.add(new Vector2d(outputX, 35.625 + 1.5 * i));
                }
                switch (getSide()) {
                    default:
                        break;
                    case STACKSIDE:
                        switch (getRandomization()) {
                            case LEFT: // 3.81s
                                traj = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .splineToConstantHeading(new Vector2d(-28, 34.8), 0)
                                        .lineToSplineHeading(new Pose2d(-6, 34.8, Math.PI))
                                        .lineTo(new Vector2d(0, 34.8))
                                        .splineToConstantHeading(new Vector2d(22.5, 47.5), 0)
                                        .splineToConstantHeading(backdropVecs.get(4), 0)
                                        .build();
                                break;
                            default: // 3.82s
                                traj = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .splineToConstantHeading(new Vector2d(-42.5,33.254), 0)
                                        .splineToSplineHeading(new Pose2d(-11.875, 35.625, Math.PI), 0)
                                        .lineTo(backdropVecs.get(2))
                                        .build();
                                break;
                            case RIGHT: // 4.10s
                                traj = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                                        .lineToSplineHeading(new Pose2d(-38.996, 37.5, Math.PI))
                                        .splineToConstantHeading(new Vector2d(-15, 7.174), 0)
                                        .splineToConstantHeading(backdropVecs.get(0), 0)
                                        .build();
                        }
                        break;
                }
                switch (getPark()) {
                    default:
                        parkVec = new Vector2d(59.375, 10);
                        break;
                    case EDGE:
                        parkVec = new Vector2d(59.375, 59.375);
                        break;
                }
                park = robot.drivetrain.getBuilder().trajectoryBuilder(traj.end())
                        .forward(5)
                        .splineToConstantHeading(parkVec, 0)
                        .build();
                break;
        }

        Command auto = followRR(traj);
        auto.addNext(cmd.scorePos());
        auto.addNext(cmd.setSlides(Slides.SlideHeight.HALF));
        auto.addNext(cmd.score());
        auto.addNext(followRR(park));

        return auto;
    }
}
