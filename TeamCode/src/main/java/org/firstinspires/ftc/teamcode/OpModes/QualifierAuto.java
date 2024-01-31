package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class QualifierAuto extends BaseAuto {

//    // Robot Parameters
//    final Double robotLength = 17.008;
//    final Double robotWidth = 14.3;
//    final Double distToPixelSlot = (8.5 + 10 + 9.75) / 3;
//    final Double distToRollerTip = 18.5;
//    final Double distToBackdropBase = 1.5;
//
//    // Calculated Field Parameters
//    final Double startY = 71.25 - 0.5 * robotLength;
//    final Double intakeX = -69.75 + distToRollerTip;
//    final Double outputX = 60 - distToBackdropBase - 0.5 * robotLength;
//    Double parkY = 0.0;
//
//    // Dynamic Points
//    List<Vector2d> backdropSlots = new ArrayList<>();
//    Vector2d stack = new Vector2d(0, 0);
//    Pose2d startPose = new Pose2d(0, 0, 0);
//    Pose2d spikeMark = new Pose2d(0, 0, 0);
//    Vector2d junction = new Vector2d(0, 0);
//    Vector2d backdrop = new Vector2d(0, 0);
//    int randomizationSlot = 0;
//    int outputSlot1 = 0;
//    int outputSlot2 = 0;

    final Pose2d startPose = new Pose2d(11.875, -71.25 + 14.3/2, Math.PI);

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Trajectory trajectory1 = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .lineTo(new Vector2d(58, -10))
                .build();

        Trajectory trajectory2 = robot.drivetrain.getBuilder().trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(58, 30))
                .build();

        Trajectory trajectory3 = robot.drivetrain.getBuilder().trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(-69.75 + 18.5, -11.875, Math.PI))
                .build();

        Command auto = commandGroups.setArm(Output.ArmState.SCORE);
        auto.addNext(commandGroups.setSlides(Slides.SlideHeight.L2));
        auto.addNext(followRR(trajectory1));
        auto.addNext(commandGroups.setClaw(Output.ClawState.OPEN));
        auto.addNext(followRR(trajectory2));
        auto.addNext(commandGroups.setArm(Output.ArmState.TRANSFER));
        auto.addNext(commandGroups.setSlides(Slides.SlideHeight.L0));

        return auto;
    }
}