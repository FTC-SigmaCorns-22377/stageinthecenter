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
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

@Autonomous
public class AutonomousParent extends BaseAuto {

    final double robotLength = 17.008;
    final double robotWidth = 14.358;
    final double distToRollerTip = 17.25;
    final double distToBackdropBase = 1.25;
    final double startX = 23.75 - 0.5 * robotWidth;
    final double startY = 71.25 - 0.5 * robotLength;
    final double startHeading = -0.5 * Math.PI;
    final Pose2d startPose = new Pose2d(startX, startY, startHeading);
    final double intakeX = -69.75 + distToRollerTip;
    final double outputX = 60 - distToBackdropBase - 0.5 * robotLength;
    final Vector2d intakeJunction = new Vector2d(-35.625, 35.625);

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        final Trajectory randomization = robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
                .forward(startY - (24.75 + 0.5 * robotLength))
                .build();
        final Trajectory randomization2 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization.end())
                .back(1)
                .splineTo(new Vector2d(outputX + 0.5, 35.625), 0)
                .build();
        final Trajectory intake1 = robot.drivetrain.getBuilder().trajectoryBuilder(randomization2.end())
                .lineTo(new Vector2d(intakeX, 35.625))
                .build();
        final Trajectory output1 = robot.drivetrain.getBuilder().trajectoryBuilder(intake1.end())
                .lineTo(new Vector2d(outputX, 35.625))
                .build();
        final Trajectory intake2 = robot.drivetrain.getBuilder().trajectoryBuilder(output1.end())
                .lineTo(new Vector2d(intakeX, 35.625))
                .build();
        final Trajectory output2 = robot.drivetrain.getBuilder().trajectoryBuilder(intake2.end())
                .lineTo(new Vector2d(outputX, 35.625))
                .build();
        final Trajectory intake3 = robot.drivetrain.getBuilder().trajectoryBuilder(output2.end())
                .lineTo(intakeJunction)
                .splineToConstantHeading(new Vector2d(intakeX, 23.75), -0.5 * Math.PI)
                .build();
        final Trajectory output3 = robot.drivetrain.getBuilder().trajectoryBuilder(intake3.end())
                .strafeRight(1)
                .splineToConstantHeading(intakeJunction, 0)
                .lineTo(new Vector2d(outputX, 35.625))
                .build();
        final Trajectory park = robot.drivetrain.getBuilder().trajectoryBuilder(output3.end())
                .forward(3)
                .build();

        ScoringCommandGroups cmd = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        Command auto = followRR(randomization)
                .addNext(cmd.scorePosNoLift())
                .addNext(cmd.setSlides(Slides.SlideHeight.L1))
                .addNext(followRR(randomization2))
                .addNext(cmd.score())
                .addNext(new MultipleCommand(followRR(intake1),
                        new DelayedCommand(0.5, cmd.postScore()),
                        new DelayedCommand(0.75,
                                         new MultipleCommand(
                                                cmd.newSetTransfer(Intake.TransferState.FIVE),
                                                cmd.setRollerPosition(Intake.TransferState.FIVE), cmd.rollerOn()
                                        )
                                )
                        ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(
                                cmd.setRollerPosition(Intake.TransferState.FOUR),
                                cmd.newSetTransfer(Intake.TransferState.FOUR)
                        ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(followRR(output1), cmd.setRollerPosition(Intake.TransferState.TRANSFER),cmd.newSetTransfer(Intake.TransferState.TRANSFER),cmd.rollerOff(),
                                new DelayedCommand(2.5, new MultipleCommand(cmd.scorePosNoLift(), cmd.setSlides(Slides.SlideHeight.L1)))))
                .addNext(cmd.score())
                .addNext(new MultipleCommand(followRR(intake2),
                        new DelayedCommand(0.5, cmd.postScore()),
                        new DelayedCommand(0.75,
                                new MultipleCommand(
                                        cmd.newSetTransfer(Intake.TransferState.FIVE),
                                        cmd.setRollerPosition(Intake.TransferState.FIVE), cmd.rollerOn()
                                )
                        )
                ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(
                        cmd.setRollerPosition(Intake.TransferState.FOUR),
                        cmd.newSetTransfer(Intake.TransferState.FOUR)
                ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(followRR(output2), cmd.setRollerPosition(Intake.TransferState.TRANSFER),cmd.newSetTransfer(Intake.TransferState.TRANSFER),cmd.rollerOff(),
                        new DelayedCommand(2.5, new MultipleCommand(cmd.scorePosNoLift(), cmd.setSlides(Slides.SlideHeight.L1)))))
                .addNext(cmd.score())
                .addNext(new MultipleCommand(followRR(intake3),
                        new DelayedCommand(0.5, cmd.postScore()),
                        new DelayedCommand(0.75,
                                new MultipleCommand(
                                        cmd.newSetTransfer(Intake.TransferState.FIVE),
                                        cmd.setRollerPosition(Intake.TransferState.FIVE), cmd.rollerOn()
                                )
                        )
                ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(
                        cmd.setRollerPosition(Intake.TransferState.FOUR),
                        cmd.newSetTransfer(Intake.TransferState.FOUR)
                ))
                .addNext(wait(1.5))
                .addNext(new MultipleCommand(followRR(output3), cmd.setRollerPosition(Intake.TransferState.TRANSFER),cmd.newSetTransfer(Intake.TransferState.TRANSFER),cmd.rollerOff(),
                        new DelayedCommand(2.5, new MultipleCommand(cmd.scorePosNoLift(), cmd.setSlides(Slides.SlideHeight.L1)))))
                .addNext(cmd.score())
                .addNext(followRR(park));

        return auto;
    }
}