package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.FieldRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.ResetHeading;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.PrimitiveMovements.SlideOverride;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Slides;

@TeleOp
public class Teleop extends BaseTeleop {
    @Override
    public Command setupTeleop(CommandScheduler scheduler) throws InterruptedException {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

//        robot.gamepad1.whenDPadDownPressed(commandGroups.slidesDown());
        //robot.gamepad1.whenDPadUpPressed();
        robot.gamepad1.whenDPadLeftPressed(commandGroups.blackscore());
        robot.gamepad1.whenDPadRightPressed(commandGroups.purplescore());
        robot.gamepad1.whenRightBumperPressed(commandGroups.rollerOn());
        robot.gamepad1.whenRightBumperLifted(commandGroups.rollerOff());
        robot.gamepad1.whenLeftBumperPressed(commandGroups.rollerReverse());
        robot.gamepad1.whenLeftBumperLifted(commandGroups.rollerOff());
        robot.gamepad1.whenTrianglePressed(commandGroups.newSetTransfer(Intake.TransferState.FOUR));
        robot.gamepad1.whenCirclePressed(commandGroups.newSetTransfer(Intake.TransferState.THREE));
        robot.gamepad1.whenSquarePressed(commandGroups.newSetTransfer(Intake.TransferState.FIVE));
        //robot.gamepad1.whenTrianglePressed(commandGroups.setTransfer(Intake.TransferState.INTAKE));
        robot.gamepad1.whenCrossPressed(commandGroups.newSetTransfer(Intake.TransferState.TRANSFER));
        //robot.gamepad1.whenDPadUpPressed(commandGroups.setClaw(Output.ClawState.CLOSED));
        //robot.gamepad1.whenDPadDownPressed(commandGroups.setClaw(Output.ClawState.OPEN));
        robot.gamepad1.whenRightTriggerPressed(commandGroups.score());
        robot.gamepad1.whenLeftTriggerPressed(commandGroups.fireDroneSequence());
        robot.gamepad1.whenDPadDownPressed(commandGroups.postScore());
        robot.gamepad1.whenDPadUpPressed(commandGroups.retractDrone());
//        robot.gamepad1.whenRightTriggerPressed(commandGroups.clawBlackOpen());
//        robot.gamepad1.whenLeftTriggerPressed(commandGroups.clawPurpleOpen());
//        robot.gamepad1.whenCrossPressed(commandGroups.transfer());
//        robot.gamepad1.whenTrianglePressed(commandGroups.scoringPosition());
//        robot.gamepad1.whenCirclePressed(commandGroups.clawsBothOpen());
        robot.gamepad2.whenRightBumperPressed(commandGroups.hangOn());
        robot.gamepad2.whenRightBumperLifted(commandGroups.hangOff());
        robot.gamepad2.whenLeftBumperPressed(commandGroups.hangReverse());
        robot.gamepad2.whenLeftBumperLifted(commandGroups.hangOff());
        robot.gamepad2.whenLeftTriggerPressed(commandGroups.scorePos());
        robot.gamepad2.whenRightTriggerPressed(commandGroups.Intake());


        //robot.gamepad2.whenLeftTriggerLifted()
        //robot.gamepad2.whenRightTriggerPressed(commandGroups.setWrist(Output.WristState.DEG120));
        robot.gamepad2.whenCirclePressed(commandGroups.setWrist(Output.WristState.DEG120));
        robot.gamepad2.whenDPadUpPressed(commandGroups.setSlides(Slides.SlideHeight.L5));
        robot.gamepad2.whenDPadLeftPressed((commandGroups.setSlides(Slides.SlideHeight.L2)));
        robot.gamepad2.whenDPadDownPressed((commandGroups.setSlides(Slides.SlideHeight.L3)));
        robot.gamepad2.whenDPadRightPressed((commandGroups.setSlides(Slides.SlideHeight.L4)));
        robot.gamepad2.whenSquarePressed((commandGroups.setWrist(Output.WristState.DEG60)));
        robot.gamepad2.whenTrianglePressed(commandGroups.setWrist(Output.WristState.DEG180));
        robot.gamepad2.whenCrossPressed(commandGroups.setWrist(Output.WristState.DEG0));
        //robot.gamepad2.whenTrianglePressed(commandGroups.setTransfer(Intake.TransferState.THREE));

        robot.gamepad1.whenRightStickButtonPressed(new ResetHeading(robot));

        return new MultipleCommand(new RobotRelative(robot, robot.gamepad1), new SlideOverride(robot,robot.gamepad2));
    }
}