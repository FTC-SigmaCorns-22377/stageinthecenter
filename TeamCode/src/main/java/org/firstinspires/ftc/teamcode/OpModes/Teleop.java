package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Intake;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Output;

@TeleOp
public class Teleop extends BaseTeleop {
    @Override
    public Command setupTeleop(CommandScheduler scheduler) throws InterruptedException {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

//        robot.gamepad1.whenDPadDownPressed(commandGroups.slidesDown());
        //robot.gamepad1.whenDPadUpPressed();
        robot.gamepad1.whenDPadLeftPressed(commandGroups.slidesDown());
        robot.gamepad1.whenDPadRightPressed(commandGroups.slidesUp());
        robot.gamepad1.whenRightBumperPressed(commandGroups.rollerOn());
        robot.gamepad1.whenRightBumperLifted(commandGroups.rollerOff());
        robot.gamepad1.whenLeftBumperPressed(commandGroups.rollerReverse());
        robot.gamepad1.whenLeftBumperLifted(commandGroups.rollerOff());
        robot.gamepad1.whenCirclePressed(commandGroups.setArm(Output.ArmState.TRANSFER));
        robot.gamepad1.whenSquarePressed(commandGroups.setArm(Output.ArmState.SCORE));
        robot.gamepad1.whenTrianglePressed(commandGroups.setTransfer(Intake.TransferState.INTAKE));
        robot.gamepad1.whenCrossPressed(commandGroups.setTransferTrans(Intake.TransferState.TRANSFER));
        robot.gamepad1.whenDPadUpPressed(commandGroups.setClaw(Output.ClawState.CLOSED));
        robot.gamepad1.whenDPadDownPressed(commandGroups.setClaw(Output.ClawState.OPEN));
//        robot.gamepad1.whenRightTriggerPressed(commandGroups.clawBlackOpen());
//        robot.gamepad1.whenLeftTriggerPressed(commandGroups.clawPurpleOpen());
//        robot.gamepad1.whenCrossPressed(commandGroups.transfer());
//        robot.gamepad1.whenTrianglePressed(commandGroups.scoringPosition());
//        robot.gamepad1.whenCirclePressed(commandGroups.clawsBothOpen());
        return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
    }
}