package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RobotRelative;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.ScoringCommandGroups;

@TeleOp
public class Teleop extends BaseTeleop {
    @Override
    public Command setupTeleop(CommandScheduler scheduler) throws InterruptedException {

        ScoringCommandGroups commandGroups = new ScoringCommandGroups(robot.scoringMechanism, robot.drivetrain);

        robot.gamepad1.whenDPadDownPressed(commandGroups.slidesDown());
        robot.gamepad1.whenDPadUpPressed(commandGroups.intakeDown());
        robot.gamepad1.whenRightBumperPressed(commandGroups.rollerOn());
        robot.gamepad1.whenRightBumperLifted(commandGroups.rollerOff());
        robot.gamepad1.whenRightTriggerPressed(commandGroups.clawBlackOpen());
        robot.gamepad1.whenLeftTriggerPressed(commandGroups.clawPurpleOpen());
        robot.gamepad1.whenCrossPressed(commandGroups.transfer());
        robot.gamepad1.whenTrianglePressed(commandGroups.scoringPosition());
        robot.gamepad1.whenCirclePressed(commandGroups.clawsBothOpen());
        return new MultipleCommand(new RobotRelative(robot, robot.gamepad1));
    }
}