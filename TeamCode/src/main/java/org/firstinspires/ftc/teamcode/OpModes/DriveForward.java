package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;

@Autonomous
public class DriveForward extends BaseAuto {
    Pose2d startPose = new Pose2d(0,0,-0.5 * Math.PI);

    @Override
    public void setRobotPosition() {
        robot.drivetrain.setPose(startPose);
    }

    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        return new RoadrunnerTrajectoryFollower(robot, robot.drivetrain.getBuilder().trajectoryBuilder(startPose)
            .lineToLinearHeading(new Pose2d(50, 50, -0.5 * Math.PI))
            .build()
        );
    }
}
