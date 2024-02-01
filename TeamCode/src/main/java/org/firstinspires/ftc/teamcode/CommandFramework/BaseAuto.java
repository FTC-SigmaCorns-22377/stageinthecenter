package org.firstinspires.ftc.teamcode.CommandFramework;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Commands.DrivetrainCommands.RoadrunnerTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.Delay;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.DelayedCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.MultipleCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.MiscCommands.RaceAction;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Utils.RandomizationSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;

public abstract class BaseAuto extends LinearOpMode {

    protected Robot robot;

    protected RandomizationSide randomizationSide = RandomizationSide.CENTER;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, Robot.OpMode.Auto, gamepad1, gamepad2, getTeam(), getSide());
        setRobotPosition();

        waitForStart();

        System.out.println("randomization side");
//        while (!isStarted()) {
//            randomizationSide = robot.randomization.getRandomizationSide();
//            telemetry.addData("randomization", randomizationSide.name());
//            telemetry.update();
//        }
//        robot.randomization.closePortal();

        robot.getScheduler().forceCommand(setupAuto(robot.getScheduler()));

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
        }
        robot.shutdown();
    }

    public abstract Command setupAuto(CommandScheduler scheduler);

    public RoadrunnerTrajectoryFollower followRR(Trajectory trajectory) {
        return new RoadrunnerTrajectoryFollower(this.robot, trajectory);
    }

    public DelayedCommand delayCommand(double time, Command command) {
        return new DelayedCommand(time, command);
    }

    public MultipleCommand multiCommand(Command... commands) {
        return new MultipleCommand(commands);
    }

    public Delay wait(double seconds) {
        return new Delay(seconds);
    }

    public RaceAction raceCommand(Command... commands) {
        return new RaceAction(commands);
    }

    public void setRobotPosition() {

    }

    public Team getTeam() {
        return Team.NOT_ASSIGNED;
    }

    public Side getSide() {
        return Side.NOT_ASSIGNED;
    }
}
