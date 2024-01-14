package org.firstinspires.ftc.teamcode.Robot.Subsystems;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class Drivetrain extends Subsystem {
	protected HardwareMap hwMap;
	public SampleMecanumDrive drive;

	@Override
	public void initAuto(HardwareMap hwMap) {
		this.hwMap = hwMap;
		drive = new SampleMecanumDrive(hwMap);
	}

	@Override
	public void periodic() {
		drive.update();
	}



	@Override
	public void shutdown() {
		drive.setMotorPowers(0, 0, 0, 0);
	}

	public void setPose(Pose2d pose) {
		drive.setPoseEstimate(pose);
	}


	public void followTrajectory(Trajectory trajectory) {
		drive.followTrajectoryAsync(trajectory);
	}

	public Pose2d getPose() {
		return drive.getPoseEstimate();
	}

	public boolean isBusy() {
		return drive.isBusy();
	}

	public SampleMecanumDrive getBuilder() {
		return drive;
	}



}
