package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;

public class Dashboard extends Subsystem {


	public static TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();
	ElapsedTime dashboardTimer = new ElapsedTime();

	@Override
	public void initAuto(HardwareMap hwMap) {
		dashboard.sendTelemetryPacket(packet);
		packet = new TelemetryPacket();
		//dashboard.setTelemetryTransmissionInterval(250);
	}

	@Override
	public void periodic() {
		dashboard.sendTelemetryPacket(packet);
		packet = new TelemetryPacket();
		packet.put("Loop time", dashboardTimer.milliseconds());
		dashboardTimer.reset();
	}


	public void startCameraStream(CameraStreamSource source, int maxFps) {
		dashboard.startCameraStream(source, maxFps);
	}

	@Override
	public void shutdown() {

	}

	public static String round2Decimals(double value) {
		return String.valueOf((double) Math.round(value * 100) / 100);
	}
}
