package org.firstinspires.ftc.teamcode.OpModes;


import android.renderscript.Matrix4f;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.RawAprilTagLocalizer;

import java.util.List;

import kotlin.Pair;
import kotlin.Triple;

@TeleOp
public class AprilTagLocalizationTest extends LinearOpMode {
	@Override
	public void runOpMode() {
		WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 2");

		RawAprilTagLocalizer localizer = new RawAprilTagLocalizer(
				webcam,
				new double[]{-8, 0, 2},
				new double[]{0, 0, 1},
				180
		);

		boolean prevA = false;
		while (!isStopRequested()) {
			boolean a = gamepad1.a;
			if (a && !prevA) {
				List<Triple<Long, Pose2d, Matrix4f>> poses = localizer.getLocalizations();
				for (int i = 0; i < poses.size(); i++) {
					telemetry.addData("Pose " + i, poses.get(i).getSecond().toString());
					telemetry.addData("Matrix " + i, getMatrix(poses.get(i).getThird()));
				}
				telemetry.update();
			}

			prevA = a;
		}
	}

	private static String getMatrix(Matrix4f mat) {
		return "{\n" +
				"\t[" + mat.get(0, 0) + ", " + mat.get(1, 0) + ", " + mat.get(2, 0) + ", " + mat.get(3, 0) + "],\n" +
				"\t[" + mat.get(0, 1) + ", " + mat.get(1, 1) + ", " + mat.get(2, 1) + ", " + mat.get(3, 1) + "],\n" +
				"\t[" + mat.get(0, 2) + ", " + mat.get(1, 2) + ", " + mat.get(2, 2) + ", " + mat.get(3, 2) + "],\n" +
				"\t[" + mat.get(0, 3) + ", " + mat.get(1, 3) + ", " + mat.get(2, 3) + ", " + mat.get(3, 3) + "]\n" +
				"}";
	}
}