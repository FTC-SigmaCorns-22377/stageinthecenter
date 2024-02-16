package org.firstinspires.ftc.teamcode.Utils;

import android.renderscript.Matrix4f;
import android.util.Log;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Math.Geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import kotlin.Pair;
import kotlin.Triple;

public class RawAprilTagLocalizer {
	private static final Matrix4f[] FIELD_FROM_TAG;

	static {
		FIELD_FROM_TAG = new Matrix4f[10];
		for (int i = 0; i < 10; i++) {
			FIELD_FROM_TAG[i] = new Matrix4f();
		}

		final float[][] TAG_COORDINATES = {
				{ 60.25f,   41.41f,  4f   }, // Blue Alliance Left
				{ 60.25f,   35.41f,  4f   }, // Blue Alliance Center
				{ 60.25f,   29.41f,  4f   }, // Blue Alliance Right

				{ 60.25f,  -29.41f,  4f   }, // Red Alliance Left
				{ 60.25f,  -35.41f,  4f   }, // Red Alliance Center
				{ 60.25f,  -41.41f,  4f   }, // Red Alliance Right


				{ -70.25f, -40.625f, 5.5f }, // Red Alliance Wall Large
				{ -70.25f, -35.125f, 4f   }, // Red Alliance Wall Small

				{ -70.25f,  35.125f, 4f   }, // Blue Alliance Wall Small
				{ -70.25f,  40.625f, 5.5f }, // Blue Alliance Wall Large
		};

//		try {
			for (int i = 0; i < 6; i++) {
				FIELD_FROM_TAG[i].loadRotate(30f, 0f, 1f, 0f);
			}
			for (int i = 6; i < 10; i++) {
				FIELD_FROM_TAG[i].loadRotate(180f, 0f, 0f, 1f);
			}
			for (int i = 0; i < 10; i++) {
				for (int j = 0; j < 3; j++) {
					FIELD_FROM_TAG[i].set(3, j, TAG_COORDINATES[i][j]);
				}
			}
//		}
//		catch (Exception e) {
//			Log.e("Matrix Error", e.toString(), e);
////			System.out.println("Error with matrices: " + e.toString());
////			e.printStackTrace();
//		}
	}

	private AprilTagProcessor tagProcessor;
	private VisionPortal visionPortal;
	private Matrix4f cameraFromRobot;

	public RawAprilTagLocalizer(CameraName camera, CameraCalibration calibration, Matrix4f cameraPose) {
		// This is the pose of the camera relative to the robot's center
		cameraFromRobot = cameraPose;
		cameraFromRobot.inverse();

		/*
		Name: SigmaCamA
		Model: Logitech C922 Pro Webcam
		Resolution: 1080p (1920x1080p)
		Date: December 15, 2023

		Camera Matrix:
				[1.43024281e+03 0.00000000e+00 9.60603975e+02
				 0.00000000e+00 1.42898689e+03 5.60364362e+02
				 0.00000000e+00 0.00000000e+00 1.00000000e+00]

		Distortion Coefficients:
			[9.72067503e-02 -6.59705675e-01  3.43162729e-03 -3.87602719e-05 1.90865575e+00]

		Rotation Vectors: ???

		Translation Vectors: ???

		Horizontal Field of View: 67.74018555070795

		Vertical Field of View: 41.402164808619
		*/

		tagProcessor = new AprilTagProcessor.Builder()
				.setLensIntrinsics(1.43024281e+03, 1.42898689e+03, 9.60603975e+02, 5.60364362e+02)
//				.setLensIntrinsics(calibration.focalLengthX, calibration.focalLengthY,
//						calibration.principalPointX, calibration.principalPointY)
				.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
				.setDrawCubeProjection(true)
				.build();

		visionPortal = new VisionPortal.Builder()
				.setCamera(camera)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.setCameraResolution(new Size(1280, 720))
				.addProcessor(tagProcessor)
				.build();

		tagProcessor.setDecimation(1.0f);
	}

	public RawAprilTagLocalizer(CameraName camera, CameraCalibration calibration, double[] position, double[] axis, double angle) {
		this(camera, calibration, cameraPose(position, axis, angle));
	}

	public RawAprilTagLocalizer(CameraName camera, double[] position, double[] axis, double angle) {
		this(camera, null, cameraPose(position, axis, angle));
	}

	private static Matrix4f cameraPose(double[] position, double[] axis, double angle) {
		Matrix4f cameraPose = new Matrix4f();
		cameraPose.loadRotate((float)angle, (float)axis[0], (float)axis[1], (float)axis[2]);
		cameraPose.set(3, 0, (float)position[0]);
		cameraPose.set(3, 1, (float)position[1]);
		cameraPose.set(3, 2, (float)position[2]);
		return cameraPose;
	}

	@NonNull
	public List<Triple<Long, Pose2d, Matrix4f>> getLocalizations() {
		List<AprilTagDetection> detections = tagProcessor.getFreshDetections();
		if (detections == null)
			return new ArrayList<>();

		List<Triple<Long, Pose2d, Matrix4f>> poses = new ArrayList<>();
		for (AprilTagDetection detection : detections) {
			if (detection == null) continue;
			if (detection.rawPose == null) continue;
			if (detection.id < 1 || detection.id > 10) continue;

			Matrix4f tagToCamera = new Matrix4f();
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					tagToCamera.set(i, j, detection.rawPose.R.get(i, j));
				}
			}
			tagToCamera.set(3, 0, (float)detection.rawPose.x);
			tagToCamera.set(3, 1, (float)detection.rawPose.y);
			tagToCamera.set(3, 2, (float)detection.rawPose.z);

			// change translation coordinates
			float x = tagToCamera.get(3, 0);
			float y = tagToCamera.get(3, 1);
			float z = tagToCamera.get(3, 2);
			tagToCamera.set(3, 0, z);
			tagToCamera.set(3, 1, -x);
			tagToCamera.set(3, 2, -y);
			float[][] rotation = new float[3][3];
			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 3; j++) {
					rotation[i][j] = tagToCamera.get(i, j);
				}
			}
			for (int i = 0; i < 3; i++) {
				tagToCamera.set(0, i, rotation[2][i]); // x
			}
			for (int i = 0; i < 3; i++) {
				tagToCamera.set(1, i, -rotation[0][i]); // y
			}
			for (int i = 0; i < 3; i++) {
				tagToCamera.set(2, i, -rotation[1][i]); // z
			}

			Matrix4f fieldFromRobot = multiply(
					FIELD_FROM_TAG[detection.id - 1], inv(tagToCamera), cameraFromRobot
			);

			float robotX = fieldFromRobot.get(3, 0);
			float robotY = fieldFromRobot.get(3, 1);
			float robotHeadingX = fieldFromRobot.get(0, 0);
			float robotHeadingY = fieldFromRobot.get(0, 1);

			Pose2d robotPose =
					new Pose2d(robotX, robotY,
							new Rotation2d(robotHeadingX, robotHeadingY).getRadians());
			poses.add(new Triple<>(detection.frameAcquisitionNanoTime, robotPose, tagToCamera));
		}

		return poses;
	}

	private static Matrix4f multiply(Matrix4f fieldFromTag,
									 Matrix4f tagFromCamera,
									 Matrix4f cameraFromRobot) {
		Matrix4f result = new Matrix4f();
		result.loadMultiply(fieldFromTag, tagFromCamera);
		result.loadMultiply(result, cameraFromRobot);
		return result;
	}

	private static Matrix4f inv(Matrix4f a) {
		Matrix4f result = new Matrix4f();
		result.load(a);
		result.inverse();
		return result;
	}
}
