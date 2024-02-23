package org.firstinspires.ftc.teamcode.Roadrunner.Drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Roadrunner.Util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class FusedLocalizer extends TwoTrackingWheelLocalizer {
	private static final double KALMAN_GAIN = 0.04;
	public static double TICKS_PER_REV = 2000;
	public static double WHEEL_RADIUS = 0.9448818898; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	// TODO: update soon
	public static double PARALLEL_X = -5.408; // X is the up and down direction
	public static double PARALLEL_Y = -2.046; // Y is the strafe direction

	// TODO: update soon
	public static double PERPENDICULAR_X = -7.400;
	public static double PERPENDICULAR_Y = -2.545;

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private Encoder parallelEncoder, perpendicularEncoder;
	private List<DcMotorEx> motors;

	private SampleMecanumDrive drive;

	private double previousForwardEncoder;
	private double[] previousMotorPositions;

	public FusedLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
		super(Arrays.asList(
				new Pose2d(PARALLEL_X, PARALLEL_Y, Math.toRadians(180)),
				new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(-90))
		));

		this.drive = drive;

		parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "hang"));
		perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "roller"));

		previousForwardEncoder = encoderTicksToInches(parallelEncoder.getCurrentPosition());

		motors = Arrays.asList(
				hardwareMap.get(DcMotorEx.class, "leftFront"),
				hardwareMap.get(DcMotorEx.class, "leftBack"),
				hardwareMap.get(DcMotorEx.class, "rightBack"),
				hardwareMap.get(DcMotorEx.class, "rightFront")
		);

		previousMotorPositions = new double[4];
		for (int i = 0; i < 4; i++) {
			previousMotorPositions[i] =
					DriveConstants.encoderTicksToInches(motors.get(i).getCurrentPosition());
		}
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading() {
		return drive.getRawExternalHeading();
	}

	@Override
	public Double getHeadingVelocity() {
		return drive.getExternalHeadingVelocity();
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		double forward = encoderTicksToInches(parallelEncoder.getCurrentPosition());
		double encoderForwardDelta = forward - previousForwardEncoder;
		double driveForwardDelta = 0;
		for (int i = 0; i < 4; i++) {
			double motorPosition =
					DriveConstants.encoderTicksToInches(motors.get(i).getCurrentPosition());
			driveForwardDelta += motorPosition - previousMotorPositions[i];
			previousMotorPositions[i] = motorPosition;
		}
		driveForwardDelta /= 4;
		double forwardDelta =
				(1 - KALMAN_GAIN) * encoderForwardDelta + KALMAN_GAIN * driveForwardDelta;
		double filteredForward = previousForwardEncoder + forwardDelta;
		previousForwardEncoder = forward;

		return Arrays.asList(
				filteredForward,
				encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities() {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getRawVelocity()),
				encoderTicksToInches(perpendicularEncoder.getRawVelocity())
		);
	}
}