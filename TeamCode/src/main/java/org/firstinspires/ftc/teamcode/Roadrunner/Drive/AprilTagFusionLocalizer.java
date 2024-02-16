package org.firstinspires.ftc.teamcode.Roadrunner.Drive;

import android.renderscript.Matrix4f;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.Math.Geometry.Twist2d;
import org.firstinspires.ftc.teamcode.Utils.RawAprilTagLocalizer;
import org.opencv.core.Mat;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

import kotlin.Pair;
import kotlin.Triple;

public class AprilTagFusionLocalizer implements Localizer {
	private static final double kalmanGain = 0.5; // higher values give more weight to AprilTags
	private Localizer odometryLocalizer;
	private RawAprilTagLocalizer aprilTagLocalizer;
	private OverwriteQueue<Pair<Long, Pose2d>> poseHistory = new OverwriteQueue<>(64);
	private Pose2d poseVelocity = new Pose2d();

	public AprilTagFusionLocalizer(Localizer odometryLocalizer,
								   RawAprilTagLocalizer aprilTagLocalizer) {
		this.odometryLocalizer = odometryLocalizer;
		this.aprilTagLocalizer = aprilTagLocalizer;
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate() {
		return poseHistory.get(0).getSecond();
	}

	@Override
	public void setPoseEstimate(@NonNull Pose2d pose2d) {
		poseHistory.push(new Pair<>(System.nanoTime(), pose2d));
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity() {
		return poseVelocity;
	}

	@Override
	public void update() {
		odometryLocalizer.update();
		poseHistory.push(new Pair<>(System.nanoTime(), odometryLocalizer.getPoseEstimate()));
		List<Triple<Long, Pose2d, Matrix4f>> detections = aprilTagLocalizer.getLocalizations();
		if (detections.size() > 0) {
			ArrayList<Pose2d> tagPoses = new ArrayList<>();
			tagPoses.ensureCapacity(detections.size());
			Pose2d preUpdatePose = getPoseEstimate();
			for (Triple<Long, Pose2d, Matrix4f> detection : detections) {
				Pose2d pastPose = getPoseAtTime(detection.getFirst());
				if (pastPose == null) continue;
				tagPoses.add(detection.getSecond().minus(pastPose).plus(preUpdatePose));
			}
			double odometryWeight = Math.pow(1 - kalmanGain, detections.size());
			Pose2d tagPose = new Pose2d();
			for (Pose2d pose : tagPoses) {
				tagPose = tagPose.plus(pose);
			}
			tagPose = tagPose.times((1 - odometryWeight) / detections.size());
			Pose2d fusedPose = preUpdatePose.times(odometryWeight).plus(tagPose);
			Pair<Long, Pose2d> fusedPair =
					new Pair<>(poseHistory.get(0).getFirst(), fusedPose);
			poseHistory.set(0, fusedPair);
		}
	}

	@Nullable
	private Pose2d getPoseAtTime(long nanoTime) {
		for (int i = 0; i < poseHistory.size(); i++) {
			if (poseHistory.get(i).getFirst() > nanoTime) continue;

			if (i == 0) return getPoseEstimate(); // more recent than most recent data
			if (nanoTime == poseHistory.get(i).getFirst()) return poseHistory.get(i).getSecond();

			Pair<Long, Pose2d> before = poseHistory.get(i);
			Pair<Long, Pose2d> after = poseHistory.get(i - 1);
			double alpha = (nanoTime - before.getFirst()) /
					(double)(after.getFirst() - before.getFirst());

			Pose2d beforePose = before.getSecond();
			Pose2d afterPose = after.getSecond();
			return afterPose.minus(beforePose).times(alpha).plus(beforePose);
		}
		return null; // older than oldest data
	}
}

class OverwriteQueue<T> {
	private final Object[] buffer;
	private int head;
	private int size;

	public OverwriteQueue(int capacity) {
		buffer = new Object[capacity];
		head = capacity;
		size = 0;
	}

	public int size() {
		return size;
	}

	public T get(int index) {
		if (index < 0 || index >= size) {
			throw new IndexOutOfBoundsException();
		}
		// noinspection unchecked
		return (T)buffer[(head + index) % buffer.length];
	}

	public void set(int index, T value) {
		if (index < 0 || index >= size) {
			throw new IndexOutOfBoundsException();
		}
		buffer[(head + index) % buffer.length] = value;
	}

	public void push(T value) {
		if (size < buffer.length) {
			size++;
		}
		head--;
		if (head == -1) {
			head = buffer.length - 1;
		}
		buffer[head] = value;
	}
}