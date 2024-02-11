/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * There are many "default" VisionPortal and AprilTag configuration parameters that may be overridden if desired.
 * These default parameters are shown as comments in the code below.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTag", group = "Concept")
//@Disabled
public class AprilTagTransforms extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    TelemetryPacket packet = new TelemetryPacket();


    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
                dashboard.sendTelemetryPacket(packet);
                packet = new TelemetryPacket();
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

        // The following default settings are available to un-comment and edit as needed.
            //.setDrawAxes(false)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
            //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

            // == CAMERA CALIBRATION ==
            // If you do not manually specify calibration parameters, the SDK will attempt
            // to load a predefined calibration for your camera
                .setLensIntrinsics(1.43024281e+03, 1.42898689e+03, 9.60603975e+02, 5.60364362e+02)

            // ... these parameters are fx, fy, cx, cy.
                .build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920,1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                packet.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                packet.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                packet.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                packet.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                // create transformation matrix of camera to tag
                VectorF cameraToRobotcenter = new VectorF(0,0,0);
                VectorF cameraToTag = new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z);
                MatrixF rotation = eulerToRotationMatrix(Math.toRadians(detection.ftcPose.pitch), Math.toRadians(detection.ftcPose.roll), Math.toRadians(detection.ftcPose.yaw));
                OpenGLMatrix transformationMatrix = createTransformationMatrix(cameraToRobotcenter, rotation);

                // print transformation matrix
                packet.put("Transformation Matrix:", transformationMatrix.toString());

                // get transformation of tag to origin
//                detection.metadata.fieldPosition = VectorF, detection.metadata.fieldOrientation = Quaternion
                VectorF originToTag = detection.metadata.fieldPosition;
                packet.put("Tag to Origin:", originToTag.toString());

                MatrixF tagRotation = eulerToRotationMatrix(Math.toRadians(-30),0,0);

                originToTag = tagRotation.multiplied(originToTag);
                packet.put("Tag to Origin Rotated:", originToTag.toString());

                // get transformation of camera to origin
                VectorF centerToTag = transformationMatrix.multiplied(cameraToTag);
                packet.put("Center to Tag:", centerToTag.toString());

                // get transformation of origin to camera
                VectorF originToCenter = centerToTag.subtracted(originToTag);

                packet.addLine(String.format("Camera Position: %6.1f %6.1f", originToCenter.get(0), originToCenter.get(1)));

                packet.fieldOverlay()
                        .setStroke("green")
                        .strokeRect(-originToCenter.get(0) + 2.5, -originToCenter.get(1) + 2.5, 5, 5)
                        .setStroke("red")
                        .strokeRect(detection.metadata.fieldPosition.get(0) -2.5, detection.metadata.fieldPosition.get(1) - 2.5, 5, 5);
            }
            else {
                packet.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                packet.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        packet.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        packet.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        packet.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    public static MatrixF eulerToRotationMatrix(double pitch, double roll, double yaw) {
        // Pre-calculate the sine and cosine of the angles
        double sinPitch = Math.sin(pitch);
        double cosPitch = Math.cos(pitch);
        double sinRoll = Math.sin(roll);
        double cosRoll = Math.cos(roll);
        double sinYaw = Math.sin(yaw);
        double cosYaw = Math.cos(yaw);

        // Create the individual rotation matrices for each axis

        MatrixF xRotation = new GeneralMatrixF(3, 3, new float[]{
                1, 0, 0,
                0, (float) cosPitch, (float) -sinPitch,
                0, (float) sinPitch, (float) cosPitch
        });

        MatrixF yRotation = new GeneralMatrixF(3, 3, new float[]{
                (float) cosRoll, 0, (float) sinRoll,
                0, 1, 0,
                (float) -sinRoll, 0, (float) cosRoll
        });

        MatrixF zRotation = new GeneralMatrixF(3, 3, new float[]{
                (float) cosYaw, (float) -sinYaw, 0,
                (float) sinYaw, (float) cosYaw, 0,
                0, 0, 1
        });


        // Combine the rotations (Z * Y * X)
        return zRotation.multiplied(yRotation.multiplied(xRotation));
    }
    // Creates a homogeneous transformation matrix from a translation vector and a rotation matrix
    public static OpenGLMatrix createTransformationMatrix(VectorF translation, MatrixF rotation) {
        OpenGLMatrix transformationMatrix = new OpenGLMatrix(rotation);
        transformationMatrix.translate(translation.get(0), translation.get(1), translation.get(2));
        return transformationMatrix;
    }

    private static double[] vectorFToDoubleArray(VectorF vectorF) {
        float[] floatArray = vectorF.getData();
        double[] doubleArray = new double[floatArray.length];

        for (int i = 0; i < floatArray.length; i++) {
            doubleArray[i] = floatArray[i]; // Implicit casting
        }

        return doubleArray;
    }

}   // end class
