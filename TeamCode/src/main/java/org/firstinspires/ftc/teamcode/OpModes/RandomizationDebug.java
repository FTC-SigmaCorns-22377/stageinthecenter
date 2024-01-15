/* Copyright (c) 2019 FIRST. All rights reserved.
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

import static android.graphics.Color.*;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp
public class RandomizationDebug extends LinearOpMode {
    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private PropDetector propDetector;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        propDetector = new PropDetector();

        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);


        builder.addProcessor(propDetector);
        visionPortal = builder.build();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Randomization randomization = propDetector.randomization;
                telemetry.addData("Randomization", randomization.name());

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Share the CPU.
                sleep(20);
            }
        }
        visionPortal.close();
    }
}


enum Randomization {
    LEFT,
    RIGHT,
    CENTER,
    NOT_ASSIGNED
}

class PropDetector implements VisionProcessor {
    public Randomization randomization = Randomization.NOT_ASSIGNED;
    private int width;
    private int height;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        System.out.println("processed frame");
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(frame, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define the red color range in HSV
        Scalar lowerRed = new Scalar(0, 120, 70);
        Scalar upperRed = new Scalar(180, 255, 255);
        Mat redMask = new Mat();

        // Create a mask for red color
        Core.inRange(hsvImage, lowerRed, upperRed, redMask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        double maxArea = 0;
        Rect largestRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestRect = Imgproc.boundingRect(contour);
            }
        }

        if (largestRect != null) {
            int centerX = largestRect.x + largestRect.width / 2;
            int third = frame.width() / 3;

            if (centerX < third) {
                randomization = Randomization.LEFT;
            } else if (centerX > 2 * third) {
                randomization = Randomization.RIGHT;
            } else {
                randomization = Randomization.CENTER;
            }
        }

        return largestRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if (userContext instanceof Rect) {
            Rect rect = (Rect) userContext;

            // Calculate the scaling factors
            float scaleX = (float) onscreenWidth / width;
            float scaleY = (float) onscreenHeight / height;

            Paint paint = new Paint();
            paint.setColor(Color.GREEN); // Use Color.GREEN for the color constant
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(3 * scaleCanvasDensity); // Adjust stroke width for screen density

            // Scale and draw the rectangle
            canvas.drawRect(
                    rect.x * scaleX,
                    rect.y * scaleY,
                    (rect.x + rect.width) * scaleX,
                    (rect.y + rect.height) * scaleY,
                    paint
            );

            // Optionally, draw position information
            paint.setColor(Color.RED); // Use Color.RED for the color constant
            paint.setTextSize(40 * scaleCanvasDensity); // Adjust text size for screen density

            // Determine the position to draw the text
            float textX = rect.x * scaleX;
            float textY = (rect.y + rect.height + 40) * scaleY; // Adjust to draw below the rectangle

            canvas.drawText(randomization.name(), textX, textY, paint);
        }
    }
}