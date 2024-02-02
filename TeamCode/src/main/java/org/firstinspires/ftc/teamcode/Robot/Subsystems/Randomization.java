package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import static org.opencv.core.Core.mean;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.CommandFramework.Subsystem;
import org.firstinspires.ftc.teamcode.Utils.RandomizationSide;
import org.firstinspires.ftc.teamcode.Utils.Side;
import org.firstinspires.ftc.teamcode.Utils.Team;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Randomization extends Subsystem {
    private VisionPortal visionPortal;
    private PropDetector propDetector;
    private final Team team;
    private boolean closed = false;

    public Randomization(Team team) {
        this.team = team;

    }

    @Override
    public void initAuto(HardwareMap hwMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        propDetector = new PropDetector(team);


        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(propDetector);

        visionPortal = builder.build();
    }


    @Override
    public void periodic() {

    }

    @Override
    public void shutdown() {

    }

    public RandomizationSide getRandomizationSide() {
        return propDetector.getRandomization();
    }

    public void closePortal() {
        visionPortal.close();
    }
}


class Data {
    Rect leftrect;
    Rect centerrect;
    Rect rightrect;

    RandomizationSide randomization;

    public Data(Rect leftrect, Rect centerrect, Rect rightrect, RandomizationSide randomization) {
        this.leftrect = leftrect;
        this.centerrect = centerrect;
        this.rightrect = rightrect;
        this.randomization = randomization;
    }
}


class PropDetector implements VisionProcessor {
    public RandomizationSide randomization = RandomizationSide.NOT_ASSIGNED;
    private int width;
    private int height;
    private Team team;

    public PropDetector(Team team) {
        this.team = team;
    }


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

        /*Rect leftrect = new Rect(0, 185, 100, 100);
        Rect centerrect = new Rect(230, 170, 160, 100);
        Rect rightrect = new Rect(550, 185, 88, 100);*/

        Rect leftrect = new Rect(20, 227, 30, 30);
        Rect centerrect = new Rect(290, 222, 40, 30);
        Rect rightrect = new Rect(590, 227, 30, 30);

        Double leftvalue = mean(new Mat(hsvImage, leftrect)).val[1];
        Double centervalue = mean(new Mat(hsvImage, centerrect)).val[1];
        Double rightvalue = mean(new Mat(hsvImage, rightrect)).val[1];

        if (leftvalue > rightvalue && leftvalue > centervalue){
            randomization = RandomizationSide.LEFT;
        }
        else if (rightvalue > centervalue){
            randomization = RandomizationSide.RIGHT;
        }
        else{
            randomization = RandomizationSide.CENTER;
        }

        Data data = new Data(leftrect, centerrect, rightrect, randomization);
        return data;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

//        if (userContext instanceof Rect) {
//            Rect rect = (Rect) userContext;
//
//            // Calculate the scaling factors
//            float scaleX = (float) onscreenWidth / width;
//            float scaleY = (float) onscreenHeight / height;
//
//            Paint paint = new Paint();
//            paint.setColor(Color.GREEN); // Use Color.GREEN for the color constant
//            paint.setStyle(Paint.Style.STROKE);
//            paint.setStrokeWidth(3 * scaleCanvasDensity); // Adjust stroke width for screen density
//
//            // Scale and draw the rectangle
//            canvas.drawRect(
//                    rect.x * scaleX,
//                    rect.y * scaleY,
//                    (rect.x + rect.width) * scaleX,
//                    (rect.y + rect.height) * scaleY,
//                    paint
//            );
//
//            // Optionally, draw position information
//            paint.setColor(Color.RED); // Use Color.RED for the color constant
//            paint.setTextSize(40 * scaleCanvasDensity); // Adjust text size for screen density
//
//            // Determine the position to draw the text
//            float textX = rect.x * scaleX;
//            float textY = (rect.y + rect.height + 40) * scaleY; // Adjust to draw below the rectangle
//
//            canvas.drawText(randomization.name(), textX, textY, paint);
//        }

        if (userContext instanceof Data) {
            Data data = (Data) userContext;

            // Calculate the scaling factors
            float scaleX = (float) onscreenWidth / width;
            float scaleY = (float) onscreenHeight / height;

            Paint paint = new Paint();
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(3 * scaleCanvasDensity); // Adjust stroke width for screen density

            if (data.randomization == RandomizationSide.LEFT) {
                paint.setColor(Color.GREEN);
            } else {
                paint.setColor(Color.RED);
            }

            // Scale and draw the rectangle
            canvas.drawRect(
                    data.leftrect.x * scaleX,
                    data.leftrect.y * scaleY,
                    (data.leftrect.x + data.leftrect.width) * scaleX,
                    (data.leftrect.y + data.leftrect.height) * scaleY,
                    paint
            );

            if (data.randomization == RandomizationSide.CENTER) {
                paint.setColor(Color.GREEN);
            } else {
                paint.setColor(Color.RED);
            }

            canvas.drawRect(
                    data.centerrect.x * scaleX,
                    data.centerrect.y * scaleY,
                    (data.centerrect.x + data.centerrect.width) * scaleX,
                    (data.centerrect.y + data.centerrect.height) * scaleY,
                    paint
            );

            if (data.randomization == RandomizationSide.RIGHT) {
                paint.setColor(Color.GREEN);
            } else {
                paint.setColor(Color.RED);
            }


            canvas.drawRect(
                    data.rightrect.x * scaleX,
                    data.rightrect.y * scaleY,
                    (data.rightrect.x + data.rightrect.width) * scaleX,
                    (data.rightrect.y + data.rightrect.height) * scaleY,
                    paint
            );

            // Optionally, draw position information
            paint.setColor(Color.RED); // Use Color.RED for the color constant
            paint.setTextSize(40 * scaleCanvasDensity); // Adjust text size for screen density

            // Determine the position to draw the text
            float textX = data.leftrect.x * scaleX;
            float textY = (data.leftrect.y + data.leftrect.height + 40) * scaleY; // Adjust to draw below the rectangle

            canvas.drawText(data.randomization.name(), textX, textY, paint);
        }
    }

    public RandomizationSide getRandomization() {
        return randomization;
    }
}