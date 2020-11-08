/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drivercontrolled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.camera.WobbleDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class WobbleAndRingDetermination extends LinearOpMode {
    OpenCvCamera webcam;
    WobbleAndRingDeterminationPipeline pipeline;
    public static final Object synchronizer = new Object();

    public WobbleAndRingDetermination(int x, int y) {
        pipeline = new WobbleAndRingDeterminationPipeline(x, y);
    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline.setTelemetry(telemetry);
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(() -> webcam.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT));

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                pipeline.decrementHeight();
            }
            if (gamepad1.dpad_up) {
                pipeline.incrementHeight();
            }
            if (gamepad1.dpad_left) {
                pipeline.decrementWidth();
            }
            if (gamepad1.dpad_right) {
                pipeline.incrementWidth();
            }
            if (gamepad1.left_stick_y > .2) {
                pipeline.moveUp();
            }
            if (gamepad1.left_stick_y < -.2) {
                pipeline.moveDown();
            }
            if (gamepad1.left_stick_x > .2) {
                pipeline.moveRight();
            }
            if (gamepad1.left_stick_x < -.2) {
                pipeline.moveLeft();
            }

            telemetry.addData("RedCount", pipeline.getRedAverage());
            telemetry.addData("RingCount", pipeline.ringCount);
            telemetry.addData("Position: ", pipeline.getRectangle());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class WobbleAndRingDeterminationPipeline extends OpenCvPipeline {
        //the color of the black part of the wobble goal
        Scalar wobbleBaseColor = new Scalar(0);
        Telemetry telemetry;
        public WobbleAndRingDeterminationPipeline(int x, int y) {
            ringStackBottomLeft.x = x;
            ringStackBottomLeft.y = y;
        }
        public void setTelemetry(Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /*
         * Red, Blue and Green color constants
         */
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        /*
         * The core values which define the location and size of the rectangle
         */
        static Point ringStackBottomLeft = new Point(124, 160);

        static volatile int rectangleHeight = 70;
        static volatile int rectangleWidth = 90;

        int fourRingThreshold = 154;
        int oneRingThreshold = 137;

        /*
         * Working variables
         */
        Mat rectangle_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int redAverage;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile Field.RingCount ringCount = Field.RingCount.FOUR;

        private WobbleDetector wobbleDetector = new WobbleDetector(new Scalar(0));

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         *
         * It then extracts the rectangle from the Cb variable and finds the average red value in it
         */
        void processInput(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
            rectangle_Cb = Cb.submat(new Rect(
                    new Point(ringStackBottomLeft.x, ringStackBottomLeft.y),
                    new Point(ringStackBottomLeft.x + rectangleWidth, ringStackBottomLeft.y + rectangleHeight)));
            redAverage = (int) Core.mean(rectangle_Cb).val[0];
        }


        @Override
        public Mat processFrame(Mat input) {
            synchronized (synchronizer) {
                processInput(input);
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        new Point(ringStackBottomLeft.x, ringStackBottomLeft.y), // First point which defines the rectangle
                        new Point(ringStackBottomLeft.x + rectangleWidth, ringStackBottomLeft.y + rectangleHeight), // Second point which defines the rectangle
                        BLUE, // The color the rectangle is drawn in
                        -1); // Thickness of the rectangle lines
                MatOfPoint wobbleContour = wobbleDetector.process(input);
                if (wobbleContour != null) {
                    ArrayList<MatOfPoint> contours = new ArrayList<>();
                    contours.add(wobbleDetector.process(input));
                    Imgproc.drawContours(input, contours, -1, RED, -1);
                }
                Imgproc.putText(input, "" + wobbleDetector.getBounds(), new Point(4, 50),
                        Imgproc.FONT_HERSHEY_COMPLEX, 1, wobbleBaseColor, 4);
                Imgproc.putText(input, "Area: " + wobbleDetector.getLargestArea(), new Point(4, 100),
                        Imgproc.FONT_HERSHEY_COMPLEX, 1, wobbleBaseColor, 4);
                Imgproc.putText(input, "x:" + wobbleDetector.getMinX() + "-" + wobbleDetector.getMaxX(), new Point(4, 150),
                        Imgproc.FONT_HERSHEY_COMPLEX, 1, wobbleBaseColor, 4);
                Imgproc.putText(input, "y:" + wobbleDetector.getMinY() + "-" + wobbleDetector.getMaxY(), new Point(4, 200),
                        Imgproc.FONT_HERSHEY_COMPLEX, 1, wobbleBaseColor, 4);
                telemetry.addData("Wobble","A:" + wobbleDetector.getLargestArea() + "x:" + wobbleDetector.getAverageX()
                    + "y:" + wobbleDetector.getAverageY());
                telemetry.update();

                ringCount = Field.RingCount.FOUR; // Record our analysis
                if (redAverage > fourRingThreshold) {
                    ringCount = Field.RingCount.FOUR;
                } else if (redAverage > oneRingThreshold) {
                    ringCount = Field.RingCount.ONE;
                } else {
                    ringCount = Field.RingCount.NONE;
                }

                return input;
            }
        }

        public int getRedAverage() {
            synchronized (synchronizer) {
                return redAverage;
            }
        }

        public String getRectangle() {
            synchronized (synchronizer) {
                return "(" + ringStackBottomLeft.x + "," + ringStackBottomLeft.y + "), h:" + rectangleHeight + ", w:" + rectangleWidth;
            }
        }

        public void incrementHeight() {
            synchronized (synchronizer) {
                rectangleHeight++;
            }
        }

        public void decrementHeight() {
            synchronized (synchronizer) {
                rectangleHeight--;
            }
        }

        public void incrementWidth() {
            synchronized (synchronizer) {
                rectangleWidth++;
            }
        }

        public void decrementWidth() {
            synchronized (synchronizer) {
                rectangleWidth--;
            }
        }

        public void moveUp() {
            synchronized (synchronizer) {
                ringStackBottomLeft.y++;
            }
        }

        public void moveDown() {
            synchronized (synchronizer) {
                ringStackBottomLeft.y--;
            }
        }

        public void moveLeft() {
            synchronized (synchronizer) {
                ringStackBottomLeft.x--;
            }
        }

        public void moveRight() {
            synchronized (synchronizer) {
                ringStackBottomLeft.x++;
            }
        }
    }
}