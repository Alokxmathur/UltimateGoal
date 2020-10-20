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

package org.firstinspires.ftc.teamcode.robot.components.camera;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RingDetector {
    public RingDetector() {
    }

    public RingDetector(int x, int y) {
        rectangleBottomLeft.x = x;
        rectangleBottomLeft.y = y;
    }

    /*
     * The core values which define the location and size of the rectangle
     */
    static Point rectangleBottomLeft = new Point(0, 0);

    static volatile int rectangleHeight = 70;
    static volatile int rectangleWidth = 90;

    int fourRingThreshold = 154;
    int oneRingThreshold = 137;

    /*
     * Working variables
     */
    Mat inputMat = new Mat();
    Mat rectangle_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int redAverage;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     *
     * It then extracts the rectangle from the Cb variable and finds the average red value in it
     */
    public Field.RingCount getNumberOfRings(Bitmap inputBitmap) {
        Utils.bitmapToMat(inputBitmap, inputMat);
        Imgproc.cvtColor(inputMat, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
        rectangle_Cb = Cb.submat(new Rect(
                new Point(rectangleBottomLeft.x, rectangleBottomLeft.y),
                new Point(rectangleBottomLeft.x + rectangleWidth, rectangleBottomLeft.y + rectangleHeight)));
        redAverage = (int) Core.mean(rectangle_Cb).val[0];
        Imgproc.rectangle(
                inputMat, // Buffer to draw on
                new Point(rectangleBottomLeft.x, rectangleBottomLeft.y), // First point which defines the rectangle
                new Point(rectangleBottomLeft.x + rectangleWidth, rectangleBottomLeft.y + rectangleHeight), // Second point which defines the rectangle
                new Scalar(255, 0, 0), // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        Field.RingCount ringCount = Field.RingCount.FOUR; // Record our analysis
        if (redAverage > fourRingThreshold) {
            ringCount = Field.RingCount.FOUR;
        } else if (redAverage > oneRingThreshold) {
            ringCount = Field.RingCount.ONE;
        } else {
            ringCount = Field.RingCount.NONE;
        }
        Match.log("Average red count=" + redAverage);

        return ringCount;

    }
}