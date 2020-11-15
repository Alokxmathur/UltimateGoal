package org.firstinspires.ftc.teamcode.robot.components.vision;

import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class WobbleDetector {
    private static final String TAG = "WobbleDetector";
    public static final double thresholdYCoordinate = 99999;
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(12.5,25,25,0);
    private Mat mSpectrum = new Mat();
    private MatOfPoint largestContour = null;

    double maxArea = 0;
    int minY = 9999, minX = 9999;
    int maxY = 0, maxX = 0;

    public int getMinY() {
        return minY;
    }

    public int getMinX() {
        return minX;
    }

    public int getAverageX() {
        return (maxX - minX) / 2;
    }

    public int getAverageY() {
        return (maxY - minY) / 2;
    }

    public int getMaxY() {
        return maxY;
    }

    public int getMaxX() {
        return maxX;
    }

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();

    public WobbleDetector(Scalar color) {
        this.setHsvColor(color);
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
        Log.e(TAG, getBounds());

        Mat spectrumHsv = new Mat(1, (int)(maxH-minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH-minH; j++) {
            byte[] tmp = {(byte)(minH+j), (byte)255, (byte)255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
    }

    public String getBounds() {
        return String.format("%.0f-%.0f,%.0f-%.0f,%.0f-%.0f",
                mLowerBound.val[0], mUpperBound.val[0], mLowerBound.val[1],
                mUpperBound.val[1], mLowerBound.val[2], mUpperBound.val[2]);
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public MatOfPoint process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contoursFound = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mDilatedMask, contoursFound, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        minX = minY = 9999;
        maxX = maxY = 0;
        maxArea = 0;
        for (MatOfPoint contour: contoursFound) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
                Core.multiply(contour, new Scalar(4,4), contour);
                Rect boundingRectangle = Imgproc.boundingRect(contour);
                if (boundingRectangle.y <= thresholdYCoordinate) {
                    largestContour = contour;
                    minX = boundingRectangle.x;
                    maxX = boundingRectangle.x + boundingRectangle.width;
                    minY = boundingRectangle.y;
                    maxY = boundingRectangle.y + boundingRectangle.height;
                }
            }
        }
        return largestContour;
    }

    public MatOfPoint getContour() {
        return largestContour;
    }

    public double getLargestArea() {
        return maxArea;
    }
}
