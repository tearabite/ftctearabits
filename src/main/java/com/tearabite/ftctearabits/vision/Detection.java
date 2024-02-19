package com.tearabite.ftctearabits.vision;

import static com.tearabite.ftctearabits.vision.Constants.GREEN;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.drawConvexHull;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.drawPoint;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.fillConvexHull;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.getBottomLeftOfContour;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.getBottomRightOfContour;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.getCenterOfContour;

import android.graphics.drawable.Icon;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

// Class for a Detection
public class Detection {
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;
    public static final Detection INVALID_DETECTION = new Detection(new Size(0, 0), 0);

    private double minAreaPx;
    private final double maxAreaPx;
    private final Size maxSizePx;
    private double areaPx =  INVALID_AREA;
    private Point centerPx = INVALID_POINT;
    private Point bottomLeftPx = INVALID_POINT;
    private Point bottomRightPx = INVALID_POINT;
    private MatOfPoint contour;

    // Constructor
    public Detection(Size frameSize, double minAreaFactor) {
        this.maxSizePx = frameSize;
        this.minAreaPx = frameSize.area() * minAreaFactor;
        this.maxAreaPx = frameSize.area();
    }

    public Detection(Size frameSize, double minAreaFactor, double maxAreaFactor) {
        this.maxSizePx = frameSize;
        this.minAreaPx = frameSize.area() * minAreaFactor;
        this.maxAreaPx = frameSize.area() * maxAreaFactor;
    }

    public void setMinArea(double minAreaFactor) {
        this.minAreaPx = maxSizePx.area() * minAreaFactor;
    }

    public void setMaxArea(double maxAreaFactor) {
        this.minAreaPx = maxSizePx.area() * maxAreaFactor;
    }

    // Draw a convex hull around the current detection on the given image
    public void draw(Mat img, Scalar color) {
        if (isValid()) {
            drawConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    // Draw a convex hull around the current detection on the given image
    public void fill(Mat img, Scalar color) {
        if (isValid()) {
            fillConvexHull(img, contour, color);
            drawPoint(img, centerPx, GREEN);
        }
    }

    // Check if the current Detection is valid
    public boolean isValid() {
        return (this.contour != null) && (this.areaPx != INVALID_AREA);
    }

    // Get the current contour
    public MatOfPoint getContour() {
        return contour;
    }

    // Set the values of the current contour
    public void setContour(MatOfPoint contour) {
        this.contour = contour;

        double area;
        if (contour != null && (area = Imgproc.contourArea(contour)) > minAreaPx && area < maxAreaPx) {
            this.areaPx = area;
            this.centerPx = getCenterOfContour(contour);
            this.bottomLeftPx = getBottomLeftOfContour(contour);
            this.bottomRightPx = getBottomRightOfContour(contour);
        } else {
            this.areaPx = INVALID_AREA;
            this.centerPx = INVALID_POINT;
            this.bottomLeftPx = INVALID_POINT;
            this.bottomRightPx = INVALID_POINT;
        }
    }

    // Returns the center of the Detection, normalized so that the width and height of the frame is from [-50,50]
    public Point getCenter() {
        if (!isValid()) {
            return INVALID_POINT;
        }

        double normalizedX = ((centerPx.x / maxSizePx.width) * 100) - 50;
        double normalizedY = ((centerPx.y / maxSizePx.height) * -100) + 50;

        return new Point(normalizedX, normalizedY);
    }

    // Get the center point in pixels
    public Point getCenterPx() {
        return centerPx;
    }

    // Get the area of the Detection, normalized so that the area of the frame is 100
    public double getArea() {
        if (!isValid()) {
            return INVALID_AREA;
        }

        return (areaPx / (maxSizePx.width * maxSizePx.height)) * 100;
    }

    // Get the leftmost bottom corner of the detection
    public Point getBottomLeftCornerPx() {
        return bottomLeftPx;
    }

    // Get the rightmost bottom corner of the detection
    public Point getBottomRightCornerPx() {
        return bottomRightPx;
    }

    public void setIgnoreSmallerThan(double ignoreSmallerThan) {
        this.minAreaPx = maxSizePx.area() * ignoreSmallerThan;
    }

    public void setIgnoreLargerThan(double ignoreLargerThan) {
        this.minAreaPx = maxSizePx.area() * ignoreLargerThan;
    }

    public double getIgnoreSmallerThan() {
         return this.minAreaPx / this.maxSizePx.area();
    }

    public double getIgnoreLargerThan() {
        return this.maxAreaPx / this.maxSizePx.area();
    }
}