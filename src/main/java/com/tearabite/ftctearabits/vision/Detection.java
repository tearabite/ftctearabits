package com.tearabite.ftctearabits.vision;

import static com.tearabite.ftctearabits.vision.OpenCVUtil.getCenterOfContour;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

/**
 * This class is used to represent a detection from the a VisionProcessor pipeline.
 * It contains various useful abstraction methods for interacting with the detection.
 */
@NoArgsConstructor()
@AllArgsConstructor
@Builder
public class Detection {
    public static final Point INVALID_POINT = new Point(Double.MIN_VALUE, Double.MIN_VALUE);
    public static final double INVALID_AREA = -1;

    @Getter @Setter private MatOfPoint contour;
    @Getter @Setter private Size frameSize;
    @Getter @Setter private double maxAreaThreshold;
    @Getter @Setter private double minAreaThreshold;

    /**
     * Returns whether the detection is valid or not.
     * A detection is considered valid if it has a contour and its area is within
     * the min and max area thresholds.
     */
    public boolean isValid() {
        double area = getArea();
        return contour != null
                && area > minAreaThreshold
                && area < maxAreaThreshold;
    }

    /**
     * Returns the area of the detection in the specified scale.
     * @param scale The scale to return the area in
     * @return The area of the detection
     */
    public double getArea(PropertyScale scale) {
        if (!isValid()) {
            return INVALID_AREA;
        }

        double areaPx = Imgproc.contourArea(contour);
        if (scale == PropertyScale.Pixels) {
            return areaPx;
        }

        return (areaPx / (frameSize.width * frameSize.height)) * 100;
    }

    /**
     * Returns the area of the detection in pixels.
     * @return The pixel area of the detection
     */
    public double getArea() {
        return getArea(PropertyScale.Pixels);
    }

    /**
     * Returns the center of the detection in the specified scale.
     * @param scale The scale to return the center in
     * @return The center of the detection
     */
    public Point getCenter(PropertyScale scale) {
        if (!isValid()) {
            return INVALID_POINT;
        }

        Point centerPx = getCenterOfContour(this.contour);
        if (scale == PropertyScale.Pixels) {
            return centerPx;
        }

        return pixelPointToPercentageOfFrame(centerPx);
    }

    /**
     * Returns the pixel center of the detection.
     * @return The pixel center of the detection
     */
    public Point getCenter() {
        return getCenter(PropertyScale.Pixels);
    }

    /**
     * Sets the maximum area threshold for the detection.
     * @param maximumAreaThreshold The maximum area threshold
     * @param scale The scale that maximumAreaThreshold is specified in
     */
    public void setMaximumAreaThreshold(double maximumAreaThreshold, PropertyScale scale) {
        switch (scale) {
            case Pixels:
                this.maxAreaThreshold = maximumAreaThreshold;
                break;
            case Percent:
                this.maxAreaThreshold = frameSize.area() * maximumAreaThreshold;
                break;
        }
    }

    /**
     * Sets the maximum area threshold for the detection in pixels.
     * @param maximumAreaThreshold The maximum area threshold
     */
    public void setMaximumAreaThreshold(double maximumAreaThreshold) {
        setMaximumAreaThreshold(maximumAreaThreshold, PropertyScale.Pixels);
    }

    /**
     * Sets the minimum area threshold for the detection.
     * @param minimumAreaThreshold The minimum area threshold
     * @param scale The scale that minimumAreaThreshold is specified in
     */
    public void setMinimumAreaThreshold(double minimumAreaThreshold, PropertyScale scale) {
        switch (scale) {
            case Pixels:
                this.minAreaThreshold = minimumAreaThreshold;
                break;
            case Percent:
                this.minAreaThreshold = frameSize.area() * minimumAreaThreshold;
                break;
        }
    }

    /**
     * Sets the minimum area threshold for the detection in pixels.
     * @param minimumAreaThreshold The minimum area threshold
     */
    public void setMinimumAreaThreshold(double minimumAreaThreshold) {
        setMinimumAreaThreshold(minimumAreaThreshold, PropertyScale.Pixels);
    }

    /**
     * Returns the maximum area threshold for the detection in the specified scale.
     * @param scale The scale to return the maximum area threshold in
     * @return The maximum area threshold
     */
    public double getMaximumAreaThreshold(PropertyScale scale) {
        switch (scale) {
            default:
            case Pixels:
                return this.maxAreaThreshold;
            case Percent:
                return this.maxAreaThreshold / this.frameSize.area();
        }
    }

    /**
     * Returns the maximum area threshold for the detection in pixels.
     * @return The maximum area threshold
     */
    public double getMaximumAreaThreshold() {
        return getMaximumAreaThreshold(PropertyScale.Pixels);
    }

    /**
     * Returns the minimum area threshold for the detection in the specified scale.
     * @param scale The scale to return the minimum area threshold in
     * @return The minimum area threshold
     */
    public double getMinimumAreaThreshold(PropertyScale scale) {
        switch (scale) {
            default:
            case Pixels:
                return this.minAreaThreshold;
            case Percent:
                return this.minAreaThreshold / this.frameSize.area();
        }
    }

    /**
     * Returns the minimum area threshold for the detection in pixels.
     * @return The minimum area threshold
     */
    public double getMinimumAreaThreshold() {
        return getMinimumAreaThreshold(PropertyScale.Pixels);
    }

    public enum PropertyScale { Pixels, Percent }

    /**
     * Converts a pixel point to a percentage of the frame.
     * @param pixelPoint The pixel point to convert
     * @return The percentage of the frame that the pixel point is at
     */
    private Point pixelPointToPercentageOfFrame(Point pixelPoint) {
        double normalizedX = ((pixelPoint.x / frameSize.width) * 100) - 50;
        double normalizedY = ((pixelPoint.y / frameSize.height) * -100) + 50;
        return new Point(normalizedX, normalizedY);
    }
}