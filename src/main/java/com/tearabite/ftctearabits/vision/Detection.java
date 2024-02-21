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

    public boolean isValid() {
        double area = getArea();
        return contour != null
                && area > minAreaThreshold
                && area < maxAreaThreshold;
    }

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

    public double getArea() {
        return getArea(PropertyScale.Pixels);
    }

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

    public Point getCenter() {
        return getCenter(PropertyScale.Pixels);
    }

    public void setMaximumAreaThreshold(double ignoreLargerThan, PropertyScale scale) {
        switch (scale) {
            case Pixels:
                this.maxAreaThreshold = ignoreLargerThan;
                break;
            case Percent:
                this.maxAreaThreshold = frameSize.area() * ignoreLargerThan;
                break;
        }
    }

    public void setMaximumAreaThreshold(double threshold) {
        setMaximumAreaThreshold(threshold, PropertyScale.Pixels);
    }

    public void setMinimumAreaThreshold(double threshold, PropertyScale scale) {
        switch (scale) {
            case Pixels:
                this.minAreaThreshold = threshold;
                break;
            case Percent:
                this.minAreaThreshold = frameSize.area() * threshold;
                break;
        }
    }

    public void setMinimumAreaThreshold(double threshold) {
        setMinimumAreaThreshold(threshold, PropertyScale.Pixels);
    }

    public double getMaximumAreaThreshold(PropertyScale scale) {
        switch (scale) {
            default:
            case Pixels:
                return this.maxAreaThreshold;
            case Percent:
                return this.maxAreaThreshold / this.frameSize.area();
        }
    }

    public double getMaximumAreaThreshold() {
        return getMaximumAreaThreshold(PropertyScale.Pixels);
    }

    public double getMinimumAreaThreshold(PropertyScale scale) {
        switch (scale) {
            default:
            case Pixels:
                return this.minAreaThreshold;
            case Percent:
                return this.minAreaThreshold / this.frameSize.area();
        }
    }

    public double getMinimumAreaThreshold() {
        return getMinimumAreaThreshold(PropertyScale.Pixels);
    }

    public enum PropertyScale { Pixels, Percent }

    private Point pixelPointToPercentageOfFrame(Point pixelPoint) {
        double normalizedX = ((pixelPoint.x / frameSize.width) * 100) - 50;
        double normalizedY = ((pixelPoint.y / frameSize.height) * -100) + 50;
        return new Point(normalizedX, normalizedY);
    }
}