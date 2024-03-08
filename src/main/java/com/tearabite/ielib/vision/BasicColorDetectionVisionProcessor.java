package com.tearabite.ielib.vision;

import static com.tearabite.ielib.graphics.LinePaint.WHITE;
import static com.tearabite.ielib.vision.FTCColors.FTC_BLUE_RANGE;
import static com.tearabite.ielib.vision.FTCColors.FTC_RED_RANGE_1;
import static com.tearabite.ielib.vision.FTCColors.FTC_RED_RANGE_2;
import static com.tearabite.ielib.vision.OpenCVUtil.getLargestContour;

import android.graphics.Canvas;

import com.tearabite.ielib.graphics.LinePaint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

import lombok.Getter;
import lombok.Setter;

/**
 * A basic color detection vision processor that detects the largest contour of a specified color
 */
public class BasicColorDetectionVisionProcessor implements VisionProcessor {
    private static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    private static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);

    /**
     * The size of the blur kernel
     */
    @Getter @Setter private Size blurSize = new Size(7, 7);

    /**
     * The color ranges to detect
     */
    @Getter @Setter private ScalarRange[] colorRanges;

    /**
     * The detection object
     */
    @Getter private Detection detection;

    /**
     * The number of iterations to erode and dilate the mask
     */
    @Getter @Setter private int erodeDilateIterations = 2;

    private final Mat blurred = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat tmpMask = new Mat();

    public BasicColorDetectionVisionProcessor(ScalarRange... colorRanges) {
        this.colorRanges = colorRanges;
    }

    /**
     * @return a new instance of a color detection vision processor targeting FTC Blue
     */
    public static BasicColorDetectionVisionProcessor Blue() {
        return new BasicColorDetectionVisionProcessor(FTC_BLUE_RANGE);
    }

    /**
     * @return a new instance of a color detection vision processor targeting FTC Red
     */
    public static BasicColorDetectionVisionProcessor Red() {
        return new BasicColorDetectionVisionProcessor(FTC_RED_RANGE_1, FTC_RED_RANGE_2);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.detection = Detection.builder().frameSize(new Size(width, height)).build();
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.GaussianBlur(input, blurred, blurSize, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        mask.release();
        for (ScalarRange colorRange : this.colorRanges) {
            Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), tmpMask);
            if (mask.empty() || mask.rows() <= 0) {
                Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), mask);
            }
            Core.add(mask, tmpMask, mask);
        }

        Imgproc.erode(mask, mask, STRUCTURING_ELEMENT, ANCHOR, erodeDilateIterations);
        Imgproc.dilate(mask, mask, STRUCTURING_ELEMENT, ANCHOR, erodeDilateIterations);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        detection.setContour(getLargestContour(contours));

        return input;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (detection != null && detection.isValid()) {
            Point center = detection.getCenter();
            canvas.drawCircle((float) center.x, (float) center.y, 10, LinePaint.WHITE);
        }
    }

    /**
     * @return the minimum area threshold in pixels
     */
    public double getMinimumAreaThreshold() {
        return this.detection.getMinimumAreaThreshold();
    }

    /**
     * @param minimumAreaThreshold the minimum area threshold in pixels
     */
    public void setMinimumAreaThreshold(double minimumAreaThreshold) {
        this.detection.setMinimumAreaThreshold(minimumAreaThreshold);
    }

    /**
     * @return the maximum area threshold in pixels
     */
    public double getMaximumAreaThreshold() {
        return this.detection.getMaximumAreaThreshold();
    }

    /**
     * @param maximumAreaThreshold the maximum area threshold in pixels
     */
    public void setMaximumAreaThreshold(double maximumAreaThreshold) {
        this.detection.setMaximumAreaThreshold(maximumAreaThreshold);
    }
}