package com.tearabite.ftctearabits.vision;

import static com.tearabite.ftctearabits.graphics.LinePaint.WHITE;
import static com.tearabite.ftctearabits.vision.Detection.PropertyScale.Pixels;
import static com.tearabite.ftctearabits.vision.FTCColors.FTC_BLUE_RANGE;
import static com.tearabite.ftctearabits.vision.FTCColors.FTC_RED_RANGE_1;
import static com.tearabite.ftctearabits.vision.FTCColors.FTC_RED_RANGE_2;
import static com.tearabite.ftctearabits.vision.OpenCVUtil.getLargestContour;

import android.graphics.Canvas;

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

public class BasicColorDetectionVisionProcessor implements VisionProcessor {
    public static final Size BLUR_SIZE = new Size(7, 7);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);

    private final Mat blurred = new Mat();
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();
    private final Mat tmpMask = new Mat();

    @Getter @Setter private ScalarRange[] colorRanges;
    @Getter private Detection detection;


    public BasicColorDetectionVisionProcessor(ScalarRange... colorRanges) {

        this.colorRanges = colorRanges;
    }

    public static BasicColorDetectionVisionProcessor Blue() {
        return new BasicColorDetectionVisionProcessor(FTC_BLUE_RANGE);
    }

    public static BasicColorDetectionVisionProcessor Red() {
        return new BasicColorDetectionVisionProcessor(FTC_RED_RANGE_1, FTC_RED_RANGE_2);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.detection = Detection.builder().frameSize(new Size(width, height)).build();
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.GaussianBlur(input, blurred, BLUR_SIZE, 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        mask.release();
        for (ScalarRange colorRange : this.colorRanges) {
            Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), tmpMask);
            if (mask.empty() || mask.rows() <= 0) {
                Core.inRange(hsv, colorRange.getLower(), colorRange.getUpper(), mask);
            }
            Core.add(mask, tmpMask, mask);
        }

        Imgproc.erode(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);
        Imgproc.dilate(mask, mask, STRUCTURING_ELEMENT, ANCHOR, ERODE_DILATE_ITERATIONS);

        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        detection.setContour(getLargestContour(contours));

        return input;
    }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (detection != null && detection.isValid()) {
            Point center = detection.getCenter();
            canvas.drawCircle((float) center.x, (float) center.y, 10, WHITE);
        }
    }

    public double getMinimumAreaThreshold() {
        return this.detection.getMinimumAreaThreshold();
    }

    public void setMinimumAreaThreshold(double ignoreSmallerThan) {
        this.detection.setMinimumAreaThreshold(ignoreSmallerThan);
    }

    public double getMaximumAreaThreshold() {
        return this.detection.getMaximumAreaThreshold();
    }

    public void setMaximumAreaThreshold(double ignoreLargerThan) {
        this.detection.setMaximumAreaThreshold(ignoreLargerThan);
    }
}