package com.tearabite.ftctearabits.vision;

import static com.tearabite.ftctearabits.vision.Colors.WHITE;
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

public class BasicColorDetectionVisionProcessor implements VisionProcessor {
    public static final Size BLUR_SIZE = new Size(7, 7);
    public static final int ERODE_DILATE_ITERATIONS = 2;
    public static final Mat STRUCTURING_ELEMENT = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    public static final Point ANCHOR = new Point((STRUCTURING_ELEMENT.cols() / 2f), STRUCTURING_ELEMENT.rows() / 2f);

    private final Mat blurred = new Mat();
    private final ScalarRange[] colorRanges;
    @Getter private Detection detection;
    private final Mat hsv = new Mat();
    private final double ignoreSmallerThan;
    private final double ignoreLargerThan;
    private final Mat mask = new Mat();
    private final Mat tmpMask = new Mat();


    public BasicColorDetectionVisionProcessor(double ignoreSmallerThan, double ignoreLargerThan, ScalarRange... colorRanges) {
        this.ignoreSmallerThan = ignoreSmallerThan;
        this.ignoreLargerThan = ignoreLargerThan;
        this.colorRanges = colorRanges;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        this.detection = new Detection(new Size(width, height), ignoreSmallerThan, ignoreLargerThan);
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
            Point center = detection.getCenterPx();
            canvas.drawCircle((float) center.x, (float) center.y, 10, WHITE);
        }
    }
}