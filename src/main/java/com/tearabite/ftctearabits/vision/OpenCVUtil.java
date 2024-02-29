package com.tearabite.ftctearabits.vision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.Collections;
import java.util.List;

/**
 * A utility class for common vision operations
 */
public class OpenCVUtil {

    public static void drawPoint(Mat img, Point point, Scalar color) {
        Imgproc.circle(img, point, 3, color,  -1);
    }

    public static Point getCenterOfContour(MatOfPoint contour) {
        Moments moments = Imgproc.moments(contour);
        return new Point(moments.m10 / moments.m00, moments.m01/ moments.m00);
    }

    public static Point getBottomLeftOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(boundingRect.x, boundingRect.y+boundingRect.height);
    }

    public static Point getBottomRightOfContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return new Point(boundingRect.x+boundingRect.width, boundingRect.y+boundingRect.height);
    }

    public static void drawContour(Mat img, MatOfPoint contour, Scalar color) {
        Imgproc.drawContours(img, Collections.singletonList(contour), 0, color, 2);
    }

    public static void drawConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(img, Collections.singletonList(convertIndexesToPoints(contour, hull)), 0, color, 2);
    }

    public static void fillConvexHull(Mat img, MatOfPoint contour, Scalar color) {
        MatOfInt hull =  new MatOfInt();
        Imgproc.convexHull(contour, hull);
        Imgproc.drawContours(img, Collections.singletonList(convertIndexesToPoints(contour, hull)), 0, color, -1);
    }

    public static MatOfPoint convertIndexesToPoints(MatOfPoint contour, MatOfInt indexes) {
        int[] arrIndex = indexes.toArray();
        Point[] arrContour = contour.toArray();
        Point[] arrPoints = new Point[arrIndex.length];

        for (int i=0;i<arrIndex.length;i++) {
            arrPoints[i] = arrContour[arrIndex[i]];
        }

        MatOfPoint hull = new MatOfPoint();
        hull.fromArray(arrPoints);
        return hull;
    }

    public static MatOfPoint getLargestContour(List<MatOfPoint> contours) {
        if (contours.size() == 0) {
            return null;
        }
        return getLargestContours(contours, 1).get(0);
    }

    public static List<MatOfPoint> getLargestContours(List<MatOfPoint> contours, int numContours) {
        Collections.sort(contours, (a, b) -> (int) Imgproc.contourArea(b) - (int) Imgproc.contourArea(a));
        return contours.subList(0, Math.min(numContours, contours.size()));
    }
}