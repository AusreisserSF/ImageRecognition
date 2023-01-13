package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class RealSenseRecognition {

    private static final String TAG = RealSenseRecognition.class.getSimpleName();

    public RealSenseReturn grayscalePath(Mat pDepthImageROI,
                                         D405Configuration pD405Configuration,
                                         RobotConstantsPowerPlay.D405CameraId pCameraId,
                                         short[] pDepthArray, float pObjectWidth,
                                         String pOutputFilenamePreamble,
                                         VisionParameters.ImageParameters pImageParameters,
                                         VisionParameters.GrayParameters pGrayParameters,
                                         DepthParameters pDepthParameters) {
        Mat thresholded = ImageUtils.performThreshold(pDepthImageROI, pOutputFilenamePreamble,
                pGrayParameters.median_target,
                pGrayParameters.threshold_low);

        return RealSenseUtils.getAngleAndDistance(pDepthImageROI, thresholded,
                pD405Configuration, pCameraId, pDepthArray, pObjectWidth,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }

    public RealSenseReturn redChannelPath(Mat pImageROI,
                                          D405Configuration pD405Configuration,
                                          RobotConstantsPowerPlay.D405CameraId pCameraId,
                                          short[] pDepthArray, float pObjectWidth,
                                          String pOutputFilenamePreamble,
                                          VisionParameters.ImageParameters pImageParameters,
                                          VisionParameters.GrayParameters pGrayParameters,
                                          DepthParameters pDepthParameters) {

        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the red channel as grayscale.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");

        Mat thresholded = ImageUtils.performThresholdOnGray(channels.get(2), pOutputFilenamePreamble, pGrayParameters.median_target, pGrayParameters.threshold_low);

        return RealSenseUtils.getAngleAndDistance(pImageROI, thresholded,
                pD405Configuration, pCameraId, pDepthArray, pObjectWidth,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }

    //**TODO for the blue cone stack only - move to ConeStackRecognition.
    // Thresholds the blue channel to isolate the bottom black railing of
    // the field. The infers that the blue cone stack is in the gap in the
    // thresholded railing.
    // Ported from c++ Visual Studio project OpenCVTestbed2, GeneralTarget.cpp
    // function blue_cones().
    public RealSenseReturn blueChannelPath(Mat pImageROI, Mat pDepthROI,
                                           D405Configuration pD405Configuration,
                                           RobotConstantsPowerPlay.D405CameraId pCameraId,
                                           short[] pDepthArray, float pObjectWidth,
                                           String pOutputFilenamePreamble,
                                           VisionParameters.ImageParameters pImageParameters,
                                           VisionParameters.HSVParameters pHSVParameters,
                                           VisionParameters.GrayParameters pGrayParameters,
                                           DepthParameters pDepthParameters) {

        // Split the original image ROI into its BGR channels and use the
        // the blue channel (lighter here than in a pure grayscale image)
        // to get better contrast with the black railing.
        ArrayList<Mat> originalImageChannels = new ArrayList<>(3);
        Core.split(pImageROI, originalImageChannels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the blue channel as grayscale.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BLUE_CHANNEL.png", originalImageChannels.get(0));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BLUE_CHANNEL.png");

        // Use an inverted threshold on the blue channel to create a white image of the black railing.
        Mat thresholded = ImageUtils.performThresholdOnGray(originalImageChannels.get(0), pOutputFilenamePreamble, pGrayParameters.median_target, pGrayParameters.threshold_low);

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() < 2) {
            RobotLogCommon.d(TAG, "Did not find at least two contours");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Within the ROI draw all of the contours.
        Mat contoursDrawn = pImageROI.clone();
        drawShapeContours(contours, contoursDrawn); //**TODO move to ShapeDrawing - see PPV
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CON.png");

        // Sort the contours and put rotated rectangles around the two largest.
        // Assume that these two rectangles enclose the bottom railing of the
        // field to the left and right of the cone stack.

        List<MatOfPoint> sortedContours = ImageUtils.sortContours(contours);
        RotatedRect rotatedRect1 = Imgproc.minAreaRect(new MatOfPoint2f(sortedContours.get(0)));
        Point[] rect_points_1 = new Point[4];
        rotatedRect1.points(rect_points_1);

        RotatedRect rotatedRect2 = Imgproc.minAreaRect(new MatOfPoint2f(sortedContours.get(1)));
        ;
        Point[] rect_points_2 = new Point[4];
        rotatedRect2.points(rect_points_2);

        // Draw the rotated rectangles.
        Mat rotatedRectangles = pImageROI.clone();
        List<MatOfPoint> rrContours = new ArrayList<>(); //**TODO move to ShapeDrawing
        rrContours.add(new MatOfPoint(rect_points_1));
        Imgproc.drawContours(rotatedRectangles, rrContours, 0, new Scalar(0, 255, 0), -1);

        rrContours.clear();
        rrContours.add(new MatOfPoint(rect_points_2));
        Imgproc.drawContours(rotatedRectangles, rrContours, 0, new Scalar(0, 255, 0), -1);

        // The two rectangles must be within 5 degrees of horizontal.
        // See https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/
        RobotLogCommon.d(TAG, "rotatedRect1.angle " + rotatedRect1.angle);
        RobotLogCommon.d(TAG, "rotatedRect2.angle " + rotatedRect2.angle);

        double angle1Abs = Math.abs(rotatedRect1.angle);
        double angle2Abs = Math.abs(rotatedRect2.angle);
        if (angle1Abs < 85.0 || angle1Abs > 95.0 ||
                angle2Abs < 85.0 || angle2Abs > 95.0) {
            RobotLogCommon.d(TAG, "One of the two largest contours is at an angle of > +-5 degrees");
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_UNSUCCESSFUL);
        }

        // Figure out which of the two rectangles is on the left.
        // "The lowest point in a rectangle is 0th vertex, and 1st, 2nd, 3rd vertices follow clockwise."
        // The sorted array is in ascending order.
        Point[] rect_points_1_sorted = rect_points_1.clone();
        Arrays.sort(rect_points_1_sorted, Comparator.comparing(point -> point.x));
        RobotLogCommon.d(TAG, "rotatedRect1 min x " + rect_points_1_sorted[0].x +
                ", max " + rect_points_1_sorted[3].x);

        Point[] rect_points_2_sorted = rect_points_2.clone();
        Arrays.sort(rect_points_2_sorted, Comparator.comparing(point -> point.x));
        RobotLogCommon.d(TAG, "rotatedRect2 min x " + rect_points_2_sorted[0].x +
                ", max " + rect_points_2_sorted[3].x);

        double maxXOfLeftSide;
        double minXOfRightSide;
        double widthOfConeStack;
        if (rect_points_1_sorted[3].x < rect_points_2_sorted[0].x) {
            RobotLogCommon.d(TAG, "rotatedRect1 is to the left of rotatedRect2");
            maxXOfLeftSide = rect_points_1_sorted[3].x;
            minXOfRightSide = rect_points_2_sorted[0].x;
        } else {
            RobotLogCommon.d(TAG, "rotatedRect2 is to the left of rotatedRect1");
            maxXOfLeftSide = rect_points_2_sorted[3].x;
            minXOfRightSide = rect_points_1_sorted[3].x;
        }

        // The difference in pixels between the maximum x-coordinate of
        // the rotated rectangle on the left and the minimum x-coordinate
        // off the rotated rectangle on the right is the width of the cone
        // stack.
        widthOfConeStack = minXOfRightSide - maxXOfLeftSide;
        RobotLogCommon.d(TAG, "Pixel width of cone stack " + widthOfConeStack);

        double pixelsPerInch = widthOfConeStack / 4.0f;
        RobotLogCommon.d(TAG, "Pixels per inch " + pixelsPerInch);

        //**TODO Now you have the width of a bounding box for the cone stack
        // even though no contour is present. Define a pixel search area and
        // use this in conjunction with the depth array.

        //**TODO The y-coordinates of the left and right side rectangles
        // must be within xx% of the bottom of the image and within yy%
        // of each other.

        //**TODO TEMP until the rest of the logic is implemented.
        return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_SUCCESSFUL);
    }

    // Analyzes a grayscale image using a combination of the red channel and the blue channel.
    public RealSenseReturn twoChannelPath(Mat pDepthImageROI,
                                          D405Configuration pD405Configuration,
                                          RobotConstantsPowerPlay.D405CameraId pCameraId,
                                          short[] pDepthArray, float pObjectWidth,
                                          String pOutputFilenamePreamble,
                                          VisionParameters.ImageParameters pImageParameters,
                                          VisionParameters.GrayParameters pGrayParameters,
                                          DepthParameters pDepthParameters) {

        // Remove distractions before we convert to grayscale: depending on the
        // current alliance set the red or blue channel pixels to black.
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(pDepthImageROI, channels); // red or blue channel. B = 0, G = 1, R = 2

        // Use both the red and blue channels.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_RED_CHANNEL.png", channels.get(2));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_RED_CHANNEL.png");

        // Write out the blue channel also.
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_BLUE_CHANNEL.png", channels.get(0));
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_BLUE_CHANNEL.png");

        // Use logical 'or' to isolate the dark junction pole cap.
        Mat combined = new Mat();
        Core.bitwise_or(channels.get(0), channels.get(2), combined);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_COMBINED.png", combined);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_COMBINED.png");

        Mat thresholded = ImageUtils.performThresholdOnGray(combined, pOutputFilenamePreamble, pGrayParameters.median_target, pGrayParameters.threshold_low);

        return RealSenseUtils.getAngleAndDistance(pDepthImageROI, thresholded,
                pD405Configuration, pCameraId, pDepthArray, pObjectWidth,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }

    // Analyze a color image.
    public RealSenseReturn colorPath(Mat pImageROI,
                                     D405Configuration pD405Configuration,
                                     RobotConstantsPowerPlay.D405CameraId pCameraId,
                                     short[] pDepthArray, float pObjectWidth,
                                     String pOutputFilenamePreamble,
                                     VisionParameters.ImageParameters pImageParameters,
                                     VisionParameters.HSVParameters pHSVParameters,
                                     DepthParameters pDepthParameters) {

        Mat thresholded = ImageUtils.applyInRange(pImageROI, pOutputFilenamePreamble, pHSVParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return RealSenseUtils.getAngleAndDistance(pImageROI, thresholded,
                pD405Configuration, pCameraId, pDepthArray, pObjectWidth,
                pOutputFilenamePreamble, pImageParameters, pDepthParameters);
    }

    // The parameters pContours is the output of a call to findContours.
    private static void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        RobotLogCommon.d(TAG, "drawContours: number of contours " + pContours.size());
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }
}