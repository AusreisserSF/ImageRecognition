package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;

//** Android import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;

// Copied on 3/12/2021 from FtcUltimateGoal on Github.
public class TowerGoalAlignment {

    private static final String TAG = "TowerGoalLocation";
    private static final String imageFilePrefix = "Image_";

    public static final double TOWER_ANGLE_NPOS = -361.0; //** RobotConstantsUltimateGoal

    private final String workingDirectory;
    private final ImageUtils imageUtils;
    private final Random rng = new Random(12345);

    public TowerGoalAlignment() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the angle that the robot needs to turn in order to face the center of tower goal.
    // Returns TOWER_ANGLE_NPOS if this method can't determine a valid angle. 
    public double getAngleToTowerGoal(ImageProvider pImageProvider,
                                      TowerParameters pTowerParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In TowerGoalLocation.getAngleToTowerGoal");

        // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> vuforiaTargetImage = pImageProvider.getImage();
        if (vuforiaTargetImage.first == null) {
            RobotLogCommon.d(TAG, "Failed to read image");
            return TOWER_ANGLE_NPOS; // don't crash
        }

        String fileDate = CommonUtils.getDateTimeStamp(vuforiaTargetImage.second);
        String outputFilenamePreamble = workingDirectory + imageFilePrefix + fileDate;

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = vuforiaTargetImage.first.clone();

        // If you don't convert RGB to BGR here then the _IMG.png file will be written
        // out with incorrect colors (gold will show up as blue).
        if (pImageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
            Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

        String imageFilename = outputFilenamePreamble + "_IMG.png";
        RobotLogCommon.d(TAG, "Writing original image " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imgOriginal);

        // Crop the image to reduce distractions.
        Mat imageROI = imageUtils.getImageROI(imgOriginal, pTowerParameters.imageParameters.image_roi);
        imageFilename = outputFilenamePreamble + "_ROI.png";
        RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imageROI);

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);
        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        // Adjust the brightness.
        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pTowerParameters.grayParameters.target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        int grayThresholdLow = pTowerParameters.grayParameters.low_threshold;
        RobotLogCommon.d(TAG, "Threshold values: low " + grayThresholdLow + ", high 255");

        // Use inRange to threshold to binary.
        Mat thresholded = new Mat();
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // value assigned to pixels over threshold value
                Imgproc.THRESH_BINARY); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        //## Adapted from findSilverMineral - the same steps work here.
        //!! [silver] For some reason, the blurring step is absolutely necessary.
        //!! Here the blurring step reduces the number of contours. Note that blurring
        //!! a binary image renders it non-binary.
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(thresholded, blurred, new Size(5, 5), 0);

        // Find the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(blurred, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        RobotLogCommon.d(TAG, "Number of contours " + contours.size());

        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No Vumark contours found");
            return TOWER_ANGLE_NPOS;
        }

        Mat drawing = imageROI.clone();
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
            Imgproc.drawContours(drawing, contours, i, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
        }

        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", drawing);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        //**TODO Port to Android!!

        double maxContourArea = 0;
        int maxValIdx = 0;
        double contourArea;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            contourArea = Imgproc.contourArea(contours.get(contourIdx));
            if (maxContourArea < contourArea) {
                maxContourArea = contourArea;
                maxValIdx = contourIdx;
            }
        }

        RobotLogCommon.d(TAG, "Largest contour area = " + maxContourArea);
        if (maxContourArea < pTowerParameters.minimum_area) {
            RobotLogCommon.d(TAG, "Area of the largest contour is below the minimum of " + pTowerParameters.minimum_area);
            return TOWER_ANGLE_NPOS;
        }

        if (maxContourArea > pTowerParameters.maximum_area) {
            RobotLogCommon.d(TAG, "Area of the largest contour is above the maximum of " + pTowerParameters.maximum_area);
            return TOWER_ANGLE_NPOS;
        }

        RobotLogCommon.d(TAG, "Found a Vumark");

        // Draw a rotated rectangle around the Vumark with a circle at the center.
        Mat vumark = imageROI.clone();
        RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(maxValIdx).toArray()));
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(vumark, Collections.singletonList(points), -1, new Scalar(255, 0, 0), 4);

        // Get the centroid of the Vumark.
        Moments vuMarkMoments = Imgproc.moments(contours.get(maxValIdx));
        Point vumarkCentroid =        //add 1e-5 to avoid division by zero
                new Point(vuMarkMoments.m10 / (vuMarkMoments.m00 + 1e-5), vuMarkMoments.m01 / (vuMarkMoments.m00 + 1e-5));

        // Draw a black circle around the centroid.
        Imgproc.circle(vumark, vumarkCentroid, 10, new Scalar(0, 0, 0), 4);
        Imgcodecs.imwrite(outputFilenamePreamble + "_RR.png", vumark);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RR.png");

        return computeAngle(pTowerParameters.imageParameters.image_roi.width, vumarkCentroid.x, pTowerParameters.vumark_center_to_frame_center_angle);
    }

    // At the robot's ideal position for shooting, the Vumark may not be in
    // the center of the region of interest (ROI) defined in the xml file.
    // The xml file supports an element vumark_center_to_frame_center_angle.
    // Account for that offset here when computing the number of degrees
    // that the robot needs to turn to be in its ideal shooting position.
    private double computeAngle(double pROIWidth, double pVumarkCentroidX, double pAngleOffset) {

        //**TODO replace these constants with a computation based on the
        // similar triangles method.
        // For distance use height instead of width.

        //** width of object as a function of skew
        // see https://medium.com/analytics-vidhya/how-to-track-distance-and-angle-of-an-object-using-opencv-1f1966d418b4

        final double INCHES_TO_TOWER = 60.0; // from the launch line
        final double VUMARK_WIDTH_INCHES = 11.0;
        final double VUMARK_HEIGHT_INCHES = 8.5;
        final double VUMARK_PIXEL_WIDTH_FROM_LAUNCH_LINE = 110; //** guess: 11.0" = 110px @60"
        final double PIXELS_PER_INCH_AT_LAUNCH_LINE = VUMARK_PIXEL_WIDTH_FROM_LAUNCH_LINE / VUMARK_WIDTH_INCHES;

        // Get the angle from the robot to the center of the Vumark.
        // Get the distance from the center of the Vumark to the center
        // of the ROI.
        double vumarkCenterOffset = pVumarkCentroidX - (pROIWidth / 2);

        // Get the direction of the center of the Vumark from the center
        // of the ROI. If the Vumark is to the right of the center of the
        // ROI then the direction will be positive; otherwise negative.
        double thetaDegrees = 0;
        double directionToVumarkCenter = Math.signum(vumarkCenterOffset);
        if (directionToVumarkCenter != 0) {
            // Get the number of degrees from the center of the ROI to the
            // center of the Vumark.
            double oppositePixels = Math.abs(vumarkCenterOffset);
            double adjacentPixels = INCHES_TO_TOWER * PIXELS_PER_INCH_AT_LAUNCH_LINE;
            double tanTheta = oppositePixels / adjacentPixels;
            double thetaAngleRadians = Math.atan(tanTheta);
            thetaDegrees = Math.toDegrees(thetaAngleRadians);
        }

        // Get the number of degrees to turn. A positive value indicates a
        // counter-clockwise turn, a negative value indicates a clockwise
        // turn.
        // If the center of the Vumark is to the right of the center of the ROI,
        // the distance is positive but the angle, which is counter-clockwise,
        // must be negative. So we have to invert the angle value.
        // Vumark center to the right of ROI center --
        // offset -5; angle 10 * directionToVumarkCenter(1) * -1 -> -5 - -10; -> +5
        // Vumark center to the left of ROI center --
        // offset -5; angle 10 * directionToVumarkCenter(-1) * -1 -> -5 - 10;  -> -15
        double finalAngleAdjustment = pAngleOffset - (thetaDegrees * directionToVumarkCenter * -1);
        RobotLogCommon.d(TAG, "Expected angle offset " + pAngleOffset);
        RobotLogCommon.d(TAG, "Raw angle to Vumark " + thetaDegrees);
        RobotLogCommon.d(TAG, "Final angle adjustment " + finalAngleAdjustment);

        return finalAngleAdjustment;
    }

}