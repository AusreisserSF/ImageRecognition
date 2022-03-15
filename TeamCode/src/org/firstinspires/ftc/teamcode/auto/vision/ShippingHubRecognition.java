package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

public class ShippingHubRecognition {

    private static final String TAG = ShippingHubRecognition.class.getSimpleName();
    private static final double LOGITECH_BRIO_FIELD_OF_VIEW = 78.0;

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public ShippingHubRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    //## 3/15/2022 Notes from Visual Studio: there is a prototype implementation in Java - both
    // in IntelliJ (project ImageRecognition) and Android Studio (Freight Frenzy). The Java
    // versions include Dennis's extra calculation to determine the angle from the center of the
    // robot instead of from the camera. Note that Dennis's algorithm uses the distance to the
    // Shipping Hub from the similar triangles method - even though the Shipping Hub may not be
    // at the center of the frame. We need to research and experiment with angular distortion;
    // what is the width difference in pixels of an object of known size at a known distance
    // when the object is not at the center of the frame.
    // Unresolved:  the angle of the Shipping Hub - left, right, towards, or away from the robot.
    // This may make this method untenable. But save for rectangle recognition and compare with
    // Vumark outline detection [in Visual Studio]. See also findShippingHubShaftRectangles,
    // which may be a better path.See also findShippingHubShaftRectangles, which may be a better
    // path: you should be able to get the angle of the shaft to the left and right but not fore
    //and aft.

    // Returns the result of image analysis.
    public ShippingHubReturn getAngleAndDistanceToShippingHub(ImageProvider pImageProvider,
                                                              VisionParameters.ImageParameters pImageParameters,
                                                              VisionParameters.HSVParameters pHSVParameters,
                                                              ShippingHubParameters pShippingHubParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In ShippingHubRecognition.getAngleAndDistanceToShippingHub");

        // The largest bounding rectangle in the image should be the Level 2
        // platter of the Shipping Hub.
        Rect largestBoundingRect = getLargestBoundingRectangle(pImageProvider, pImageParameters, pHSVParameters);
        if (largestBoundingRect == null)
            return new ShippingHubReturn(RobotConstants.OpenCVResults.OCV_ERROR);

        // Calculate the angle from the camera to the center of the bounding rectangle.
        // The centroid of the bounding rectangle is relative to the entire image, not
        // just the ROI.
        double angleFromCameraToShippingHub = imageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, pImageParameters.image_roi.x + largestBoundingRect.x + (largestBoundingRect.width / 2), LOGITECH_BRIO_FIELD_OF_VIEW);
        RobotLogCommon.d(TAG, "Angle from camera to Shipping Hub " + angleFromCameraToShippingHub);

        // Test for calibration run.
        double distanceFromCameraToShippingHub;
        if (pShippingHubParameters.distanceParameters.focalLength == 0.0) {
            RobotLogCommon.d(TAG, "Calibration run");
            double focalLength = (largestBoundingRect.width * pShippingHubParameters.distanceParameters.calibrationObjectDistance) / pShippingHubParameters.distanceParameters.calibrationObjectWidth;
            RobotLogCommon.d(TAG, "Calculated focal length " + focalLength);
            distanceFromCameraToShippingHub = pShippingHubParameters.distanceParameters.calibrationObjectDistance;
            RobotLogCommon.d(TAG, "Calibration distance from camera to Shipping Hub " + distanceFromCameraToShippingHub);
        } else {
            // Distance determination (non-calibration) path.
            RobotLogCommon.d(TAG, "Using focal length from calibration run of " + pShippingHubParameters.distanceParameters.focalLength);
            distanceFromCameraToShippingHub = (pShippingHubParameters.distanceParameters.calibrationObjectWidth * pShippingHubParameters.distanceParameters.focalLength) / largestBoundingRect.width;
            RobotLogCommon.d(TAG, "Calculated distance from camera to Shipping Hub " + distanceFromCameraToShippingHub + " inches");
        }

        // Calculate the angle from the robot center to the Shipping Hub.
        double trueBearing = TrueBearing.computeTrueBearing(distanceFromCameraToShippingHub, angleFromCameraToShippingHub, pShippingHubParameters.distanceParameters.cameraToRobotCenter);
        RobotLogCommon.d(TAG, "Angle from robot center to Shipping Hub  " + trueBearing);

        return new ShippingHubReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, trueBearing, distanceFromCameraToShippingHub);
    }

    private Rect getLargestBoundingRectangle(ImageProvider pImageProvider,
                                             VisionParameters.ImageParameters pImageParameters,
                                             VisionParameters.HSVParameters pHSVParameters) throws InterruptedException {

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> shippingHubImage = pImageProvider.getImage();
        if (shippingHubImage.first == null)
            return null; // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = shippingHubImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(shippingHubImage.second);
        String outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.ocv_image, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        Mat imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        List<MatOfPoint> contours = imageUtils.applyInRangeAndFindContours(imageROI, outputFilenamePreamble, pHSVParameters);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return null;
        }

        Mat contoursDrawn = imageROI.clone();
        drawShapeContours(contours, contoursDrawn);
        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        // The largest contour should be the level 2 platter of the Shipping Hub.
        Optional<MatOfPoint> largestContour = imageUtils.getLargestContour(contours);
        if (largestContour.isEmpty())
            return null;

        // Get its bounding rectangle.
        Rect largestBoundingRect = Imgproc.boundingRect(largestContour.get());

        // Because the shaft is the same color as the level 2 platter the bounding
        // rectangle will be too high. But we only care about its width.
        RobotLogCommon.d(TAG, "Width of largest contour " + largestBoundingRect.width);

        // Draw a rectangle around the largest contour.
        Mat drawnRectangle = imageROI.clone();
        drawOneRectangle(largestBoundingRect, drawnRectangle);
        Imgcodecs.imwrite(outputFilenamePreamble + "_BRECT.png", drawnRectangle);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BRECT.png");

        return largestBoundingRect;
    }

    // The parameters pContours is the output of a call to findContours.
    private void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        RobotLogCommon.d(TAG, "drawContours: number of contours " + pContours.size());
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }

    private void drawOneRectangle(Rect pRect, Mat pImageOut) {
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0)); // GREEN
    }
}