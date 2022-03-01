package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Optional;

public class ShippingHubRecognition {

    private static final String TAG = ShippingHubRecognition.class.getSimpleName();
    private static final String imageFilePrefix = "Image_";

    private static final double SHIPPING_HUB_ANGLE_NPOS = -360.0;
    private static final double SHIPPING_HUB_DISTANCE_NPOS = -1.0;
    private static final double LOGITECH_BRIO_FIELD_OF_VIEW = 78.0;

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    private String outputFilenamePreamble;

    public ShippingHubRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    public ShippingHubAngleReturn getAngleToShippingHub(ImageProvider pImageProvider,
                                                        VisionParameters.ImageParameters pImageParameters,
                                                        VisionParameters.HSVParameters pHSVParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In ShippingHubRecognition.getAngleToShippingHub");

        // The largest bounding rectangle in the image should be the Level 2
        // platter of the Shipping Hub.
        Rect largestBoundingRect = getlargestBoundingRectangle(pImageProvider, pImageParameters, pHSVParameters);
        if (largestBoundingRect == null)
            return new ShippingHubAngleReturn(true, SHIPPING_HUB_ANGLE_NPOS);

        // Calculate the angle from the camera to the center of the bounding rectangle.
        // The centroid of the bounding rectangle is relative to the entire image, not
        // just the ROI.
        //**TODO May need a "magic number" parameter to get the correct angle (see 2019)
        double angleToShippingHub = imageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, pImageParameters.image_roi.x + largestBoundingRect.x + (largestBoundingRect.width / 2), LOGITECH_BRIO_FIELD_OF_VIEW);
        RobotLogCommon.d(TAG, "Angle to Shipping Hub " + angleToShippingHub);

        return new ShippingHubAngleReturn(false, angleToShippingHub);
    }

    public ShippingHubDistanceReturn getDistanceToShippingHub(ImageProvider pImageProvider,
                                                              VisionParameters.ImageParameters pImageParameters,
                                                              VisionParameters.HSVParameters pHSVParameters,
                                                              ShippingHubParameters pShippingHubParameters) throws InterruptedException {
        // The largest bounding rectangle in the image should be the Level 2
        // platter of the Shipping Hub.
        Rect largestBoundingRect = getlargestBoundingRectangle(pImageProvider, pImageParameters, pHSVParameters);
        if (largestBoundingRect == null)
            return new ShippingHubDistanceReturn(true, SHIPPING_HUB_DISTANCE_NPOS);

        //**TODO code goes here
        return new ShippingHubDistanceReturn(false, 0.0);
    }

    private Rect getlargestBoundingRectangle(ImageProvider pImageProvider,
                                             VisionParameters.ImageParameters pImageParameters,
                                             VisionParameters.HSVParameters pHSVParameters) throws InterruptedException {

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> shippingHubImage = pImageProvider.getImage();
        if (shippingHubImage.first == null)
            return null; // don't crash

        String fileDate = TimeStamp.getLocalDateTimeStamp(shippingHubImage.second);
        outputFilenamePreamble = workingDirectory + imageFilePrefix + fileDate;

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = shippingHubImage.first.clone();

        // If you don't convert RGB to BGR here then the _IMG.png file will be written
        // out with incorrect colors (gold will show up as blue).
        if (pImageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB) {
            // The image came from a camera.
            Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

            String imageFilename = outputFilenamePreamble + "_IMG.png";
            Imgcodecs.imwrite(imageFilename, imgOriginal);
            RobotLogCommon.d(TAG, "Writing original image " + imageFilename);
        }

        RobotLogCommon.d(TAG, "Image width " + imgOriginal.cols() + ", height " + imgOriginal.rows());
        if ((imgOriginal.cols() != pImageParameters.resolution_width) ||
                (imgOriginal.rows() != pImageParameters.resolution_height))
            throw new AutonomousRobotException(TAG,
                    "Mismatch between actual image width and expected image width " + pImageParameters.resolution_width +
                            ", height " + pImageParameters.resolution_height);

        // Crop the image to reduce distractions.
        Mat imageROI = imageUtils.getImageROI(imgOriginal,
                new Rect(pImageParameters.image_roi.x,
                        pImageParameters.image_roi.y,
                        pImageParameters.image_roi.width,
                        pImageParameters.image_roi.height));

        String imageFilename = outputFilenamePreamble + "_ROI.png";
        Imgcodecs.imwrite(imageFilename, imageROI);
        RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);

        Mat thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pHSVParameters);
        List<MatOfPoint> contours = imageUtils.findContoursInThresholdedImage(thresholded);

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
        Mat drawnRect = new Mat();
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0)); // GREEN
    }
}