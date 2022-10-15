package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.EnumMap;

public class BarcodeRecognition {

    private static final String TAG = BarcodeRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    private String outputFilenamePreamble;
    private Mat imageROI;
    private Rect leftBarcodeElementWindow;
    private Rect rightBarcodeElementWindow;
    private int minWhitePixels;

    public BarcodeRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    public BarcodeReturn findTeamScoringElement(ImageProvider pImageProvider,
                                                VisionParameters.ImageParameters pImageParameters,
                                                BarcodeParameters pBarcodeParameters,
                                                RobotConstantsFreightFrenzy.RecognitionPath pRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In BarcodeRecognition.findTeamScoringElement");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> barcodeImage = pImageProvider.getImage();
        if (barcodeImage.first == null)
            return new BarcodeReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = barcodeImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(barcodeImage.second);
        outputFilenamePreamble = imageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        imageROI = imageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Draw the barcode element windows (sub-ROIs) on the original image
        // so that we can see the placement during debugging.
        Mat barcodeElementWindows = imgOriginal.clone();

        // Get the left window from the barcode parameters.
        // Remember - the barcode element windows are relative to the overall ROI,
        // not the original image.
        EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> barcodeElements =
                pBarcodeParameters.getBarcodeElements();
        leftBarcodeElementWindow = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWindow.LEFT);
        Point leftWindowUpperLeft =
                new Point(pImageParameters.image_roi.x + leftBarcodeElementWindow.x, pImageParameters.image_roi.y + leftBarcodeElementWindow.y);
        Point leftWindowLowerRight = new Point(pImageParameters.image_roi.x + leftBarcodeElementWindow.x + leftBarcodeElementWindow.width,
                pImageParameters.image_roi.y + leftBarcodeElementWindow.y + leftBarcodeElementWindow.height);

        // Get the right window from the barcode parameters.
        rightBarcodeElementWindow = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWindow.RIGHT);
        Point rightWindowUpperLeft = new Point(pImageParameters.image_roi.x + rightBarcodeElementWindow.x, pImageParameters.image_roi.y + rightBarcodeElementWindow.y);
        Point rightWindowLowerRight = new Point(pImageParameters.image_roi.x + rightBarcodeElementWindow.x + rightBarcodeElementWindow.width,
                pImageParameters.image_roi.y + rightBarcodeElementWindow.y + rightBarcodeElementWindow.height);

        // Draw the windows in red.
        Imgproc.rectangle(barcodeElementWindows, leftWindowUpperLeft, leftWindowLowerRight, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(barcodeElementWindows, rightWindowUpperLeft, rightWindowLowerRight, new Scalar(0, 0, 255), 3);
        Imgcodecs.imwrite(outputFilenamePreamble + "_WIN.png", barcodeElementWindows);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_WIN.png");

        // Set the minimum pixel count for recognition.
        minWhitePixels = pBarcodeParameters.minWhitePixels;

        RobotLogCommon.d(TAG, "Recognition path " + pRecognitionPath);
        BarcodeReturn retVal;
        switch (pRecognitionPath) {
            case GRAY: {
                retVal = grayRecognitionPath(pBarcodeParameters.grayParameters);
                break;
            }
            case HSV: {
                retVal = hsvRecognitionPath(pBarcodeParameters.hsvParameters);
                break;
            }
            case REFLECTIVE_TAPE: {
                retVal = reflectiveTapeRecognitionPath(pBarcodeParameters.grayParameters);
                break;
            }
            default:
                throw new AutonomousRobotException(TAG, "Unsupported recognition path " + pRecognitionPath);
        }

        return retVal;
    }

    private BarcodeReturn grayRecognitionPath(VisionParameters.GrayParameters pGrayParameters) {
        // Adapted from ...\OpenCV_Projects\OpenCVTestbed2\OpenCVTestbed2\GeneralTarget::analyzeSkystoneStripe
        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pGrayParameters.median_target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        int grayThresholdLow = pGrayParameters.threshold_low;
        RobotLogCommon.d(TAG, "Inverse threshold values: low " + grayThresholdLow + ", max 255 (white)");

        // Threshold the image with inversion, i.e. set pixels *under* the threshold
        // value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY_INV); // thresholding type

        // Our target will now appear white in the thresholded image.
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        return lookThroughWindows(thresholded);
    }

    // Send the ROI from the original BGR input down the HSV recognition path.
    private BarcodeReturn hsvRecognitionPath(VisionParameters.HSVParameters pHSVParameters) {
        Mat thresholded = imageUtils.applyInRange(imageROI, outputFilenamePreamble, pHSVParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return lookThroughWindows(morphed);
    }

    private BarcodeReturn reflectiveTapeRecognitionPath(VisionParameters.GrayParameters pGrayParameters) {
        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pGrayParameters.median_target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_REF_ADJ.png");

        int grayThresholdLow = pGrayParameters.threshold_low;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        // Our target will now appear white in the thresholded image.
        Imgcodecs.imwrite(outputFilenamePreamble + "_REF_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_REF_ADJ_THR.png");

        return lookThroughWindows(thresholded);
    }

    // Look through the left and right windows and determine if the team shipping
    // element is in the left window, the right window, or neither.
    private BarcodeReturn lookThroughWindows(Mat pThresholded) {
        // We're going to concentrate only on the two windows. Create a sub-matrix for
        // each.
        Mat leftWindow = pThresholded.submat(leftBarcodeElementWindow);
        Mat rightWindow = pThresholded.submat(rightBarcodeElementWindow);

        // Count the non-zero pixels in each window.
        int leftWindowNonZeroPixelCount = Core.countNonZero(leftWindow);
        RobotLogCommon.d(TAG, "Left window: number of non-zero pixels " + leftWindowNonZeroPixelCount);

        int rightWindowNonZeroPixelCount = Core.countNonZero(rightWindow);
        RobotLogCommon.d(TAG, "Right window: number of non-zero pixels " + rightWindowNonZeroPixelCount);

        // Check the minimum non-zero pixel count.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count " + minWhitePixels);
        if (leftWindowNonZeroPixelCount < minWhitePixels && rightWindowNonZeroPixelCount < minWhitePixels) {
            // Didn't find the Team Shipping Element on either of the barcode elements
            // we looked at.
            RobotLogCommon.d(TAG, "Neither window contains the minimum number of non-zero pixels.");
            return new BarcodeReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsFreightFrenzy.BarcodeElementWindow.WINDOW_NPOS);
        }

        // Determine which window the shipping element is in.
        if (leftWindowNonZeroPixelCount >= rightWindowNonZeroPixelCount)
            return new BarcodeReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsFreightFrenzy.BarcodeElementWindow.LEFT);

        return new BarcodeReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsFreightFrenzy.BarcodeElementWindow.RIGHT);
    }

}