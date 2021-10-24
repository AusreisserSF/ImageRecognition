package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.constants.RobotConstantsFreightFrenzy;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.List;
import java.util.Map;

public class BarcodeRecognition {

    private static final String TAG = "BarcodeRecognition";
    private static final String imageFilePrefix = "Image_";
    private static final int MIN_WHITE_PIXELS = 20;

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public BarcodeRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.
    public BarcodeReturn findTeamScoringElement(ImageProvider pImageProvider, BarcodeParameters pBarcodeParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In BarcodeRecognition.findTeamScoringElement");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> barcodeImage = pImageProvider.getImage();
        if (barcodeImage.first == null)
            return new BarcodeReturn(true, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS); // don't crash

        String fileDate = TimeStamp.getLocalDateTimeStamp(barcodeImage.second);
        String outputFilenamePreamble = workingDirectory + imageFilePrefix + fileDate;

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        Mat imgOriginal = barcodeImage.first.clone();

        // If you don't convert RGB to BGR here then the _IMG.png file will be written
        // out with incorrect colors (gold will show up as blue).
        if (pImageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
            Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

        String imageFilename = outputFilenamePreamble + "_IMG.png";
        RobotLogCommon.d(TAG, "Writing original image " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imgOriginal);

        RobotLogCommon.d(TAG, "Image width " + imgOriginal.cols() + ", height " + imgOriginal.rows());
        if ((imgOriginal.cols() != pBarcodeParameters.imageParameters.resolution_width) ||
                (imgOriginal.rows() != pBarcodeParameters.imageParameters.resolution_height))
            throw new AutonomousRobotException(TAG,
                    "Mismatch between actual image width and expected image width " + pBarcodeParameters.imageParameters.resolution_width +
                            ", height " + pBarcodeParameters.imageParameters.resolution_height);

        // Crop the image to reduce distractions.
        Mat imageROI = imageUtils.getImageROI(imgOriginal,
                new Rect(pBarcodeParameters.imageParameters.image_roi.x,
                        pBarcodeParameters.imageParameters.image_roi.y,
                        pBarcodeParameters.imageParameters.image_roi.width,
                        pBarcodeParameters.imageParameters.image_roi.height));
        imageFilename = outputFilenamePreamble + "_ROI.png";
        RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imageROI);

        // Draw the barcode element windows (sub-ROIs) on the original image
        // so that we can see the placement during debugging.
        Mat barcodeElementWindows = imgOriginal.clone();

        // Get the left window within the ROI from the barcode parameters.
        // Remember - the barcode element windows are relative to the overall ROI.
        Map<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeParameters.BarcodeElement> barcodeElements =
                pBarcodeParameters.getBarcodeElements();
        BarcodeParameters.BarcodeElement leftBarcodeElement = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWithinROI.LEFT_WITHIN_ROI);
        Point leftWindowUpperLeft =
                new Point(pBarcodeParameters.imageParameters.image_roi.x + leftBarcodeElement.x, pBarcodeParameters.imageParameters.image_roi.y);
        Point leftWindowLowerRight = new Point(pBarcodeParameters.imageParameters.image_roi.x + leftBarcodeElement.x + leftBarcodeElement.width,
                pBarcodeParameters.imageParameters.image_roi.y + pBarcodeParameters.imageParameters.image_roi.height);

        // Get the right window within the ROI from the barcode parameters.
        BarcodeParameters.BarcodeElement rightBarcodeElement = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWithinROI.RIGHT_WITHIN_ROI);
        Point rightWindowUpperLeft = new Point(pBarcodeParameters.imageParameters.image_roi.x + rightBarcodeElement.x, pBarcodeParameters.imageParameters.image_roi.y);
        Point rightWindowLowerRight = new Point(pBarcodeParameters.imageParameters.image_roi.x + rightBarcodeElement.x + rightBarcodeElement.width,
                pBarcodeParameters.imageParameters.image_roi.y + pBarcodeParameters.imageParameters.image_roi.height);

        // Draw the windows in red.
        Imgproc.rectangle(barcodeElementWindows, leftWindowUpperLeft, leftWindowLowerRight, new Scalar(0, 0, 255), 3);
        Imgproc.rectangle(barcodeElementWindows, rightWindowUpperLeft, rightWindowLowerRight, new Scalar(0, 0, 255), 3);
        Imgcodecs.imwrite(outputFilenamePreamble + "_WIN.png", barcodeElementWindows);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_WIN.png");

        // Adapted from ...\OpenCV_Projects\OpenCVTestbed2\OpenCVTestbed2\GeneralTarget::analyzeSkystoneStripe
        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pBarcodeParameters.grayParameters.target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        int grayThresholdLow = pBarcodeParameters.grayParameters.low_threshold;
        RobotLogCommon.d(TAG, "Inverse threshold values: low " + grayThresholdLow + ", max 255 (white)");

        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // value assigned to pixels *under* the threshold value
                Imgproc.THRESH_BINARY_INV); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        // The next step is to reduce the thresholded image by column to a single row
        // of pixels that contains the maximum value for each column. Since our thresholded
        // image is binary there are only two values, 0 (black) and 255 (white), that figure
        // into the reduction.
        Mat reduced = new Mat();
        Core.reduce(thresholded, reduced, 0, Core.REDUCE_MAX);
        Imgcodecs.imwrite(outputFilenamePreamble + "_STRIPE.png", reduced);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_STRIPE.png");

        // Locate the white (because inverted) pixels that belong to the black Team Shipping Element.
        // See https://stackoverflow.com/questions/18147611/opencv-java-how-to-access-coordinates-returned-by-findnonzero
        Mat rawNonZeroLocations = new Mat();
        Core.findNonZero(reduced, rawNonZeroLocations);
        if (rawNonZeroLocations.empty()) {
            RobotLogCommon.d(TAG, "The stripe contains no white pixels.");
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS);
        }

        MatOfPoint nzLocations = new MatOfPoint(rawNonZeroLocations);
        List<Point> nzList = nzLocations.toList(); // crucial: I added this

        //## Remember: white pixels now represent the Team Scoring Element.
        RobotLogCommon.d(TAG, "Number of white pixels " + nzList.size());
        if (nzList.size() < MIN_WHITE_PIXELS) {
            RobotLogCommon.d(TAG, "The stripe contains " + nzList.size() + " white pixels, min = " + MIN_WHITE_PIXELS);
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS);
        }

        double firstWhitePixel = nzList.get(0).x;
        double lastWhitePixel = nzList.get(nzList.size() - 1).x;
        double whiteStripeCenter = firstWhitePixel + ((lastWhitePixel - firstWhitePixel) / 2);
        RobotLogCommon.d(TAG, "Found first white pixel at x position " + firstWhitePixel);
        RobotLogCommon.d(TAG, "Found last white pixel at x position " + lastWhitePixel);
        RobotLogCommon.d(TAG, "Length of white pixel stripe " + (lastWhitePixel - firstWhitePixel));
        RobotLogCommon.d(TAG, "Center of white stripe at x position " + whiteStripeCenter);

        // Calculate the barcode position.
        if ((firstWhitePixel >= leftBarcodeElement.x) && (firstWhitePixel <= leftBarcodeElement.x + leftBarcodeElement.width))
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.LEFT_WITHIN_ROI);

        if ((firstWhitePixel >= rightBarcodeElement.x) && (firstWhitePixel <= rightBarcodeElement.x + rightBarcodeElement.width))
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.RIGHT_WITHIN_ROI);

        // Didn't find the Team Shipping Element on either of the barcode elements
        // we looked at.
        return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS);
    }

}