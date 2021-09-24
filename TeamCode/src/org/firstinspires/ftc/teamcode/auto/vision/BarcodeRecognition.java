package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Map;

public class BarcodeRecognition {

    private static final String TAG = "BarcodeRecognition";
    private static final String imageFilePrefix = "Image_";

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public BarcodeRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of image analysis.

    //**TODO Watch out for glare (see Image_04170921_15573_ROI_2021_07_14_19-18-10_214_GRAY.png)
    // Matte black may be best.
    // and shadows (see Image_04170921_15573_ROI_2021_07_14_19-18-10_214_SHARP.png)
    //** Make sure the height of the custom element allows us to define an ROI that cuts off its
    // shadow.
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

        // Adapted from ...\OpenCV_Projects\OpenCVTestbed2\OpenCVTestbed2\GeneralTarget::analyzeSkystoneStripe
        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pBarcodeParameters.grayParameters.target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        //??TODO Do you need a sharpening kernel? First try without.
        /*
	    // 7/14/2021 Try an image sharpening kernel. See also pyimagesearch Lesson
	    // 102 for a different kernel and the scratchpad file UnsharpMask.cpp for
	    // more. 
	    // See https://learnopencv.com/image-filtering-using-convolution-in-opencv/
	    Mat sharp_img;
	    Mat kernel3 = (Mat_<double>(3, 3) << 0, -1, 0,
		-1, 5, -1,
		0, -1, 0);
	    filter2D(adjustedGray, sharp_img, -1, kernel3, Point(-1, -1), 0, BORDER_DEFAULT);
	    imwrite(pOutputFilenamePreamble + "_SHARP.png", sharp_img);
	    LogManager::log("Writing sharpened grayscale image " + pOutputFilenamePreamble + "_SHARP.png");
        */

        int grayThresholdLow = pBarcodeParameters.grayParameters.low_threshold;
        RobotLogCommon.d(TAG, "Inverse threshold values: low " + grayThresholdLow + ", max 255 (white)");

        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // value assigned to pixels *under* the threshold value
                Imgproc.THRESH_BINARY_INV); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        //??TODO erode/dilate to eliminate white noise?

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
        MatOfPoint nzLocations = new MatOfPoint();
        Core.findNonZero(reduced, nzLocations);

        //## Remember: white pixels now represent the Team Scoring Element.
        //**TODO use a const with better minimum number of white pixels.
        RobotLogCommon.d(TAG, "Number of white pixels " + nzLocations.size().width);
        if (nzLocations.size().width < 3) {
            RobotLogCommon.d(TAG, "The stripe contains < 3 white pixels.");
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS);
        }

        // MatOfPoint.get returns double[] where [0] = x, [1] = y
        double firstWhitePixel = nzLocations.get(0, 0)[0];
        double lastWhitePixel = nzLocations.get(0, nzLocations.cols() - 1)[1];
        double whiteStripeCenter = firstWhitePixel + ((lastWhitePixel - firstWhitePixel) / 2);
        RobotLogCommon.d(TAG, "Found first white pixel at x position " + firstWhitePixel);
        RobotLogCommon.d(TAG, "Found last white pixel at x position " + lastWhitePixel);
        RobotLogCommon.d(TAG, "Length of white pixel stripe " + (lastWhitePixel - firstWhitePixel));
        RobotLogCommon.d(TAG, "Center of white stripe at x position " + whiteStripeCenter);

        // Draw a small circle in the original image at the center of the white stripe. The circle should
        // appear n the Team Scoring Element.
        Mat markTeamScoringElement = imageROI.clone();

        //**TODO hardcode center of circle - dependent on stripeROI above
        //Imgproc.circle(markTeamScoringElement, new Point(whiteStripeCenter, 67), 10, new Scalar(0, 0, 255), 3, 8, 0);
        //Imgcodecs.imwrite(pOutputFilenamePreamble + "_CIR.png", markTeamScoringElement);
        //RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_CIR.png");

        // Calculate the barcode position.
        // Get the left window within the ROI from the barcode parameters.
        Map<RobotConstantsFreightFrenzy.BarcodeElementWithinROI, BarcodeParameters.BarcodeElement> barcodeElements =
                pBarcodeParameters.getBarcodeElements();
        BarcodeParameters.BarcodeElement leftBarcodeElement = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWithinROI.LEFT_WITHIN_ROI);
        if ((firstWhitePixel >= leftBarcodeElement.x) && (firstWhitePixel <= leftBarcodeElement.x + leftBarcodeElement.width))
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.LEFT_WITHIN_ROI);

        // Get the right window within the ROI from the barcode parameters.
        BarcodeParameters.BarcodeElement rightBarcodeElement = barcodeElements.get(RobotConstantsFreightFrenzy.BarcodeElementWithinROI.RIGHT_WITHIN_ROI);
        if ((firstWhitePixel >= rightBarcodeElement.x) && (firstWhitePixel <= rightBarcodeElement.x + rightBarcodeElement.width))
            return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.RIGHT_WITHIN_ROI);

        // Didn't find the Team Shipping Element on either of the barcode elements
        // we looked at.
        return new BarcodeReturn(false, RobotConstantsFreightFrenzy.BarcodeElementWithinROI.BARCODE_ELEMENT_NPOS);
    }

}