// Partial port of ImageUtils.cpp: only those functions that are
// needed for ring recognition.
package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.*;

import static org.opencv.imgcodecs.Imgcodecs.IMREAD_COLOR;

public class ImageUtils {

    public static final String TAG = ImageUtils.class.getSimpleName();

    // Load an image.
    public Mat loadImage(String pInputFilename) {
        RobotLogCommon.d(TAG, "File name " + pInputFilename);
        Mat imageOut = Imgcodecs.imread(pInputFilename, IMREAD_COLOR);
        if (imageOut.empty())
            throw new AutonomousRobotException(TAG, "Could not find or open the image " + pInputFilename);

        RobotLogCommon.d(TAG, "Image width " + imageOut.cols() + ", height " + imageOut.rows());
        return imageOut;
    }

    // Define a region of interest.
    public Mat getImageROI(Mat pSrcImage, Rect pROIDefinition) {

        if ((pROIDefinition.height == 0) && (pROIDefinition.width == 0)) {
            RobotLogCommon.d(TAG, "At least one ROI dimension was 0");
            return new Mat();
        }

        Mat roi = new Mat(pSrcImage, pROIDefinition);
        RobotLogCommon.d(TAG, "Image ROI x " + pROIDefinition.x + ", y " + pROIDefinition.y + ", width " + pROIDefinition.width + ", height " + pROIDefinition.height);
        return roi;
    }

    // Adjust the brightness of a grayscale image.
    public Mat adjustGrayscaleBrightness(Mat pGray, int pTarget) {
        int medianGray = getSingleChannelMedian(pGray);
        RobotLogCommon.d(TAG, "Original image: grayscale median " + medianGray);
        RobotLogCommon.d(TAG, "Grayscale median target " + pTarget);

        // adjustment = target - median;
        int adjustment = pTarget - medianGray;
        Mat adjustedGray = new Mat();
        pGray.convertTo(adjustedGray, -1, 1, adjustment);
        RobotLogCommon.d(TAG, "Grayscale adjustment " + adjustment);

        return adjustedGray;
    }

    // Adjust image saturation and value levels in the image to match the targets.
    public Mat adjustSaturationAndValue(Mat pHSVImage, int pSatLowTarget, int pValLowTarget) {
        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(pHSVImage, channels);

        // Get the median of the S channel.
        int medianSaturation = getColorChannelMedian(channels.get(1), new Mat());

        // Get the median of the V channel.
        int medianValue = getColorChannelMedian(channels.get(2), new Mat());

        RobotLogCommon.d(TAG, "HSV saturation channel median " + medianSaturation);
        RobotLogCommon.d(TAG, "HSV value channel median " + medianValue);

        // adjustment = target - median;
        int satAdjustment = pSatLowTarget - medianSaturation;
        int valAdjustment = pValLowTarget - medianValue;
        channels.get(1).convertTo(channels.get(1), -1, 1, satAdjustment);
        channels.get(2).convertTo(channels.get(2), -1, 1, valAdjustment);

        RobotLogCommon.d(TAG, "Adjust HSV saturation by " + satAdjustment);
        RobotLogCommon.d(TAG, "Adjust HSV value by " + valAdjustment);

        // Merge the channels back together.
        Mat adjustedImage = new Mat();
        Core.merge(channels, adjustedImage);
        return adjustedImage;
    }

    //**TODO TEST and correct in Android.
    // See https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html
    public int getDominantHSVHue(Mat pHSVImageIn, Mat pMask) {

        List<Mat> channelsHSV = new ArrayList<>();
        Core.split(pHSVImageIn, channelsHSV);

        // Set the number of bins and range for HSV hue in OpenCV.
        int hueHistSize = 180; // number of bins
        float hueRange[] = {0, 180};
        MatOfFloat hueHistRange = new MatOfFloat(hueRange);

        Mat hueHist = new Mat();
        boolean accumulate = false;
        // MatOfInt(0) indicates channel 0 -> hue
        Imgproc.calcHist(channelsHSV, new MatOfInt(0), new Mat(), hueHist, new MatOfInt(hueHistSize), hueHistRange, accumulate);

        // Normalize the result to [ 0, hue_hist.rows ]
        //## Normalization is done for graphings, which we don't need.
        // normalize(hue_hist, hue_hist, 0, hue_hist.rows, NORM_MINMAX, -1, Mat());

        //## DEBUG
        //RobotLogCommon.d(TAG, "Hue histogram: bins");
        //for (int i = 0; i < 180; i++)
        //  if (hue_hist.at<float>(i) != 0.0f)
        //	RobotLogCommon.d(TAG, "Bin " + to_string(i) + " = " + to_string(hue_hist.at<float>(i)));

        // Get the bin with the greatest pixel count.
        Core.MinMaxLocResult mmlResult = Core.minMaxLoc(hueHist);
        RobotLogCommon.d(TAG, "Hue histogram: largest bin x " + mmlResult.maxLoc.x + ", y " + mmlResult.maxLoc.y + ", count " + mmlResult.maxVal);

        // The y-coordinate of the maxLoc Point contains the index to the bin
        // with the greatest value. The index to the bin is the hue itself.
        int dominantHue = (int) mmlResult.maxLoc.y;
        RobotLogCommon.d(TAG, "HSV dominant hue " + dominantHue);

        return dominantHue;
    }

    // The input image is BGR. This function converts it to HSV, adjusts the saturation and value according
    // to the targets in the HSVParameters, and applies the OpenCV thresholding function inRange using
    // the hue range in the HSVParameters.
    // In the OpenCV tutorial, no blurring is applied before inRange (unlike grayscale thresholding).
    // https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
    public Mat applyInRange(Mat pInputROI, String pOutputFilenamePreamble,
                            VisionParameters.HSVParameters pHSVParameters) {

        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(pInputROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        // Adjust the HSV saturation and value levels in the image to match the targets.
        int hueLow = pHSVParameters.hue_low;
        int hueHigh = pHSVParameters.hue_high;
        int satTarget = pHSVParameters.saturation_target;
        int satHigh = 255;
        int valTarget = pHSVParameters.value_target;
        int valHigh = 255;
        RobotLogCommon.d(TAG, "Target hue levels: low " + hueLow + ", high " + hueHigh);

        // Adjust saturation and value to the target levels.
        Mat adjusted = adjustSaturationAndValue(hsvROI, satTarget, valTarget);
        RobotLogCommon.d(TAG, "Adjusted image levels: saturation low " + satTarget + ", value low " + valTarget);

        // Convert back to BGR.
        //## This debugging step will not be needed in production.
        Mat adjustedBGR = new Mat();
        Imgproc.cvtColor(adjusted, adjustedBGR, Imgproc.COLOR_HSV2BGR);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ.png", adjustedBGR);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ.png");

        // Use inRange to threshold to binary.
        // Account for hue ranges that cross the 180 degree boundary.
        // Red, for example, might have a hueLow of 170 and a hueHigh of
        // 10.
        // See https://stackoverflow.com/questions/32522989/opencv-better-detection-of-red-color

        Mat thresholded = new Mat();
        int inRangeSatLow = pHSVParameters.saturation_low_threshold;
        int inrangeValLow = pHSVParameters.value_low_threshold;
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: hue low " + hueLow + ", hue high " + hueHigh);
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: saturation low " + inRangeSatLow + ", value low " + inrangeValLow);

        // Sanity check for hue.
        if (!((hueLow >= 0 && hueLow <= 180) && (hueHigh >= 0 && hueHigh <= 180) &&
                (hueLow != hueHigh)))
            throw new AutonomousRobotException(TAG, "Hue out of range");

        // Normal hue range.
        if (hueLow < hueHigh)
            Core.inRange(adjusted, new Scalar(hueLow, inRangeSatLow, inrangeValLow), new Scalar(hueHigh, satHigh, valHigh), thresholded);
        else {
            // For a hue range from the XML file of low 170, high 10
            // the following yields two new ranges: 170 - 180 and 0 - 10.
            Mat range1 = new Mat();
            Mat range2 = new Mat();
            Core.inRange(adjusted, new Scalar(hueLow, inRangeSatLow, inrangeValLow), new Scalar(180, satHigh, valHigh), range1);
            Core.inRange(adjusted, new Scalar(0, inRangeSatLow, inrangeValLow), new Scalar(hueHigh, satHigh, valHigh), range2);
            Core.bitwise_or(range1, range2, thresholded);
        }

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ_THR.png");

        return thresholded;
    }

    // Combine the frequently associated steps of applying inRange to
    // an HSV image and then finding contours in the thresholded output.
    //!! findContours works without blurring and morphological opening.
    //!! But there are fewer artifacts in the contour recognition if only
    //!! morphological opening is included.
    public List<MatOfPoint> applyInRangeAndFindContours(Mat pInputROI,
                                                        String pOutputFilenamePreamble, VisionParameters.HSVParameters pHSVParameters) {

        Mat thresholded = applyInRange(pInputROI, pOutputFilenamePreamble, pHSVParameters);
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Identify the contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morphed, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    // The target low hue may be greater than the target high hue. For example,
    // target low 170, target high 10.
    public boolean hueInRange(int pHue, int pTargetLow, int pTargetHigh) {

        // Sanity check for hue.
        if (!((pTargetLow >= 0 && pTargetLow <= 180) && (pTargetHigh >= 0 && pTargetHigh <= 180) &&
                (pTargetLow != pTargetHigh)))
            throw new AutonomousRobotException(TAG, "Hue out of range");

        if (pTargetLow < pTargetHigh)
            return (pHue >= pTargetLow && pHue <= pTargetHigh);

        return (pHue >= pTargetLow && pHue <= 180) ||
                (pHue >= 0 && pHue <= pTargetHigh);
    }

    // Attempt at a common path for converting an RGB (OpenCV BGR) image
    // to grayscale, adjusting the grayscale image to a target, performing
    // morphological opening, blurring the image, and thresholding it.
    // This function is designed to work in advance of HoughCircles and
    // findContours. This accords with our policy of controlling the
    // thresholding, even though the standard path is to pass grayscale
    // images in to HoughCircles (because it calls Canny).

    // Note that pyimagesearch always blurs grayscale images before thresholding:
    // https://www.pyimagesearch.com/2021/04/28/opencv-thresholding-cv2-threshold/
    // This OpenCV tutorial does not:
    // https://docs.opencv.org/3.4/db/d8e/tutorial_threshold.html
    // But this one does with convincing results:
    // https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html
    public Mat performThreshold(Mat pBGRInputROI, String pOutputFilenamePreamble,
                                int pGrayscaleTarget, int pLowThreshold) {

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(pBGRInputROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = adjustGrayscaleBrightness(grayROI, pGrayscaleTarget);
        Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + pOutputFilenamePreamble + "_ADJ.png");

        Mat morphed = new Mat();
        Imgproc.erode(adjustedGray, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        // Remove noise by Gaussian blurring.
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(morphed, blurred, new Size(5, 5), 0);

        RobotLogCommon.d(TAG, "Threshold values: low " + pLowThreshold + ", high 255");

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(blurred, thresholded,
                pLowThreshold,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        Imgcodecs.imwrite(pOutputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + pOutputFilenamePreamble + "_ADJ_THR.png");

        return thresholded;
    }

    // Get the median of any single-channel Mat.
    public int getSingleChannelMedian(Mat pSingleChannelMat) {

        if ((pSingleChannelMat.dims() != 2) || (!pSingleChannelMat.isContinuous()))
            throw new AutonomousRobotException(TAG, "Expected a single-channel Mat");

        byte[] byteBuff = new byte[(int) pSingleChannelMat.total()];
        int[] intBuff = new int[(int) pSingleChannelMat.total()];
        int buffLength = byteBuff.length;
        pSingleChannelMat.get(0, 0, byteBuff);

        // !! Since Java does not have an unsigned char data type, the byte values
        // may come out as negative. So we have to use a separate array of ints and
        // copy in the bytes with the lower 8 bytes preserved.
        // https://stackoverflow.com/questions/9581530/converting-from-byte-to-int-in-java
        for (int i = 0; i < buffLength; i++)
            intBuff[i] = Byte.toUnsignedInt(byteBuff[i]); // or byteBuff[i] & 0xFF;

        Arrays.sort(intBuff);
        return (intBuff[buffLength / 2] + (intBuff[(buffLength / 2) - 1])) / 2;
    }

    // From https://stackoverflow.com/questions/40669684/opencv-sorting-contours-by-area-in-java
    // See lambda in https://stackoverflow.com/questions/24378646/finding-max-with-lambda-expression-in-java
    public Optional<MatOfPoint> getLargestContour(List<MatOfPoint> pContours) {
        return pContours.stream().max(Comparator.comparing(Imgproc::contourArea));
    }

    //**TODO TEST and correct in Android.
    public Point getContourCentroid(MatOfPoint pOneContour) {
        Moments moments = Imgproc.moments(pOneContour);
        Point centroid = new Point(moments.get_m10() / moments.get_m00(),
                moments.get_m01() / moments.get_m00());
        return centroid;
    }

    // Ported from 2018-2019 auto/VisionOCV.java
    // Compute the angle from the front-center of the robot to the center of the
    // object of interest. The parameeter pImageWidth is the width of the entire
    // image, not just the ROI, and the parameter pObjectCentroidX is the x
    // offset from the left-hand edge of the emtire image to the center of the
    // object of interest.
    public double computeAngleToObjectCenter(int pImageWidth, int pObjectCentroidX, double pCameraFieldOfView) {

        // This solution follows https://stackoverflow.com/questions/32524787/how-to-compute-horizontal-angle-of-a-pixel-from-a-computer-vision-camera
        // in the answer from Leandro Caniglia:
	/*
	   double image_width_pixels = CAMERA_HORIZONTAL_RESOLUTION_PIXELS;
	   double fov_radians = CAMERA_HORIZONTAL_FIELD_OF_VIEW_RADIANS;
	   double f = ( image_width_pixels / 2.0 ) / tan( fov_radians / 2.0 );
	   int x = <<< horizontal pixel coordinate >>>;
	   double angle_radians = atan( x / f );
	*/

        // tan(FOV / 2.0) = (pImageWidth / 2.0) [opposite] / adjacent [distance from the camera to the image in pseudo pixels]
        double halfFOVRadians = Math.toRadians(pCameraFieldOfView / 2.0);
        double distanceToImageInPseudoPixels = (pImageWidth / 2.0) / Math.tan(halfFOVRadians);
        double angleRadians = Math.atan(((pImageWidth / 2.0) - pObjectCentroidX) / distanceToImageInPseudoPixels);

        return Math.toDegrees(angleRadians);
    }

    // Get the median of a color channel.
    private int getColorChannelMedian(Mat pChannel, Mat pMask) {
        // If we're dealing with a non-masked image then we just take the median
        // of all the pixels.
        if (pMask.total() == 0) {
            return getSingleChannelMedian(pChannel);
        } else
            throw new AutonomousRobotException(TAG, "getColorChannelMedian with mask is not supported at this time");
    }

}

