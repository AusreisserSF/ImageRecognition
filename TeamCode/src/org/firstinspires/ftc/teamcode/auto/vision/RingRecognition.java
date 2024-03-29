package org.firstinspires.ftc.teamcode.auto.vision;

//!! Android only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;

import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsUltimateGoal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;

public class RingRecognition {

    private static final String TAG = "RingRecognition";
    private static final String imageFilePrefix = "Image_";

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public RingRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of the image analysis.
    public RingReturn findGoldRings(ImageProvider pImageProvider, RingParameters pRingParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In RingRecognition.findGoldRings");

        // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> ringImage = pImageProvider.getImage();
        if (ringImage.first == null)
            return new RingReturn(true, RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_NPOS); // don't crash

        String fileDate = TimeStamp.getLocalDateTimeStamp(ringImage.second);
        String outputFilenamePreamble = workingDirectory + imageFilePrefix + fileDate;

        // The image may be RGB (from a camera) or BGR ( OpenCV imread from a file).
        Mat imgOriginal = ringImage.first.clone();

        // If you don't convert RGB to BGR here then the _IMG.png file will be written
        // out with incorrect colors (gold will show up as blue).
        if (pImageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
            Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

        String imageFilename = outputFilenamePreamble + "_IMG.png";
        RobotLogCommon.d(TAG, "Writing original image " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imgOriginal);

        RobotLogCommon.d(TAG, "Image width " + imgOriginal.cols() + ", height " + imgOriginal.rows());
        if ((imgOriginal.cols() != pRingParameters.imageParameters.resolution_width) ||
                (imgOriginal.rows() != pRingParameters.imageParameters.resolution_height))
            throw new AutonomousRobotException(TAG,
                    "Mismatch between actual image width and expected image width " + pRingParameters.imageParameters.resolution_width +
                            ", height " + pRingParameters.imageParameters.resolution_height);

        // Crop the image to reduce distractions.
        Mat imageROI = imageUtils.getImageROI(imgOriginal,
                new Rect(pRingParameters.imageParameters.image_roi.x,
                        pRingParameters.imageParameters.image_roi.y,
                        pRingParameters.imageParameters.image_roi.width,
                        pRingParameters.imageParameters.image_roi.height));
        imageFilename = outputFilenamePreamble + "_ROI.png";
        RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imageROI);

        // Adapted from ...\OpenCV_Projects\OpenCVTestbed2\OpenCVTestbed2\GeneralTarget.cpp
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(imageROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        // Adjust the HSV saturation and value levels in the image to match the targets.
        int goldHueLow = pRingParameters.hsvParameters.hue_low;
        int goldHueHigh = pRingParameters.hsvParameters.hue_high;
        int goldSatTarget = pRingParameters.hsvParameters.saturation_target;
        int goldSatHigh = 255;
        int goldValTarget = pRingParameters.hsvParameters.value_target;
        int goldValHigh = 255;
        RobotLogCommon.d(TAG, "Target hue levels: low " + goldHueLow + ", high " + goldHueHigh);

        // Adjust saturation and value to the target levels.
        Mat adjusted = imageUtils.adjustSaturationAndValue(hsvROI, goldSatTarget, goldValTarget);
        RobotLogCommon.d(TAG, "Adjusted image levels: saturation low " + goldSatTarget + ", value low " + goldValTarget);

        // Convert back to BGR.
        //## This debugging step will not be needed in production.
        //Mat adjustedBGR = new Mat();
        //Imgproc.cvtColor(adjusted, adjustedBGR, Imgproc.COLOR_HSV2BGR);
        //Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedBGR);
        //RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ.png");

        // Use inRange to threshold to binary.
        Mat thresholded = new Mat();
        int inRangeSatLow = pRingParameters.hsvParameters.saturation_low_threshold;
        int inrangeValLow = pRingParameters.hsvParameters.value_low_threshold;
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: hue low " + goldHueLow + ", hue high " + goldHueHigh);
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: saturation low " + inRangeSatLow + ", value low " + inrangeValLow);

        Core.inRange(adjusted, new Scalar(goldHueLow, inRangeSatLow, inrangeValLow), new Scalar(goldHueHigh, goldSatHigh, goldValHigh), thresholded);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        // Instead of trying to find contours, just count the number of white pixels,
        // which are those that are in range for the gold color.
        int white_pixels = Core.countNonZero(thresholded);
        RobotLogCommon.d(TAG, "Number of white pixels " + white_pixels);

        // If the number of white pixels is less than the minimum for a single
        // ring then assume there are no rings on the field.
        RobotConstantsUltimateGoal.TargetZone targetZone;
        if (white_pixels < pRingParameters.minimum_pixel_count_1_ring) {
            targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_A;
            RobotLogCommon.d(TAG, "No rings detected: set Target Zone Goal A");
        } else

            // If the number of white pixels is greater than the minimum for a stack
            // of  4 rings then the target is Goal C.
            if (white_pixels > pRingParameters.minimum_pixel_count_4_rings) {
                targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_C;
                RobotLogCommon.d(TAG, "Found four rings: set Target Zone Goal C");
            } else { // Must be 1 ring.
                targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_B;
                RobotLogCommon.d(TAG, "Found one ring: set Target Zone Goal B");
            }

        return new RingReturn(false, targetZone);
    }

}
