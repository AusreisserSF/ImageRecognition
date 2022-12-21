package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.time.LocalDateTime;

// For recognizing a PowerPlay junction and getting the angle and
// distance.
public class JunctionRecognition {

    private static final String TAG = JunctionRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;
    private final RealSenseRecognition realSenseRecognition;

    public JunctionRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        alliance = pAlliance;
        realSenseRecognition = new RealSenseRecognition();
    }

    // Returns the result of image analysis.
    public RealSenseReturn recognizeJunction(ImageProvider pImageProvider,
                                             D405Configuration pD405Configuration,
                                             RobotConstantsPowerPlay.D405CameraId pCameraId,
                                             VisionParameters.ImageParameters pImageParameters,
                                             JunctionParameters pJunctionParameters,
                                             RobotConstantsPowerPlay.JunctionRecognitionPath pJunctionRecognitionPath) throws InterruptedException, IOException {

        RobotLogCommon.d(TAG, "In JunctionRecognition.recognizeJunction");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> junctionImage = pImageProvider.getImage();
        if (junctionImage == null)
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in ImageUtils.preProcessImage.
        Mat imgOriginal = junctionImage.first.clone();
        String fileDate = TimeStamp.getLocalDateTimeStamp(junctionImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Subject the ROI to depth filtering on all paths.
        short[] depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);

        //**TODO VERY VERY TEMP - ROI_RANGE input ONLY
        /*
        Mat depthImageROI = RealSenseUtils.removeBackground(imageROI, pImageParameters,
                pD405Configuration, depthArray,
                pJunctionParameters.depthParameters.minDepth,
                pJunctionParameters.depthParameters.maxDepth);

        Imgcodecs.imwrite(outputFilenamePreamble + "_ROI_RANGE.png", depthImageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ROI_RANGE.png");

         */

        //**TODO get more pictures and look at the ROI_RANGE file. Preliminary
        // indications are the it blocks out all extraneous black objects and
        // other junctions on the field. Consider using this file for
        // thresholding on black.

        //**TODO TEMP TEMP Try the ROI_RANGE path here.
        Mat thresholded = ImageUtils.performThreshold(imageROI, outputFilenamePreamble,
                pJunctionParameters.junctionCapGrayscaleParameters.median_target,
                pJunctionParameters.junctionCapGrayscaleParameters.threshold_low);

        return RealSenseUtils.getAngleAndDistance(imageROI, thresholded,
                pD405Configuration, pCameraId, depthArray,
                outputFilenamePreamble, pImageParameters, pJunctionParameters.depthParameters);

        //**TODO or use the RotatedRectangle of the junction.

        //**TODO Recognition of a junction needs to be done in two parts because
        // the most reliable target is the black cap on the top of the junction.
        // Using the black cap eliminates the problem of having to account for
        // a junction that is not perfectly vertical. So the first step is to
        // isolate the black cap onn the top of our target junction without
        // becoming confused by other black objects and other junctions on the
        // field.

        // The first step is to establish a region of interest - we should be
        // close enough to the target junction to crop off the caps of
        // neighboring junctions - and then isolate black objects by inverse
        // thresholding. One of the thresholded objects will be the cap for
        // our target junction.

        // But which one? The best way is to create contours and bounding
        // boxes for each of the black objects (now white because of the
        // inverse thresholding), apply an area filter, and then figure out
        // which black object is immediately adjacent to a junction pole.

        // This means that we need a separate recognition path for the
        // junction poles. The path can be either red channel grayscale or
        // color but the ROI dimensions *must* be the same as those for the
        // knob recognition.

        // For each black object that has passed all of the filters, define
        // a 10x10 pixel square just above (because our original image is
        // upside-down) the black object. Then check the square at the same
        // coordinates in the thresholded junction pole ROI; if the square
        // contains non-zero pixels we have a match.

        // From the depth array returned from the camera we can get the
        // depth values for any pixel in the black object.

        //**TODO TEMP TEMP while working on ROI_RANGE above
        /*
        RobotLogCommon.d(TAG, "Recognition path " + pJunctionRecognitionPath);
        switch (pJunctionRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE -> {
                return realSenseRecognition.redChannelPath(imageROI,
                        pD405Configuration, pCameraId,
                        depthArray, outputFilenamePreamble,
                        pImageParameters, pJunctionParameters.junctionPoleGrayscaleParameters, pJunctionParameters.depthParameters);
            }
            case COLOR -> {
                return realSenseRecognition.colorPath(imageROI,
                        pD405Configuration, pCameraId,
                        depthArray, outputFilenamePreamble,
                        pImageParameters, pJunctionParameters.junctionPoleHsvParameters, pJunctionParameters.depthParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
         */
    }

}