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
                                             RobotConstantsPowerPlay.D405Orientation pOrientation,
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
        Mat imageROIOriginal = ImageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Subject the ROI to depth filtering on all paths.
        short[] depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);
        Mat imageROI = RealSenseUtils.removeBackground(imageROIOriginal, pImageParameters,
                pD405Configuration, depthArray,
                pJunctionParameters.depthParameters.minDepth,
                pJunctionParameters.depthParameters.maxDepth);

        Imgcodecs.imwrite(outputFilenamePreamble + "_ROI_RANGE.png", imageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ROI_RANGE.png");

        RobotLogCommon.d(TAG, "Recognition path " + pJunctionRecognitionPath);
        switch (pJunctionRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE -> {
                return realSenseRecognition.redChannelPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, outputFilenamePreamble,
                        pImageParameters, pJunctionParameters.grayscaleParameters, pJunctionParameters.depthParameters);
            }
            case COLOR -> {
                return realSenseRecognition.colorPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, outputFilenamePreamble,
                        pImageParameters, pJunctionParameters.hsvParameters, pJunctionParameters.depthParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

}