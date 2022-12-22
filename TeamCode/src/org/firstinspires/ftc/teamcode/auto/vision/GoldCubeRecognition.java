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

public class GoldCubeRecognition {

    private static final String TAG = GoldCubeRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;
    private final RealSenseRecognition realSenseRecognition;

    public GoldCubeRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        alliance = pAlliance;
        realSenseRecognition = new RealSenseRecognition();
    }

    // Returns the result of image analysis.
    public RealSenseReturn recognizeGoldCube(ImageProvider pImageProvider,
                                             D405Configuration pD405Configuration,
                                             RobotConstantsPowerPlay.D405CameraId pOrientation,
                                             VisionParameters.ImageParameters pImageParameters,
                                             GoldCubeParameters pGoldCubeParameters,
                                             RobotConstants.RecognitionPath pGoldCubeRecognitionPath) throws InterruptedException, IOException {

        RobotLogCommon.d(TAG, "In GoldCubeRecognition.recognizeGoldCube");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> goldCubeImage = pImageProvider.getImage();
        if (goldCubeImage == null)
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in ImageUtils.preProcessImage.
        Mat imgOriginal = goldCubeImage.first.clone();
        String fileDate = TimeStamp.getLocalDateTimeStamp(goldCubeImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        Mat imageROI = ImageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Subject the ROI to depth filtering on all paths.
        short[] depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);
        Mat depthImageROI = RealSenseUtils.removeBackground(imageROI, pImageParameters,
                pD405Configuration, depthArray,
                pGoldCubeParameters.depthParameters.minDepth,
                pGoldCubeParameters.depthParameters.maxDepth);

        Imgcodecs.imwrite(outputFilenamePreamble + "_ROI_RANGE.png", depthImageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ROI_RANGE.png");

        RobotLogCommon.d(TAG, "Recognition path " + pGoldCubeRecognitionPath);
        switch (pGoldCubeRecognitionPath) {
            case RED_CHANNEL_GRAYSCALE -> {
                return realSenseRecognition.redChannelPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, RobotConstantsPowerPlay.WIDTH_OF_GOLD_CUBE,
                        outputFilenamePreamble,
                        pImageParameters, pGoldCubeParameters.grayscaleParameters, pGoldCubeParameters.depthParameters);
            }
            case COLOR -> {
                return realSenseRecognition.colorPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, RobotConstantsPowerPlay.WIDTH_OF_GOLD_CUBE,
                        outputFilenamePreamble,
                        pImageParameters, pGoldCubeParameters.hsvParameters, pGoldCubeParameters.depthParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }
}