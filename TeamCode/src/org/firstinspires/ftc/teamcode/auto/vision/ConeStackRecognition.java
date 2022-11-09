package org.firstinspires.ftc.teamcode.auto.vision;

//!! IntelliJ only

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.TimeStamp;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.IOException;
import java.time.LocalDateTime;

// This class uses ConeStackParameters, which include parameters for red
// grayscale, red hsv and blue grayscale, blue hsv. Both grayscale paths
// actually use the red channel of the image.
public class ConeStackRecognition {

    private static final String TAG = ConeStackRecognition.class.getSimpleName();

    private final String workingDirectory;
    private final RobotConstants.Alliance alliance;
    private final RealSenseRecognition realSenseRecognition;

    public ConeStackRecognition(RobotConstants.Alliance pAlliance) {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        alliance = pAlliance;
        realSenseRecognition = new RealSenseRecognition();
    }

    // Returns the result of image analysis.
    // The targets are:
    // A stack of 1 to 5 red cones
    // A stack of 1 to 5 blue cones
    public RealSenseReturn recognizeConeStack(ImageProvider pImageProvider,
                                              D405Configuration pD405Configuration,
                                              RobotConstantsPowerPlay.D405Orientation pOrientation,
                                              VisionParameters.ImageParameters pImageParameters,
                                              ConeStackParameters pConeStackParameters,
                                              RobotConstantsPowerPlay.ConeStackRecognitionPath pConeStackRecognitionPath) throws InterruptedException, IOException {

        RobotLogCommon.d(TAG, "In ConeStackRecognition.recognizeConeStack");

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> coneStackImage = pImageProvider.getImage();
        if (coneStackImage == null)
            return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

        // The image may be RGB (from a camera) or BGR (OpenCV imread from a file).
        // OpenCV wants BGR; the possible conversion is taken care of in ImageUtils.preProcessImage.
        Mat imgOriginal = coneStackImage.first.clone();
        String fileDate = TimeStamp.getLocalDateTimeStamp(coneStackImage.second);
        String outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, RobotConstants.imageFilePrefix, fileDate);
        Mat imageROIOriginal = ImageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        // Subject the ROI to depth filtering on all paths.
        short[] depthArray = RealSenseUtils.getDepthArrayFromFile(pImageParameters);
        Mat imageROI = RealSenseUtils.removeBackground(imageROIOriginal, pImageParameters,
                pD405Configuration, depthArray,
                pConeStackParameters.depthParameters.minDepth,
                pConeStackParameters.depthParameters.maxDepth);

        Imgcodecs.imwrite(outputFilenamePreamble + "_ROI_RANGE.png", imageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ROI_RANGE.png");

        RobotLogCommon.d(TAG, "Recognition path " + pConeStackRecognitionPath);
        switch (pConeStackRecognitionPath) {
            case GRAYSCALE -> {
                VisionParameters.GrayParameters grayscaleParameters;
                if (alliance == RobotConstants.Alliance.RED)
                    grayscaleParameters = pConeStackParameters.redGrayscaleParameters;
                else
                if (alliance == RobotConstants.Alliance.BLUE)
                    grayscaleParameters = pConeStackParameters.blueGrayscaleParameters;
                else
                    return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

                return realSenseRecognition.redChannelPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, outputFilenamePreamble,
                pImageParameters, grayscaleParameters, pConeStackParameters.depthParameters);
            }
            case COLOR -> {
                VisionParameters.HSVParameters hsvParameters;
                if (alliance == RobotConstants.Alliance.RED)
                    hsvParameters = pConeStackParameters.redHSVParameters;
                else
                if (alliance == RobotConstants.Alliance.BLUE)
                    hsvParameters = pConeStackParameters.blueHSVParameters;
                else
                    return new RealSenseReturn(RobotConstants.RecognitionResults.RECOGNITION_INTERNAL_ERROR); // don't crash

                return realSenseRecognition.colorPath(imageROI,
                        pD405Configuration, pOrientation,
                        depthArray, outputFilenamePreamble,
                pImageParameters, hsvParameters, pConeStackParameters.depthParameters);
            }
            default -> throw new AutonomousRobotException(TAG, "Unrecognized recognition path");
        }
    }

}