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
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class SignalSleeveRecognition {

    private static final String TAG = SignalSleeveRecognition.class.getSimpleName();

    private final String workingDirectory;
    private String outputFilenamePreamble;
    private Mat imageROI;
    private RobotConstants.Alliance alliance;

    public SignalSleeveRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
    }

    public SignalSleeveReturn recognizeSignalSleeve(ImageProvider pImageProvider,
                                                    VisionParameters.ImageParameters pImageParameters,
                                                    SignalSleeveParameters pSignalSleeveParameters,
                                                    RobotConstants.Alliance pAlliance,
                                                    RobotConstantsPowerPlay.SignalSleeveRecognitionPath pSignalSleeveRecognitionPath) throws InterruptedException {

        RobotLogCommon.d(TAG, "In SignalSleeveRecognition.recognizeSignalSleeve");

        alliance = pAlliance;

        // LocalDateTime requires Android minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, LocalDateTime> signalSleeveImage = pImageProvider.getImage();
        if (signalSleeveImage == null)
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.OCV_ERROR); // don't crash

        // The image is always BGR (OpenCV imread from a file).
        Mat imgOriginal = signalSleeveImage.first.clone();

        String fileDate = TimeStamp.getLocalDateTimeStamp(signalSleeveImage.second);
        outputFilenamePreamble = ImageUtils.createOutputFilePreamble(pImageParameters.image_source, workingDirectory, fileDate);
        imageROI = ImageUtils.preProcessImage(pImageProvider, imgOriginal, outputFilenamePreamble, pImageParameters);

        RobotLogCommon.d(TAG, "Recognition path " + pSignalSleeveRecognitionPath);
        SignalSleeveReturn retVal;
        switch (pSignalSleeveRecognitionPath) {
            case COLOR -> retVal = colorSleeve(pSignalSleeveParameters.colorSleeveParameters);
            case GRAYSCALE_SLASH -> {
                VisionParameters.GrayParameters grayParameters;
                int colorChannel; // B = 0; G = 1; R = 2
                if (alliance == RobotConstants.Alliance.RED) {
                    grayParameters = pSignalSleeveParameters.redGrayscaleParameters.grayParameters;
                    colorChannel = 2;
                } else if (alliance == RobotConstants.Alliance.BLUE) {
                    grayParameters = pSignalSleeveParameters.blueGrayscaleParameters.grayParameters;
                    colorChannel = 0;
                } else {
                    RobotLogCommon.d(TAG, "GRAYSCALE_SLASH requires RED or BLUE alliance");
                    return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
                }

                retVal = grayscaleSlash(grayParameters, colorChannel);
            }
            default -> throw new AutonomousRobotException(TAG, "Unsupported recognition path " + pSignalSleeveRecognitionPath);
        }

        return retVal;
    }

    // Recognition of the signal sleeve by an angled rectangle.
    private SignalSleeveReturn grayscaleSlash(VisionParameters.GrayParameters pGrayParameters, int pColorChannel) {

        // Split the original image ROI into its BGR channels and use the
        // the color channel (lighter here than in a pure grayscale image)
        // to get better contrast with the black railing.
        ArrayList<Mat> originalImageChannels = new ArrayList<>(3);
        Core.split(imageROI, originalImageChannels); // red or blue channel. B = 0, G = 1, R = 2

        // Write out the selected channel as grayscale.
        //if (RobotLogCommon.isLoggable("v")) {
        switch (pColorChannel) {
            case 0: {
                Imgcodecs.imwrite(outputFilenamePreamble + "_BLUE_CHANNEL.png", originalImageChannels.get(0));
                RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_BLUE_CHANNEL.png");
                break;
            }
            case 2: {
                Imgcodecs.imwrite(outputFilenamePreamble + "_RED_CHANNEL.png", originalImageChannels.get(2));
                RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RED_CHANNEL.png");
                break;
            }
            default: {
                RobotLogCommon.d(TAG, "Invalid color channel " + pColorChannel);
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
            }
        }
        // }

        // Use an inverted threshold on the blue channel to create a white image of the black railing.
        Mat thresholded = ImageUtils.performThresholdOnGray(originalImageChannels.get(pColorChannel), outputFilenamePreamble, pGrayParameters.median_target, pGrayParameters.threshold_low);

        // Identify the contours.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (contours.size() == 0) {
            RobotLogCommon.d(TAG, "No contours found");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Within the ROI draw all of the contours.
        Mat contoursDrawn = imageROI.clone();
        drawShapeContours(contours, contoursDrawn);
        Imgcodecs.imwrite(outputFilenamePreamble + "_CON.png", contoursDrawn);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_CON.png");

        //**TODO Find the contour whose center is closest to that of the ROI.
        Point roiCentroid = new Point(imageROI.width() / 2, imageROI.height() / 2);
        double closestDistance = imageROI.width(); // start with high value
        int indexToClosest = -1;
        for (int i = 0; i < contours.size(); i++) {
            Point contourCentroid = ImageUtils.getContourCentroid(contours.get(i));
            double dist = Math.pow(Math.pow((roiCentroid.x - contourCentroid.x), 2) + Math.pow((roiCentroid.y - contourCentroid.y), 2), 0.5);
            if (dist < closestDistance) {
                closestDistance = dist;
                indexToClosest = i;
            }
        }

        // Sanity check.
        if (indexToClosest == -1) {
            RobotLogCommon.d(TAG, "Failed sanity check on contour closest to center of ROI");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL); // don't crash
        }

        // Fit a rotated rectangle around the contour closest to the center.
        // See https://stackoverflow.com/questions/25837934/matofpoint-to-matofpoint2f-size-opencv-java
        MatOfPoint2f temp = new MatOfPoint2f();
        temp.fromList(contours.get(indexToClosest).toList());
        RotatedRect rotatedRect = Imgproc.minAreaRect(temp);
        Point[] rect_points = new Point[4];
        rotatedRect.points(rect_points);

        // Draw the rotated rectangle.
        Mat drawnRotatedRectangle = imageROI.clone();
        List<MatOfPoint> rrContours = new ArrayList<>();
        rrContours.add(new MatOfPoint(rect_points));
        Imgproc.drawContours(drawnRotatedRectangle, rrContours, 0, new Scalar(0, 255, 0), -1);

        Imgcodecs.imwrite(outputFilenamePreamble + "_RRECT.png", drawnRotatedRectangle);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_RRECT.png");

        // Log the 4 corners of the RotatedRect.
        RobotLogCommon.d(TAG, "Rotated rectangle points: 0 " + rect_points[0] +
                ", 1 " + rect_points[1] + ", 2 " + rect_points[2] + ", 3 " + rect_points[3]);

        double rrAngle = rotatedRect.angle;
        RobotLogCommon.d(TAG, "Rotated rectangle: width " + rotatedRect.size.width + ", height " + rotatedRect.size.height);
        RobotLogCommon.d(TAG, "Rotated rectangle: angle " + rrAngle);

        // See https://theailearner.com/tag/cv2-minarearect/
        // "The angle always lies between [-90,0] because if the object is rotated more
        // than 90 degrees, then the next edge is used to calculate the angle from the
        // horizontal."
        if (rect_points[0].x > rect_points[1].x && rotatedRect.size.height < rotatedRect.size.width) {
            RobotLogCommon.d(TAG, "The slash is angled towards the top right");
            if (rrAngle <= -85.0) {
                RobotLogCommon.d(TAG, "The slash is within 5 degrees of upright");
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
            } else {
                if (rrAngle <= -45.0 && rrAngle >= -55.0)
                    return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_3);

                RobotLogCommon.d(TAG, "The angle of the slash is out of range on the right");
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL);
            }
        } else if (rect_points[0].x > rect_points[1].x && rotatedRect.size.height > rotatedRect.size.width) {
            RobotLogCommon.d(TAG, "The slash is angled towards the top left");
            if (rrAngle >= -5.0) {
                RobotLogCommon.d(TAG, "The slash is within 5 degrees of upright");
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
            } else {
                if (rrAngle <= -45.0 && rrAngle >= -55.0)
                    return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_1);

                RobotLogCommon.d(TAG, "The angle of the slash is out of range on the left");
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL);
            }
        } else // exactly upright?
            if (rect_points[0].x == rect_points[1].x && rotatedRect.size.height > rotatedRect.size.width) {
                RobotLogCommon.d(TAG, "The slash is exactly upright");
                return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
            }

        RobotLogCommon.d(TAG, "I have no idea what's going on");
        return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_UNSUCCESSFUL);
    }

    private SignalSleeveReturn colorSleeve(SignalSleeveParameters.ColorSleeveParameters pColorSleeveParameters) {
        Mat thresholded = ImageUtils.applyInRange(imageROI, outputFilenamePreamble, pColorSleeveParameters.hsvParameters);

        // Clean up the thresholded image via morphological opening.
        Mat morphed = new Mat();
        Imgproc.erode(thresholded, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));
        Imgproc.dilate(morphed, morphed, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)));

        return getLocation(morphed,
                pColorSleeveParameters.minWhitePixelsLocation2,
                pColorSleeveParameters.minWhitePixelsLocation3);
    }

    //## 10/22/2022 Failed experiment. The chroma green tape is not distinct enough
    // from the gray tiles. However, the technique of splitting the channels should
    // prove useful in recognition of the red and blue cone stacks.
    /*
    private SignalSleeveReturn splitGreenRecognitionPath(SignalSleeveParameters.GrayscaleParameters pSplitGreenParameters) {
        // Remove distractions before we convert to grayscale: set the red and blue
        // channels to black and the green channel to white.
        ArrayList<Mat> channels = new ArrayList<>(3);

        Core.split(imageROI, channels);

        // Write out the green channel only as grayscale.
        Imgcodecs.imwrite(outputFilenamePreamble + "_GREEN_GRAY.png", channels.get(1));

        // B = 0, G = 1, R = 2.
        Mat blackChannel = Mat.zeros(channels.get(0).size(), CvType.CV_8UC1);
        Mat whiteChannel = Mat.ones(channels.get(0).size(), CvType.CV_8UC1);
        // Replace the blue channel with the black channel.
        //channels.set(0, blackChannel);
        // Replace the green channel with the white channel.
        //channels.set(1, whiteChannel);
        // Replace the red channel with the black channel.
        channels.set(2, blackChannel);

        // Put the BGR image back together.
        Core.merge(channels, imageROI);

        // Here's how to create an image with only the green channel.
        //Mat green = new Mat(480, 640, CvType.CV_8UC3, new Scalar(0,255,0));

        Imgcodecs.imwrite(outputFilenamePreamble + "_GREEN.png", imageROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GREEN.png");

        // We're on the grayscale path.
        Mat grayROI = new Mat();
        Imgproc.cvtColor(imageROI, grayROI, Imgproc.COLOR_BGR2GRAY);

        Imgcodecs.imwrite(outputFilenamePreamble + "_GRAY.png", grayROI);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_GRAY.png");

        Mat adjustedGray = imageUtils.adjustGrayscaleBrightness(grayROI, pSplitGreenParameters.grayParameters.median_target);
        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ.png", adjustedGray);
        RobotLogCommon.d(TAG, "Writing adjusted grayscale image " + outputFilenamePreamble + "_ADJ.png");

        int grayThresholdLow = pSplitGreenParameters.grayParameters.threshold_low;
        RobotLogCommon.d(TAG, "Threshold value: low " + grayThresholdLow);

        //!! Wrong - Use ImageUtils.performThreshold()

        // Threshold the image: set pixels over the threshold value to white.
        Mat thresholded = new Mat(); // output binary image
        Imgproc.threshold(adjustedGray, thresholded,
                grayThresholdLow,    // threshold value
                255,   // white
                Imgproc.THRESH_BINARY); // thresholding type

        Imgcodecs.imwrite(outputFilenamePreamble + "_ADJ_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_ADJ_THR.png");

        return getLocation(thresholded,
                pSplitGreenParameters.minWhitePixelsLocation2,
                pSplitGreenParameters.minWhitePixelsLocation3);
    }
     */

    private SignalSleeveReturn getLocation(Mat pThresholded, int pMinWhitePixelsLocation2, int pMinWhitePixelsLocation3) {
        // Our target,unless it's location 1, which is black, will now appear
        // white in the thresholded image.
        int nonZeroPixelCount = Core.countNonZero(pThresholded);
        RobotLogCommon.d(TAG, "Number of non-zero pixels " + nonZeroPixelCount);

        // Check the minimum non-zero pixel count for the sleeve with the greatest
        // number of white pixels, location 3.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 3 " + pMinWhitePixelsLocation3);
        if (nonZeroPixelCount > pMinWhitePixelsLocation3) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 3.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_3);
        }

        // Try location 2.
        RobotLogCommon.d(TAG, "Minimum non-zero-pixel count for location 2 " + pMinWhitePixelsLocation2);
        if (nonZeroPixelCount > pMinWhitePixelsLocation2) {
            RobotLogCommon.d(TAG, "The signal sleeve indicates location 2.");
            return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_2);
        }

        // Default: must be location 1.
        return new SignalSleeveReturn(RobotConstants.OpenCVResults.RECOGNITION_SUCCESSFUL, RobotConstantsPowerPlay.SignalSleeveLocation.LOCATION_1);
    }

    // The parameters pContours is the output of a call to findContours.
    private void drawShapeContours(List<MatOfPoint> pContours, Mat pImageOut) {
        RobotLogCommon.d(TAG, "drawContours: number of contours " + pContours.size());
        Scalar color = new Scalar(0, 255, 0); // BGR green - good against dark background

        for (int i = 0; i < pContours.size(); i++) {
            Imgproc.drawContours(pImageOut, pContours, i, color, 2);
        }
    }

    // Thickness of < 0 means fill with color.
    private void drawOneRectangle(Rect pRect, Mat pImageOut, int pThickness) {
        Imgproc.rectangle(pImageOut, pRect, new Scalar(0, 255, 0), pThickness); // GREEN
    }
}