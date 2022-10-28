package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.intellij.WorkingDirectory;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.opencv.core.Point;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class RealSenseUtils {

    private static final String TAG = RealSenseUtils.class.getSimpleName();

    // Read the depth file that corresponds to the color image file.
    // The file is a collection of bytes; read them into an array
    // and then convert to an array of shorts.
    public static short[] getDepthArrayFromFile(VisionParameters.ImageParameters pImageParameters) throws IOException {
        String filenameWithoutExt = pImageParameters.image_source.substring(0, pImageParameters.image_source.lastIndexOf('.'));
        String depthFilename = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir + filenameWithoutExt + ".depth";
        byte[] depth8UC1 = new byte[pImageParameters.resolution_width * pImageParameters.resolution_height];
        try (InputStream output = new FileInputStream(depthFilename)) {
            try (DataInputStream depthInputStream =
                         new DataInputStream(output)) {
                depthInputStream.read(depth8UC1);
            }
        }

        // Convert an array of bytes to an array of shorts.
        short[] depth16UC1 = new short[depth8UC1.length / 2]; // length is in bytes
        ByteBuffer.wrap(depth8UC1).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(depth16UC1);
        return depth16UC1;
    }

    // Returns the angle and distance from the center of the robot to
    // the target pixel.
    public static Pair<Double, Double> getAngleAndDistanceToPixel(VisionParameters.ImageParameters pImageParameters,
                                                       int pTargetPixelX, int pTargetPixelY, double pScaledPixelDepth) {
        // The coordinates of the target pixel are relative to the ROI.
        // We need to adjust those values to reflect the position of the
        // target pixel in the entire image.
        int targetPixelX = pImageParameters.image_roi.x + pTargetPixelX;

        // Calculate the angle from the camera to the target pixel.
        double angleFromCameraToPixel = ImageUtils.computeAngleToObjectCenter(pImageParameters.resolution_width, targetPixelX, RobotConstants.D405_FOV);
        RobotLogCommon.d(TAG, "Angle from camera to target pixel " + angleFromCameraToPixel);

        // We have the hypotenuse from the camera to the center of the object
        // in meters and we have the angle from the camera to the center
        // of the object. Solve for the other two sides in meters.

        // sin(angleFromCameraToObjectr) = opposite / scaledPixelDepth
        double sinACC = Math.sin(Math.toRadians(angleFromCameraToPixel));
        // The "opposite" side if the triangle is from the centroid of the
        // object to the center of the image.
        double opposite = sinACC * pScaledPixelDepth;

        // cos(angleFromCameraToObject) = adjacent / scaledPixelDepth
        double cosACC = Math.cos(Math.toRadians(angleFromCameraToPixel));
        // The "adjacent" side if the triangle is from the center of the
        // camera to the center of the image.
        double adjacentFromCameraCenter = cosACC * pScaledPixelDepth;

        // Since the center of the robot is behind the camera, add this
        // distance to the adjacent value.
        double adjacentFromRobotCenter = adjacentFromCameraCenter + RobotConstants.D405_CAMERA_TO_ROBOT_CENTER_METERS;

        // We have a new triangle. We need to get the angle from the robot
        // center to the center of the object.
        double ratioARC = opposite / adjacentFromRobotCenter;
        double tanARC = Math.tan(ratioARC);
        double angleFromRobotCenter = Math.toDegrees(tanARC);
        RobotLogCommon.d(TAG, "Angle from robot center to target pixel " + angleFromRobotCenter);

        // Get the distance from the center of the robot to the center of
        // the object. This is the hypotenuse of our new triangle.
        double distanceFromRobotCenter = Math.sqrt(Math.pow(opposite, 2) + Math.pow(adjacentFromRobotCenter, 2));
        RobotLogCommon.d(TAG, "Distance (meters) from robot center to pixel at x " + pTargetPixelX + ", y " + pTargetPixelY + " = " + distanceFromRobotCenter);

        return Pair.create(angleFromRobotCenter, distanceFromRobotCenter);
    }
}
