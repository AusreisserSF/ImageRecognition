// Port of ImageUtils.cpp
package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;

import static org.opencv.imgcodecs.Imgcodecs.IMREAD_COLOR;

public class ImageUtils {

    public static final String TAG = "ImageUtils";

// Loads an image.
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

/*
// Define a swatch, typically from the background of the original image.
void ImageUtils::getSwatch(const Mat& pSrcImage, Mat& pSwatch, const cv::Rect& pSwatchDefinition) {

	if ((pSwatchDefinition.height == 0) && (pSwatchDefinition.width == 0)) {
		LogManager::log("At least one swatch dimension was 0");
		return;
	}

	pSwatch = pSrcImage(pSwatchDefinition);
	LogManager::log("Swatch x " + to_string(pSwatchDefinition.x) + ", y " + to_string(pSwatchDefinition.y) + ", width " + to_string(pSwatchDefinition.width) + ", height " + to_string(pSwatchDefinition.height));
}
*/

/*
// Adjust the brightness of a grayscale image.
void ImageUtils::adjustGrayscaleBrightness(const Mat& pGray, Mat& pAdjustedImage, int pTarget) {
	int medianGray = getSingleChannelMedian(pGray);
	LogManager::log("Original image: grayscale median " + to_string(medianGray));

	int grayMedianTarget = pTarget;
	LogManager::log("Grayscale median target " + to_string(grayMedianTarget));

	// adjustment = target - median;
	int adjustment = grayMedianTarget - medianGray;
	Mat adjustedGray;
	pGray.convertTo(pAdjustedImage, -1, 1, adjustment);
	LogManager::log("Grayscale adjustment " + to_string(adjustment));
}
*/

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

/*
// Get the dominant HSV hue.
uchar ImageUtils::getDominantHSVHue(const Mat& pHSVImageIn, const Mat& pMask) {

*/

/*
// Get the median of any single-channel Mat.
int ImageUtils::getSingleChannelMedian(const Mat& pInputImage) {

	if ((pInputImage.dims != 2) || (!pInputImage.isContinuous()))
		throw autonomous_robot_exception(TAG, "Expected a single-channel Mat");

	//** Keep only oneDimension.assign after we're sure this works.
	//	https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
	vector<uchar> oneDimension;
	if (pInputImage.isContinuous()) {
		// array.assign(pInputImage.datastart, pInputImage.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
		oneDimension.assign(pInputImage.data, pInputImage.data + pInputImage.total());
	}
	else {
		for (int i = 0; i < pInputImage.rows; ++i) {
			oneDimension.insert(oneDimension.end(), pInputImage.ptr<uchar>(i), pInputImage.ptr<uchar>(i) + pInputImage.cols);
		}
	}

	nth_element(oneDimension.begin(), oneDimension.begin() + (oneDimension.size() / 2), oneDimension.end());
	uchar median = oneDimension[oneDimension.size() / 2];
	return median;
}
*/

// Get the median of any single-channel Mat.
public int getSingleChannelMedian(Mat pSingleChannelMat) {
   // Adapted from 2019 Worlds Teamcode VisionOCV.java.
    // Generalized from the value channel of an HSV swatch to any Mat.


	if ((pSingleChannelMat.dims() != 2) || (!pSingleChannelMat.isContinuous()))
		throw new AutonomousRobotException(TAG, "Expected a single-channel Mat");

	byte byteBuff[] = new byte[(int) pSingleChannelMat.total()];
		int intBuff[] = new int[(int) pSingleChannelMat.total()];
		int buffLength = byteBuff.length;
		pSingleChannelMat.get(0, 0, byteBuff);
	
		// !! Since Java does not have an unsigned char data type, the byte values
		// may come out as negative. So we have to use a separate array of ints and
		// copy in the bytes with the lower 8 bytes preserved.
		// https://stackoverflow.com/questions/9581530/converting-from-byte-to-int-in-java
		for (int i = 0; i < buffLength; i++)
			intBuff[i] = Byte.toUnsignedInt(byteBuff[i]); // or byteBuff[i] & 0xFF;

		Arrays.sort(intBuff);
		int median = (intBuff[buffLength / 2] + (intBuff[(buffLength / 2) - 1])) / 2;
		return median;
}

/*
void ImageUtils::getLargestContour(const vector<vector<Point>>& pContours, vector<Point>& pLargestContour) {
	auto largestContourIt = std::max_element(pContours.begin(), pContours.end(),
		[](vector<Point> a, vector<Point> b) {return contourArea(a, false) < contourArea(b, false); });
	pLargestContour = *largestContourIt;
}

Point ImageUtils::getContourCentroid(const vector<Point>& pOneContour) {
	Moments contourMoments = moments(pOneContour);
	Point centroid(static_cast<int>(contourMoments.m10 / contourMoments.m00), static_cast<int>(contourMoments.m01 / contourMoments.m00));
	return centroid;
}
*/

/*
// Ported from 2018-2019 auto/VisionOCV.java
double ImageUtils::getAngleToObject(int pROIWidth, int pObjectCentroidX,
	double pCameraFieldOfView, int pCameraHorizontalResolution, double pMagicCorrectionNumber) {
	if (pObjectCentroidX == CommonConstants::CENTROID_X_NPOS)
		return CommonConstants::ANGLE_TO_OBJECT_NPOS; // shouldn't be here

	// This solution follows https://stackoverflow.com/questions/32524787/how-to-compute-horizontal-angle-of-a-pixel-from-a-computer-vision-camera
	// Note: what we're after is the angle from the camera to the centroid of the
	// object of interest. The distance from the center of the image to the centroid
	// of the object is in pixels so the distance from the camera to the plane of the
	// region of interest, i.e. the adjacent side of the triangle, is also in pixels;
	// this is not of practical value but it allows us to use trigonometry.
	double distanceToROI = (1.0 / tan(((pCameraFieldOfView / 2.0) * 3.14159265) / 180.0)) * (pCameraHorizontalResolution / 2.0);
	distanceToROI += pMagicCorrectionNumber; // manipulate "adjacent" side of the triangle to narrow or widen the angle
	double angleRadians = atan((pObjectCentroidX - 0.5 * pROIWidth) / distanceToROI);

	return -(angleRadians * (180.0 / 3.14159265));
}
*/

// Get the median of a color channel.
private int getColorChannelMedian(Mat pChannel, Mat pMask) {
	// If we're dealing with a non-masked image then we just take the median
	// of all the pixels.
	if (pMask.total() == 0) {
		return getSingleChannelMedian(pChannel);
	}
	else
		throw new AutonomousRobotException(TAG, "getColorChannelMedian with mask is not supported at this time");
}

/*
	// If we're dealing with a non-masked image then we just take the median
	// of all the pixels.
	if (pMask.total() == 0) {
		return getSingleChannelMedian(pChannel);
	}
	else {
		// But if we have a non-empty mask then we only want to take the median of
		// of the masked-in pixels.
		// We need to know how many pixels there are in the masked-in area.
		int pixelsMaskedIn = Core.countNonZero(pMask);
		LogManager::log("Pixels in the masked-in area " + to_string(pixelsMaskedIn));

		vector<uchar> maskedInPixels(pixelsMaskedIn, 0);
		int maskedInPixelIndex = 0;

		// Iterate over the image and the mask simultaneously and only save the
		// HSV value for pixels that are masked-in.
		if ((pChannel.rows != pMask.rows) ||
			(pChannel.cols != pMask.cols)) // sanity check
			throw autonomous_robot_exception(TAG, "Image and mask are not the same size");

		// https://docs.opencv.org/2.4/doc/tutorials/core/how_to_scan_images/how_to_scan_images.html#howtoscanimagesopencv
		// To efficently loop over all the pixels you can get a pointer to the data at the
		// start of each row with .ptr().
		// https://answers.opencv.org/question/40006/how-to-iterate-through-pixels-in-android/
		for (int row = 0; row < pChannel.rows; ++row) {
			const uchar* sRowP = pChannel.ptr<uchar>(row);
			const uchar* maskRowP = pMask.ptr<uchar>(row);
			for (int col = 0; col < pChannel.cols; ++col) {
				if (maskRowP[col] != 0) // != 0 is masked-in
					maskedInPixels[maskedInPixelIndex++] = sRowP[col];
			}
		}
		LogManager::log("Saturation values stored for median " + to_string(maskedInPixelIndex));

		// Get the median of the masked-in values.
		nth_element(maskedInPixels.begin(), maskedInPixels.begin() + (maskedInPixels.size() / 2), maskedInPixels.end());
		return maskedInPixels[maskedInPixels.size() / 2];
    }
*/
}

