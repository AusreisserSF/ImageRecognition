package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

// Holds the results of image recognition.
public class BarcodeReturn {

    public final RobotConstants.OpenCVResults openCVResults;
    public final RobotConstantsFreightFrenzy.BarcodeElementWindow barcodeElementWindow;

    public BarcodeReturn(RobotConstants.OpenCVResults pOpenCVResults, RobotConstantsFreightFrenzy.BarcodeElementWindow pBarcodeElementWindow) {
        openCVResults = pOpenCVResults;
        barcodeElementWindow = pBarcodeElementWindow;
    }

    // Constructor for OpenCV errors such as "no such file".
    public BarcodeReturn(RobotConstants.OpenCVResults pOpenCVResults) {
        openCVResults = pOpenCVResults;
        barcodeElementWindow = RobotConstantsFreightFrenzy.BarcodeElementWindow.WINDOW_NPOS;
    }
}