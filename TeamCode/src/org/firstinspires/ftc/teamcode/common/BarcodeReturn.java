package org.firstinspires.ftc.teamcode.common;

// Holds the results of image recognition.
public class BarcodeReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstantsFreightFrenzy.BarcodeElementWithinROI barcodePosition;

    public BarcodeReturn(boolean pFatalComputerVisionError, RobotConstantsFreightFrenzy.BarcodeElementWithinROI pBarcodePosition) {
        fatalComputerVisionError = pFatalComputerVisionError;
        barcodePosition = pBarcodePosition;
    }

}