package org.firstinspires.ftc.teamcode.common;

// Holds the results of image recognition.
public class BarcodeReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstantsFreightFrenzy.BarcodeElementPosition barcodePosition;

    public BarcodeReturn(boolean pFatalComputerVisionError, RobotConstantsFreightFrenzy.BarcodeElementPosition pBarcodePosition) {
        fatalComputerVisionError = pFatalComputerVisionError;
        barcodePosition = pBarcodePosition;
    }

}