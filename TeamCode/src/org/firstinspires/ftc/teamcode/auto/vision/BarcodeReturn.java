package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

// Holds the results of image recognition.
public class BarcodeReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstantsFreightFrenzy.BarcodePosition barcodePosition;

    public BarcodeReturn(boolean pFatalComputerVisionError, RobotConstantsFreightFrenzy.BarcodePosition pBarcodePosition) {
        fatalComputerVisionError = pFatalComputerVisionError;
        barcodePosition = pBarcodePosition;
    }

}