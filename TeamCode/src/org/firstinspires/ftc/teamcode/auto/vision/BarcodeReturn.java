package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;

// Holds the results of image recognition.
public class BarcodeReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstantsFreightFrenzy.BarcodeElementWindow barcodeElementWindow;

    public BarcodeReturn(boolean pFatalComputerVisionError, RobotConstantsFreightFrenzy.BarcodeElementWindow pBarcodeElementWindow) {
        fatalComputerVisionError = pFatalComputerVisionError;
        barcodeElementWindow = pBarcodeElementWindow;
    }

}