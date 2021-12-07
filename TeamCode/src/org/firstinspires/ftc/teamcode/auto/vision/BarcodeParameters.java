package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.opencv.core.Rect;

import java.util.EnumMap;

// Input parameters to barcode recognition.
public class BarcodeParameters {

    public final VisionParameters.GrayParameters grayParameters; // extracted from BarcodeParameters.xml.
    private EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> barcodeElements;
 
    public BarcodeParameters(VisionParameters.GrayParameters pGrayParameters) {
        grayParameters = pGrayParameters;
    }

    public void setBarcodeElements(EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> pBarcodeElements) {
        barcodeElements = pBarcodeElements;
    }

    public EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> getBarcodeElements() {
        return barcodeElements;
    }
    
}