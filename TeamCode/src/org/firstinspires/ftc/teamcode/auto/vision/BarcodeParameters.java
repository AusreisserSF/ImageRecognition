package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.common.RobotConstantsFreightFrenzy;
import org.opencv.core.Rect;

import java.util.EnumMap;

// Input parameters to barcode recognition.
public class BarcodeParameters {

    public final VisionParameters.GrayParameters grayParameters; // extracted from BarcodeParameters.xml.
    public final VisionParameters.HSVParameters hsvParameters;
    private EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> barcodeElements;
 
    public BarcodeParameters(VisionParameters.GrayParameters pGrayParameters,
                             VisionParameters.HSVParameters pHSVParameters) {
        grayParameters = pGrayParameters;
        hsvParameters = pHSVParameters;
    }

    public void setBarcodeElements(EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> pBarcodeElements) {
        barcodeElements = pBarcodeElements;
    }

    public EnumMap<RobotConstantsFreightFrenzy.BarcodeElementWindow, Rect> getBarcodeElements() {
        return barcodeElements;
    }
    
}