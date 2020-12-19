package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.auto.RingRecognitionConstants;

// Holds the results of image recognition.
public class RingReturn {

    public final boolean fatalComputerVisionError;
    public final RingRecognitionConstants.TargetZone targetZone;

    public RingReturn(boolean pFatalComputerVisionError, RingRecognitionConstants.TargetZone pTargetZone) {
        fatalComputerVisionError = pFatalComputerVisionError;
        targetZone = pTargetZone;
    }

}

