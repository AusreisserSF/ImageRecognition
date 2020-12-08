package org.firstinspires.ftc.teamcode.auto.vision;

//!! Android only
/*
import android.util.Pair;
*/

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.opencv.core.Mat;

import java.util.Date;

public interface ImageProvider {

    public enum ImageFormat {RGB, BGR}

    //**TODO requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    public Pair<Mat, Date> getImage() throws InterruptedException;

    public ImageFormat getImageFormat();
}
