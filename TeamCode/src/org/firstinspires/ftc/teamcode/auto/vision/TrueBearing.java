package org.firstinspires.ftc.teamcode.auto.vision;

//When the camera is not at the center of rotation of the robot, the bearing to an object must be translated
//to the actual value that the entire robot must rotate to be pointed at the same object that the camera sees

public class TrueBearing {
    //d   Distance from the camera to the shipping hub (inches)
    //b   Bearing from the camera to the shipping hub (degrees)
    //r   Distance from the camera to the center of the robot. (inches)

    public static double computeTrueBearing(double d, double b, double r){

        double sinVal = Math.sin(Math.toRadians(180.0-b));
        double cosVal = Math.cos(Math.toRadians(180.0-b));

        double sinOfTheta = (d* sinVal)/(Math.sqrt(d*d+r*r -2*d*r*cosVal));

        return Math.toDegrees(Math.asin(sinOfTheta));
    }

}
