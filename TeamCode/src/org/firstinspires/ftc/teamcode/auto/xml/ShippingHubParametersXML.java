package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.auto.vision.ShippingHubParameters;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize a Shipping Hub in Autonomous.
public class ShippingHubParametersXML {
    public static final String TAG = ShippingHubParametersXML.class.getSimpleName();
    private static final String SH_FILE_NAME = "ShippingHubParameters.xml";

    private final Document document;
    private final XPath xpath;

    public ShippingHubParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + SH_FILE_NAME));
            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    public ShippingHubParameters getShippingHubParameters() throws XPathExpressionException {
        XPathExpression expr;
        
        VisionParameters.HSVParameters blueAllianceHSVParameters;
        VisionParameters.HSVParameters redAllianceHSVParameters; 
        ShippingHubParameters.DistanceParameters distanceParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML shipping_hub_parameters");

        expr = xpath.compile("//shipping_hub_parameters");
        Node shipping_hub_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (shipping_hub_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//shipping_hub_parameters' not found");

        // Point to <alliance_parameters><BLUE><hsv_parameters>
        expr = xpath.compile("//shipping_hub_parameters/alliance_parameters/BLUE/hsv_parameters");
        Node blue_hsv_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (blue_hsv_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//shipping_hub_parameters/alliance_parameters/BLUE/hsv_parameters' not found");

        blueAllianceHSVParameters = ImageXML.parseHSVParameters(blue_hsv_parameters_node);

        // Point to <alliance_parameters><RED><hsv_parameters>
        expr = xpath.compile("//shipping_hub_parameters/alliance_parameters/RED/hsv_parameters");
        Node red_hsv_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (red_hsv_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//shipping_hub_parameters/alliance_parameters/RED/hsv_parameters' not found");

        redAllianceHSVParameters = ImageXML.parseHSVParameters(red_hsv_parameters_node);

        // Point to <distance_parameters>
        expr = xpath.compile("//shipping_hub_parameters/distance_parameters");
        Node distance_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (distance_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//shipping_hub_parameters/distance_parameters' not found");

        distanceParameters = parseDistanceParameters(distance_parameters_node);

        return new ShippingHubParameters(blueAllianceHSVParameters, redAllianceHSVParameters, distanceParameters);
    }

    // Parse the parameters that the "similar triangles" method needs to
    // calculate the distance from the robot to the shipping hub.
    /*
    <distance_parameters>
      <calibration_object_distance>22.0</calibration_object_distance>
      <calibration_object_width>15.0</calibration_object_width>
      <focal_length>601.33</focal_length>
      <camera_to_robot_center>2.5</camera_to_robot_center>
    </distance_parameters>
    */
    private ShippingHubParameters.DistanceParameters parseDistanceParameters(Node pDistanceParametersNode) {
        double calibrationDistance;
        double calibrationWidth;
        double focalLength;
        double cameraToRobotCenter;

        // Parse the <calibration_object_distance> element.
        Node calibration_distance_node = pDistanceParametersNode.getFirstChild();
        calibration_distance_node = getNextElement(calibration_distance_node);
        if ((calibration_distance_node == null) || !calibration_distance_node.getNodeName().equals("calibration_object_distance") || calibration_distance_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_parameters/calibration_object_distance' missing or empty");

        try {
            calibrationDistance = Double.parseDouble(calibration_distance_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_parameters/calibration_distance'");
        }

        // Parse the <calibration_object_width> element.
        Node calibration_object_width_node = calibration_distance_node.getNextSibling();
        calibration_object_width_node = getNextElement(calibration_object_width_node);
        if ((calibration_object_width_node == null) || !calibration_object_width_node.getNodeName().equals("calibration_object_width") || calibration_object_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_parameters/calibration_object_width' missing or empty");

        try {
            calibrationWidth = Double.parseDouble(calibration_object_width_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_parameters/calibration_object_width'");
        }
    
        // Parse the <focal_length> element.
        Node focal_length_node = calibration_object_width_node.getNextSibling();
        focal_length_node = getNextElement(focal_length_node);
        if ((focal_length_node == null) || !focal_length_node.getNodeName().equals("focal_length") || focal_length_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_parameters/focal_length' missing or empty");

        try {
            focalLength = Double.parseDouble(focal_length_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_parameters/focal_length'");
        }

        // Parse the <camera_to_robot_center> element.
        Node camera_node = focal_length_node.getNextSibling();
        camera_node = getNextElement(camera_node);
        if ((camera_node == null) || !camera_node.getNodeName().equals("camera_to_robot_center") || camera_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_parameters/camera_to_robot_center' missing or empty");

        try {
            cameraToRobotCenter = Double.parseDouble(camera_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_parameters/camera_to_robot_center'");
        }

        return new ShippingHubParameters.DistanceParameters(calibrationDistance, calibrationWidth, focalLength, cameraToRobotCenter);
    }

    private Node getNextElement(Node pNode) {
        Node nd = pNode;
        while (nd != null) {
            if (nd.getNodeType() == Node.ELEMENT_NODE) {
                return nd;
            }
            nd = nd.getNextSibling();
        }
        return null;
    }
}

