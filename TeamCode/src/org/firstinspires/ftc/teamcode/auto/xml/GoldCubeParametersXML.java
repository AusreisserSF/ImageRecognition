package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.GoldCubeParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
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
// needed for our OpenCV methods to recognize a gold cube during Autonomous.
public class GoldCubeParametersXML {
    public static final String TAG = GoldCubeParametersXML.class.getSimpleName();
    private static final String BCP_FILE_NAME = "GoldCubeParameters.xml";

    private final Document document;
    private final XPath xpath;

    public GoldCubeParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + BCP_FILE_NAME));
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

    public GoldCubeParameters getGoldCubeParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.HSVParameters hsvParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML gold_cube_parameters");

        expr = xpath.compile("//gold_cube_parameters");
        Node gold_cube_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (gold_cube_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//gold_cube_parameters' not found");

        // Point to <hsv_parameters>
        Node hsv_node = gold_cube_parameters_node.getFirstChild();
        Node hsv_parameters_node = getNextElement(hsv_node);
        if ((hsv_parameters_node == null) || !hsv_parameters_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' not found");

        hsvParameters = ImageXML.parseHSVParameters(hsv_parameters_node);

        // Point to <depth_camera>
        Node depth_camera_node = hsv_parameters_node.getNextSibling();
        depth_camera_node = getNextElement(depth_camera_node);
        if ((depth_camera_node == null) || !depth_camera_node.getNodeName().equals("depth_camera"))
            throw new AutonomousRobotException(TAG, "Element 'depth_camera' not found");

        // Point to <color_camera_fov>
        Node color_camera_fov_node = depth_camera_node.getFirstChild();
        color_camera_fov_node = getNextElement(color_camera_fov_node);
        if ((color_camera_fov_node == null) || !color_camera_fov_node.getNodeName().equals("color_camera_fov") || color_camera_fov_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_camera/color_camera_fov' missing or empty");

        float colorCameraFOV;
        try {
            colorCameraFOV = Float.parseFloat(color_camera_fov_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_camera/color_camera_fov'");
        }

        // Point to <scale>
        Node scale_node = color_camera_fov_node.getNextSibling();
        scale_node = getNextElement(scale_node);
        if ((scale_node == null) || !scale_node.getNodeName().equals("scale") || scale_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_camera/scale' missing or empty");

        float depthCameraScale;
        try {
            depthCameraScale = Float.parseFloat(scale_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_camera/scale'");
        }

        // Point to <distance_filter>
        Node distance_filter_node = scale_node.getNextSibling();
        distance_filter_node = getNextElement(distance_filter_node);
        if ((distance_filter_node == null) || !distance_filter_node.getNodeName().equals("distance_filter") || distance_filter_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_camera/distance_filter' missing or empty");

        float depthCameraDistanceFilter;
        try {
            depthCameraDistanceFilter = Float.parseFloat(distance_filter_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_camera/distance_filter'");
        }

        // Point to <camera_to_robot_center_meters>
        Node camera_to_robot_node = distance_filter_node.getNextSibling();
        camera_to_robot_node = getNextElement(camera_to_robot_node);
        if ((camera_to_robot_node == null) || !camera_to_robot_node.getNodeName().equals("camera_to_robot_center_meters") || distance_filter_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_camera/camera_to_robot_center_meters' missing or empty");

        float cameraToRobotCenter;
        try {
            cameraToRobotCenter = Float.parseFloat(camera_to_robot_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_camera/camera_to_robot_center_meters'");
        }

        return new GoldCubeParameters(hsvParameters, colorCameraFOV, depthCameraScale, depthCameraDistanceFilter, cameraToRobotCenter);
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

