package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.ConeStackParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.w3c.dom.Document;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.*;
import java.io.File;
import java.io.IOException;

// Class whose job it is to read an XML file that contains all of the
// information needed for our OpenCV methods to recognize a stack
// of cones during Autonomous.
public class ConeStackParametersXML {
    public static final String TAG = ConeStackParametersXML.class.getSimpleName();
    private static final String CONE_STACK_FILE_NAME = "ConeStackParameters.xml";

    private final Document document;
    private final XPath xpath;

    public ConeStackParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + CONE_STACK_FILE_NAME));
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

    public ConeStackParameters getConeStackParameters() throws XPathExpressionException {
        XPathExpression expr;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML cone_stack_parameters");

        expr = xpath.compile("//cone_stack_parameters");
        Node cone_stack_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (cone_stack_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//cone_stack_parameters' not found");

        // Point to <RED>
        Node red_node = cone_stack_parameters_node.getFirstChild();
        red_node = getNextElement(red_node);
        if (red_node == null)
            throw new AutonomousRobotException(TAG, "Element 'RED' not found");
        RobotConstants.Alliance shouldBeRed = RobotConstants.Alliance.valueOf(red_node.getNodeName());
        if (shouldBeRed != RobotConstants.Alliance.RED)
            throw new AutonomousRobotException(TAG, "Expected element 'RED");

        // Point to <gray_parameters>
        Node gray_parameters_node = red_node.getFirstChild();
        gray_parameters_node = getNextElement(gray_parameters_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' for red not found");

        VisionParameters.GrayParameters grayParametersRed = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters> for red.
        Node red_hsv_node = gray_parameters_node.getNextSibling();
        red_hsv_node = getNextElement(red_hsv_node);
        if ((red_hsv_node == null) || !red_hsv_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' for blue not found");

        VisionParameters.HSVParameters hsvParametersRed = ImageXML.parseHSVParameters(red_hsv_node);

        // Point to <BLUE>
        Node blue_node = red_node.getNextSibling();
        blue_node = getNextElement(blue_node);
        if (blue_node == null)
            throw new AutonomousRobotException(TAG, "Element 'BLUE' not found");
        RobotConstants.Alliance shouldBeBlue = RobotConstants.Alliance.valueOf(blue_node.getNodeName());
        if (shouldBeBlue != RobotConstants.Alliance.BLUE)
            throw new AutonomousRobotException(TAG, "Expected element 'BLUE");

        // Point to <gray_parameters>
        gray_parameters_node = blue_node.getFirstChild();
        gray_parameters_node = getNextElement(gray_parameters_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' for blue not found");

        VisionParameters.GrayParameters grayParametersBlue = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters> for blue.
        Node blue_hsv_node = gray_parameters_node.getNextSibling();
        blue_hsv_node = getNextElement(blue_hsv_node);
        if ((blue_hsv_node == null) || !blue_hsv_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' for blue not found");

        VisionParameters.HSVParameters hsvParametersBlue = ImageXML.parseHSVParameters(blue_hsv_node);

        // Parse <depth_parameters>
        Node depth_parameters_node = blue_node.getNextSibling();
        depth_parameters_node = getNextElement(depth_parameters_node);
        if (depth_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element 'depth_parameters' not found");

        Node window_offset_x_node = depth_parameters_node.getFirstChild();
        window_offset_x_node = getNextElement(window_offset_x_node);
        if (window_offset_x_node == null || !window_offset_x_node.getNodeName().equals("depth_window_offset_percent_x") ||
                window_offset_x_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_offset_percent_x' not found or empty");

        String windowOffsetXText = window_offset_x_node.getTextContent();
        int window_offset_x;
        try {
            window_offset_x = Integer.parseInt(windowOffsetXText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_window_offset_percent_x'");
        }

        Node window_offset_y_node = window_offset_x_node.getNextSibling();
        window_offset_y_node = getNextElement(window_offset_y_node);
        if (window_offset_y_node == null || !window_offset_y_node.getNodeName().equals("depth_window_offset_percent_y") ||
                window_offset_y_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_offset_percent_y' not found or empty");

        String windowOffsetYText = window_offset_y_node.getTextContent();
        int window_offset_y;
        try {
            window_offset_y = Integer.parseInt(windowOffsetYText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_window_offset_percent_y'");
        }

        Node depth_window_height_node = window_offset_y_node.getNextSibling();
        depth_window_height_node = getNextElement(depth_window_height_node);
        if (depth_window_height_node == null || !depth_window_height_node.getNodeName().equals("depth_window_percent_height") ||
                depth_window_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_percent_height' not found");

        String windowHeightText = depth_window_height_node.getTextContent();
        int window_height;
        try {
            window_height = Integer.parseInt(windowHeightText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_window_percent_height'");
        }

        // <depth_filter min="0.5" max="1.00"/>
        Node depth_filter_node = depth_window_height_node.getNextSibling();
        depth_filter_node = getNextElement(depth_filter_node);
        if ((depth_filter_node == null) || !depth_filter_node.getNodeName().equals("depth_filter"))
            throw new AutonomousRobotException(TAG, "Element 'depth_filter' not found");

        NamedNodeMap depth_filter_node_attributes = depth_filter_node.getAttributes();
        Node min_distance_node = depth_filter_node_attributes.getNamedItem("min");
        if (min_distance_node == null || min_distance_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Attribute 'min' in element depth_filter not found or empty");

        float min_distance;
        try {
            min_distance = Float.parseFloat(min_distance_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in attribute 'min' in element depth_filter");
        }

        Node max_distance_node = depth_filter_node_attributes.getNamedItem("max");
        if (max_distance_node == null || max_distance_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Attribute 'max' in element depth_filter not found or empty");

        float max_distance;
        try {
            max_distance = Float.parseFloat(max_distance_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in attribute 'max' in element depth_filter'");
        }

        ConeStackParameters.DepthParameters depthParameters = new ConeStackParameters.DepthParameters(window_offset_x, window_offset_y,
                window_height,
                min_distance, max_distance);

        return new ConeStackParameters(grayParametersRed, hsvParametersRed,
                grayParametersBlue, hsvParametersBlue,
                depthParameters);
    }

    //**TODO THIS belongs in ftcdevcommon for IntelliJ and Android -> XMLUtils
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

