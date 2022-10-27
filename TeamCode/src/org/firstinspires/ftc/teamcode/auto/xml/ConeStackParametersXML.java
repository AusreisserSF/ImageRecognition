package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.ConeStackParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VisionParameters;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.w3c.dom.Document;
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
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        VisionParameters.GrayParameters grayParametersRed = ImageXML.parseGrayParameters(gray_parameters_node);

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
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        VisionParameters.GrayParameters grayParametersBlue = ImageXML.parseGrayParameters(gray_parameters_node);

        //**TODO 10/27/2022 STOPPED HERE

        // Parse <depth_parameters>
        Node depth_parameters_node = blue_node.getNextSibling();
        depth_parameters_node = getNextElement(depth_parameters_node);
        if (depth_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element 'depth_parameters' not found");

        Node window_offset_x_node = depth_parameters_node.getFirstChild();
        if ((window_offset_x_node == null) || !window_offset_x_node.getNodeName().equals("depth_window_offset_x"))
            throw new AutonomousRobotException(TAG, "Element 'depth_window_offset_x' not found");

        // Parse as int
        String windowOffsetXText = window_offset_x_node.getTextContent();
        int window_offset_x = Integer.parseInt(windowOffsetXText);

        /*
               try {
            calibrationDistance = Double.parseDouble(calibration_distance_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_parameters/calibration_distance'");
        }
         */

     /*
            <depth_parameters>
        <depth_window_offset_x>20</depth_window_offset_x>
        <depth_window_offset_y>120</depth_window_offset_y>
        <depth_window_width>40</depth_window_width>
        <depth_window_height>80</depth_window_height>
        <depth_filter min="0.5" max="1.00"/>
    </depth_parameters>

    DepthParameters(int pDepthWindowOffsetX, int pDepthWindowOffsetY,
                        int pDepthWindowWidth, int pDepthWindowHeight,
                        float pMinDepth, float pMaxDepth)
      */

        return new ConeStackParameters(grayParametersRed, grayParametersBlue, depthParameters);
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

