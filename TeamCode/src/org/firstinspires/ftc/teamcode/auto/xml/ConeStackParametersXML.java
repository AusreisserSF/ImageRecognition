package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.ConeStackParameters;
import org.firstinspires.ftc.teamcode.auto.vision.DepthParameters;
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
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' for red not found");

        VisionParameters.GrayParameters grayParametersRed = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters> for red.
        Node red_hsv_node = gray_parameters_node.getNextSibling();
        red_hsv_node = getNextElement(red_hsv_node);
        if ((red_hsv_node == null) || !red_hsv_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' for red not found");

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

        DepthParameters depthParameters = DepthParametersXML.parseDepthParameters(depth_parameters_node);

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

