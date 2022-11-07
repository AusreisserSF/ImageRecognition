package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.DepthParameters;
import org.firstinspires.ftc.teamcode.auto.vision.JunctionParameters;
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
// needed for our OpenCV methods to recognize a PowerPlay junction during Autonomous.
public class JunctionParametersXML {
    public static final String TAG = JunctionParametersXML.class.getSimpleName();
    private static final String JUNCTION_FILE_NAME = "JunctionParameters.xml";

    private final Document document;
    private final XPath xpath;

    public JunctionParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + JUNCTION_FILE_NAME));
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

    public JunctionParameters getJunctionParameters() throws XPathExpressionException {
        XPathExpression expr;
        VisionParameters.GrayParameters grayParameters;
        VisionParameters.HSVParameters hsvParameters;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML junction_parameters");

        expr = xpath.compile("//junction_parameters");
        Node junction_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (junction_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//junction_parameters' not found");

        // Point to <gray_parameters>
        Node gray_node = junction_parameters_node.getFirstChild();
        Node gray_parameters_node = getNextElement(gray_node);
        if ((gray_parameters_node == null) || !gray_parameters_node.getNodeName().equals("gray_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'gray_parameters' not found");

        grayParameters = ImageXML.parseGrayParameters(gray_parameters_node);

        // Point to <hsv_parameters>
        Node hsv_node = gray_node.getNextSibling();
        Node hsv_parameters_node = getNextElement(hsv_node);
        if ((hsv_parameters_node == null) || !hsv_parameters_node.getNodeName().equals("hsv_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'hsv_parameters' not found");

        hsvParameters = ImageXML.parseHSVParameters(hsv_parameters_node);

        // Parse <depth_parameters>
        Node depth_parameters_node = hsv_node.getNextSibling();
        depth_parameters_node = getNextElement(depth_parameters_node);
        if (depth_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element 'depth_parameters' not found");

        DepthParameters depthParameters = DepthParametersXML.parseDepthParameters(depth_parameters_node);

        return new JunctionParameters(grayParameters, hsvParameters, depthParameters);
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

