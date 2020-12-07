package org.firstinspires.ftc.teamcode.auto.xml;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

import org.firstinspires.ftc.ftcappcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcappcommon.CommonUtils;
import org.firstinspires.ftc.ftcappcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcappcommon.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.vision.CommonParameters;
import org.firstinspires.ftc.teamcode.auto.vision.RingParameters;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

// Class whose job it is to read an XML file that contains all of the information
// needed for our OpenCV methods to recognize 0, 1, or 4 gold rings in autonomous.
public class RingParametersXML {
    public static final String TAG = "RingParametersXML";
    private static final String FILE_NAME = "RingParameters.xml";

    private final DocumentBuilderFactory dbFactory;
    private final Document document;
    private final DocumentBuilder dBuilder;

    private final XPathFactory xpathFactory;
    private final XPath xpath;

    public RingParametersXML() throws ParserConfigurationException, SAXException, IOException {

        try {
            dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);
            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);
            dBuilder = dbFactory.newDocumentBuilder();
            String configFilename = WorkingDirectory.getWorkingDirectory() + File.separator + FILE_NAME;
            document = dBuilder.parse(new File(configFilename));
            xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    public RingParameters getRingParameters() throws XPathExpressionException {
        XMLCommon xmlCommon = new XMLCommon();
        XPathExpression expr;
        CommonParameters.ImageParameters imageParameters;
        CommonParameters.HSVParameters hsvParameters;
        double minimum_pixel_count_1_ring;
        double minimum_pixel_count_4_rings;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML ring_parameters");

        expr = xpath.compile("//ring_parameters");
        Node ring_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (ring_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//ring_parameters' not found");

        // Point to <image_parameters>
        Node image_node = ring_parameters_node.getFirstChild();
        Node image_parameters_node = getNextElement(image_node);
        if ((image_parameters_node == null) || !image_parameters_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'image_parameters' not found");

        imageParameters = xmlCommon.parseImageParameters(image_parameters_node);

        // Point to <hsv_parameters>
        expr = xpath.compile("//ring_parameters/hsv_parameters");
        Node hsv_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (hsv_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//ring_parameters/hsv_parameters' not found");

        hsvParameters = xmlCommon.parseHSVParameters(hsv_parameters_node);

        // Point to <size_parameters>
        expr = xpath.compile("//ring_parameters/size_parameters");
        Node size_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (size_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//ring_parameters/size_parameters' not found");

        // Process the two children of the <size_parameters> element
        Node min1_node = size_parameters_node.getFirstChild();
        min1_node = getNextElement(min1_node);
        if ((min1_node == null) || !min1_node.getNodeName().equals("minimum_pixel_count_1_ring") || min1_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'minimum_pixel_count_1_ring' missing or empty");

        try {
            minimum_pixel_count_1_ring = Integer.parseInt(min1_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'minimum_pixel_count_1_ring'");
        }

        Node min4_node = min1_node.getNextSibling();
        min4_node = getNextElement(min4_node);
        if ((min4_node == null) || !min4_node.getNodeName().equals("minimum_pixel_count_4_rings") || min4_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'minimum_pixel_count_4_rings' missing or empty");

        try {
            minimum_pixel_count_4_rings = Integer.parseInt(min4_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'minimum_pixel_count_4_rings'");
        }

        return new RingParameters(imageParameters, hsvParameters, minimum_pixel_count_1_ring, minimum_pixel_count_4_rings);
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

