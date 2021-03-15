package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.CommonParameters;
import org.firstinspires.ftc.teamcode.auto.vision.TowerParameters;
import org.firstinspires.ftc.teamcode.auto.xml.ImageXMLCommon;
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
// needed for our OpenCV methods to a Vumark below a tower in autonomous.
public class TowerParametersXML {
    public static final String TAG = "TowerParametersXML";
    private static final String RP_FILE_NAME = "TowerParameters.xml";

    private final Document document;
    private final XPath xpath;

    public TowerParametersXML(String pXMLDir) {
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema)
            // dbFactory.setIgnoringElementContentWhitespace(true);
            // Not supported in Android Studio dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            document = dBuilder.parse(new File(pXMLDir + RP_FILE_NAME));
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

    public TowerParameters getTowerParameters() throws XPathExpressionException {
        ImageXMLCommon imageXmlCommon = new ImageXMLCommon();
        XPathExpression expr;
        CommonParameters.ImageParameters imageParameters;
        CommonParameters.GrayParameters grayParameters;
        double minArea;
        double maxArea;
        double angleOffset;
     
        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML tower_parameters");

        expr = xpath.compile("//tower_parameters");
        Node ring_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (ring_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//tower_parameters' not found");

        // Point to <image_parameters>
        Node image_node = ring_parameters_node.getFirstChild();
        Node image_parameters_node = getNextElement(image_node);
        if ((image_parameters_node == null) || !image_parameters_node.getNodeName().equals("image_parameters"))
            throw new AutonomousRobotException(TAG, "Element 'image_parameters' not found");

        imageParameters = imageXmlCommon.parseImageParameters(image_parameters_node);

        // Point to <gray_parameters>
        expr = xpath.compile("//tower_parameters/gray_parameters");
        Node gray_parameters_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (gray_parameters_node == null)
            throw new AutonomousRobotException(TAG, "Element '//tower_parameters/hsv_parameters' not found");

        grayParameters = imageXmlCommon.parseGrayParameters(gray_parameters_node);

        // Point to <validation_and_adjustment>
        expr = xpath.compile("//tower_parameters/validation_and_adjustment");
        Node val_adj_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (val_adj_node == null)
            throw new AutonomousRobotException(TAG, "Element '//tower_parameters/validation_and_adjustment' not found");

        // Process the three children of the <validation_and_adjustment> element
        Node min_area_node = val_adj_node.getFirstChild();
        min_area_node = getNextElement(min_area_node);
        if ((min_area_node == null) || !min_area_node.getNodeName().equals("minimum_area") || min_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'minimum_area' missing or empty");

        try {
            minArea = Double.parseDouble(min_area_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'minimum_area'");
        }

        Node max_area_node = min_area_node.getNextSibling();
        max_area_node = getNextElement(max_area_node);
        if ((max_area_node == null) || !max_area_node.getNodeName().equals("maximum_area") || max_area_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'maximum_area' missing or empty");

        try {
            maxArea = Double.parseDouble(max_area_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'maximum_area'");
        }

        Node angle_offset_node = max_area_node.getNextSibling();
        angle_offset_node = getNextElement(angle_offset_node);
        if ((angle_offset_node == null) || !angle_offset_node.getNodeName().equals("vumark_center_to_frame_center_angle") || angle_offset_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'vumark_center_to_frame_center_angle' missing or empty");

        try {
            angleOffset = Double.parseDouble(angle_offset_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'vumark_center_to_frame_center_angle'");
        }

        return new TowerParameters(imageParameters, grayParameters, minArea, maxArea, angleOffset);
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

