package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.ConeStackParameters;
import org.firstinspires.ftc.teamcode.auto.vision.D405Configuration;
import org.firstinspires.ftc.teamcode.auto.vision.DepthParameters;
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

// Parse the XML elements that describe the configuration of the RealSense
// D405 depth camera.
public class D405ConfigurationXML {

    public static final String TAG = D405ConfigurationXML.class.getSimpleName();
    private static final String CONE_STACK_FILE_NAME = "D405Config.xml";

    private final Document document;
    private final XPath xpath;

    public D405ConfigurationXML(String pXMLDir) {
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

    public D405Configuration getD405Configuration() throws XPathExpressionException {
        XPathExpression expr;

        // Point to the first node.
        RobotLogCommon.d(TAG, "Parsing XML cone_stack_parameters");

        expr = xpath.compile("//DEPTH_CAMERA_D405");
        Node d405_node = (Node) expr.evaluate(document, XPathConstants.NODE);
        if (d405_node == null)
            throw new AutonomousRobotException(TAG, "Element '//DEPTH_CAMERA_D405' not found");

        //**TODO if <DEPTH_CAMERA_D405 configured="yes"> = "no"
        // return null;

        // Parse the children of the <DEPTH_CAMERA_D405> element in the XML file.
        Node characteristics_node = d405_node.getFirstChild();
        characteristics_node = getNextElement(characteristics_node);
        if (characteristics_node == null || !characteristics_node.getNodeName().equals("characteristics") ||
                characteristics_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'characteristics' not found or empty");

        // <field_of_view>
        Node fov_node = characteristics_node.getFirstChild();
        fov_node = getNextElement(fov_node);
        if (fov_node == null || !fov_node.getNodeName().equals("field_of_view") ||
                fov_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'field_of_view' not found or empty");

        String fovText = fov_node.getTextContent();
        double fov;
        try {
            fov = Float.parseFloat(fovText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'field_of_view'");
        }

        // <depth_scale>
        Node depth_scale_node = fov_node.getNextSibling();
        depth_scale_node = getNextElement(depth_scale_node);
        if (depth_scale_node == null || !depth_scale_node.getNodeName().equals("depth_scale") ||
                depth_scale_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_scale' not found or empty");

        String depthScaleText = depth_scale_node.getTextContent();
        double depth_scale;
        try {
            depth_scale = Double.parseDouble(depthScaleText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_scale'");
        }

        // Note: if there is only one D405 camera in the configuration
        // it must be identified as <camera1>.
        // <camera_1>
        Node camera1_node = characteristics_node.getNextSibling();
        camera1_node = getNextElement(camera1_node);
        if (camera1_node == null || !camera1_node.getNodeName().equals("camera1") ||
                camera1_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'camera1' not found");

        //**TODO STOPPED HERE 11/6/2022
        //**TODO make a method - the parsing of camera1 and camera2 is the same.
        // <camera2> is optional

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

        return new DepthParameters(window_offset_x, window_offset_y,
                window_height,
                min_distance, max_distance);
    }

    //**TODO THIS belongs in ftcdevcommon for IntelliJ and Android -> XMLUtils
    private static Node getNextElement(Node pNode) {
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
