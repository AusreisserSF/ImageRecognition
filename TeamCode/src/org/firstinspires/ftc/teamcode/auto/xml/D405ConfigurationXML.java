package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.intellij.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.vision.D405Configuration;
import org.firstinspires.ftc.teamcode.common.RobotConstantsPowerPlay;
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

        // If <DEPTH_CAMERA_D405 configured="no"> then go no further.
        NamedNodeMap d405_node_attributes = d405_node.getAttributes();
        Node configured_node = d405_node_attributes.getNamedItem("configured");
        if (configured_node == null || configured_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Attribute 'configured' not found or empty");

        String configuredText = configured_node.getTextContent();
        if (configuredText.equals("no"))
            return null;

        if (!configuredText.equals("yes"))
            throw new AutonomousRobotException(TAG, "Invalid attribute 'configured");

        // Parse the children of the <DEPTH_CAMERA_D405> element.
        Node characteristics_node = d405_node.getFirstChild();
        characteristics_node = getNextElement(characteristics_node);
        if (characteristics_node == null || !characteristics_node.getNodeName().equals("characteristics"))
            throw new AutonomousRobotException(TAG, "Element 'characteristics' not found");

        // <field_of_view>
        Node fov_node = characteristics_node.getFirstChild();
        fov_node = getNextElement(fov_node);
        if (fov_node == null || !fov_node.getNodeName().equals("field_of_view") ||
                fov_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'field_of_view' not found or empty");

        String fovText = fov_node.getTextContent();
        double field_of_view;
        try {
            field_of_view = Float.parseFloat(fovText);
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
        float depth_scale;
        try {
            depth_scale = Float.parseFloat(depthScaleText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_scale'");
        }

        // Note: if there is only one D405 camera in the configuration
        // it must be identified as <camera_1>.
        // <camera_1>
        Node camera1_node = characteristics_node.getNextSibling();
        camera1_node = getNextElement(camera1_node);
        if (camera1_node == null || !camera1_node.getNodeName().equals("camera_1"))
            throw new AutonomousRobotException(TAG, "Element 'camera_1' not found");

        D405Configuration.D405Camera camera1Data = parseCameraData(camera1_node);

        // <camera_2> optional
        D405Configuration.D405Camera camera2Data = null;
        Node camera2_node = camera1_node.getNextSibling();
        if (camera2_node != null) {
            camera2_node = getNextElement(camera2_node);
            if (camera2_node != null) {
                camera2_node = getNextElement(camera2_node);
                if (camera2_node == null || !camera2_node.getNodeName().equals("camera_2"))
                    throw new AutonomousRobotException(TAG, "Element 'camera_2' not found");

                camera2Data = parseCameraData(camera2_node);
            }
        }

        return new D405Configuration(field_of_view, depth_scale, camera1Data, camera2Data);
    }

    private D405Configuration.D405Camera parseCameraData(Node pCameraNode) {
        // <orientation>
        Node orientation_node = pCameraNode.getFirstChild();
        orientation_node = getNextElement(orientation_node);
        if (orientation_node == null || !orientation_node.getNodeName().equals("orientation") ||
                orientation_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'orientation' not found");

        RobotConstantsPowerPlay.D405CameraId orientation =
                RobotConstantsPowerPlay.D405CameraId.valueOf(orientation_node.getTextContent().toUpperCase());

        // <serial_number>
        Node serial_number_node = orientation_node.getNextSibling();
        serial_number_node = getNextElement(serial_number_node);
        if (serial_number_node == null || !serial_number_node.getNodeName().equals("serial_number") ||
                serial_number_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'serial_number' not found");

        String serial_number = serial_number_node.getTextContent();

        // <distance_to_camera_canter>
        Node camera_center_node = serial_number_node.getNextSibling();
        camera_center_node = getNextElement(camera_center_node);
        if (camera_center_node == null || !camera_center_node.getNodeName().equals("distance_to_camera_center") ||
                camera_center_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'distance_to_camera_center' not found or empty");

        String cameraCenterText = camera_center_node.getTextContent();
        double distance_to_camera_center;
        try {
            distance_to_camera_center = Double.parseDouble(cameraCenterText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'distance_to_camera_center'");
        }

        // <offset_from_camera_center>
        Node camera_offset_node = camera_center_node.getNextSibling();
        camera_offset_node = getNextElement(camera_offset_node);
        if (camera_offset_node == null || !camera_offset_node.getNodeName().equals("offset_from_camera_center") ||
                camera_offset_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'offset_from_camera_center' not found or empty");

        String cameraOffsetText = camera_offset_node.getTextContent();
        double offset_from_camera_center;
        try {
            offset_from_camera_center = Double.parseDouble(cameraOffsetText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'offset_from_camera_center'");
        }

        return new D405Configuration.D405Camera(orientation,
                serial_number, distance_to_camera_center, offset_from_camera_center);
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
