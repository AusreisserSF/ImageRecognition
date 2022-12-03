package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.auto.vision.DepthParameters;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;

// Parse the XML elements that describe the parameters for the RealSense
// D405 depth camera.
public class DepthParametersXML {

    public static final String TAG = DepthParametersXML.class.getSimpleName();

    // Parse the children of the <depth_parameters> element in the XML file.
    public static DepthParameters parseDepthParameters(Node pDepthParametersNode) {
        Node window_offset_x_node = pDepthParametersNode.getFirstChild();
        window_offset_x_node = getNextElement(window_offset_x_node);
        if (window_offset_x_node == null || !window_offset_x_node.getNodeName().equals("depth_window_offset_percent_x") ||
                window_offset_x_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_offset_percent_x' not found or empty");

        String windowOffsetXText = window_offset_x_node.getTextContent();
        double window_offset_x;
        try {
            window_offset_x = Double.parseDouble(windowOffsetXText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_window_offset_percent_x'");
        }

        Node window_offset_y_node = window_offset_x_node.getNextSibling();
        window_offset_y_node = getNextElement(window_offset_y_node);
        if (window_offset_y_node == null || !window_offset_y_node.getNodeName().equals("depth_window_offset_percent_y") ||
                window_offset_y_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_offset_percent_y' not found or empty");

        String windowOffsetYText = window_offset_y_node.getTextContent();
        double window_offset_y;
        try {
            window_offset_y = Double.parseDouble(windowOffsetYText);
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'depth_window_offset_percent_y'");
        }

        Node depth_window_height_node = window_offset_y_node.getNextSibling();
        depth_window_height_node = getNextElement(depth_window_height_node);
        if (depth_window_height_node == null || !depth_window_height_node.getNodeName().equals("depth_window_percent_height") ||
                depth_window_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'depth_window_percent_height' not found");

        String windowHeightText = depth_window_height_node.getTextContent();
        double window_height;
        try {
            window_height = Double.parseDouble(windowHeightText);
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
