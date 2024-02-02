#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <morpheus_msgs/ContactMap.h>

// name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Set of names of links included in the robot for collision detection
static const std::set<std::string> A_BOT_LINK_SET
{
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tcp_link",
    "tcp_collision_link",
    "d415_mount_link",
    "camera_link",
    "coupler",
    "cable_protector",
    "gripper_body",
    "left_outer_knuckle",
    "left_outer_finger",
    "left_inner_finger",
    "left_inner_finger_pad",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_outer_finger",
    "right_inner_finger",
    "right_inner_finger_pad",
    "right_inner_knuckle"
};

// Set of names of links included in the obstacles for collision detection
static const std::set<std::string> OBSTACLE_SET
{
    "teapot",
    "cylinder"
};

namespace collision
{

};

class CollisionNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        ros::Publisher g_distance_publisher;
        ros::Publisher g_contacts_publisher;
        ros::Publisher g_nearest_publisher;
        ros::Publisher g_contactmap_publisher;
        collision_detection::CollisionResult g_c_res;
        collision_detection::CollisionRequest g_c_req;

        ros::Publisher* g_marker_array_publisher = nullptr;
        visualization_msgs::MarkerArray g_collision_points;
        // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        CollisionNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::init(argc, argv, "collision");
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();

            // Create a RobotModelLoader to load the robot's URDF and SRDF
            // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

            // Create a PlanningScene object and set the robot model
            // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

            // Create a PlanningSceneMonitor around the PlanningScene
            // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
            //     new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);

            // Set update callback
            // g_planning_scene_monitor->addUpdateCallback(planningSceneMonitorCallback);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene"))
            {
                ROS_INFO("Planning Scene Monitor is active and ready.");
            }
            else
            {
                ROS_ERROR("Failed to set up Planning Scene Monitor.");
            }

            // Request the PlanningScene itself and change collision detection engine to Bullet
            // planning_scene::PlanningScenePtr planning_scene;
            // Get read/write pointer to planning_scene
            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                g_planning_scene_monitor->getPlanningScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), 
                                                    true /* exclusive */);
                
                if (strcmp((g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    ROS_INFO("Planning Scene is active and ready.");
                }    
                else
                {
                    ROS_INFO("Collision detector incorrect");
                    // ROS_INFO(g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName());
                    std::string collision_detector_name = g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                ROS_ERROR("Failed to retrieve PlanningScene.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");

            // Prepare collision result and request objects
            g_c_req.contacts = true;
            g_c_req.distance = true;
            g_c_req.max_contacts = 20;
            g_c_req.max_contacts_per_pair = 1;
            
            /*
            // Edit the allowed collision matrix to focus only on robot-obstacle collisions
            collision_detection::AllowedCollisionMatrix allowed_collision_matrix = 
                g_planning_scene_monitor->getPlanningScene().getAllowedCollisionMatrix();
            allowed_collision_matrix.setEntry(true); // Allow all collisions
            allowed_collision_matrix.setEntry("teapot", false); // Register collisions involving teapot
            */

            // Create collision publishers
            g_distance_publisher = nh.advertise<std_msgs::Float64>("collision/distance", 10);
            g_contacts_publisher = nh.advertise<std_msgs::String>("collision/contacts", 10);
            g_nearest_publisher = nh.advertise<std_msgs::String>("collision/nearest", 10);
            g_contactmap_publisher = nh.advertise<morpheus_msgs::ContactMap>("collision/contactmap", 10);

            
            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher =
                new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100));
            
            // Instantiate visual tools for visualizing markers in Rviz
            // visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "world", "/moveit_visual_tools");

            // Add callback which dictates behavior after each scene update
            // planning_scene_monitor->addUpdateCallback
            
            // Get a list of all links in the robot so we can check them for collisions


            // ros::shutdown();
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                update();
                publish();
                visualize(g_c_res.contacts);
                // Get all contact vectors which correspond to robot<->obstacle pairs
                //for (int i : contact_map)
                //{

                //}
                loop_rate.sleep();
            }

            // Spin the ROS node
            ros::spin();
        }

        void update()
        {
            // Update the planning scene monitor, in case new collision objects have been added
            // g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene");
            // Update the collision result, based on the collision request
            g_c_res.clear();
            g_planning_scene_monitor->getPlanningScene()->checkCollision(g_c_req, g_c_res);

            /*
            ros::Time update_time = g_planning_scene_monitor->getLastUpdateTime(); // Get last update time
            std::stringstream update_time_ss; // Instantiate stringstream for concatenation
            update_time_ss << update_time.sec << "." << update_time.nsec; // Concatenate seconds.nanoseconds
            std::string update_time_str = update_time_ss.str(); // Convert to std::string
            ROS_INFO(update_time_str.c_str()); // Convert to const char*
            */
        }

        void publish()
        {
            // Publish minimum distance
            std_msgs::Float64 distance_msg;
            distance_msg.data = g_c_res.distance;
            g_distance_publisher.publish(distance_msg);

            // Publish contact map as text
            std_msgs::String contacts_msg;
            contacts_msg.data = contactMapToString(g_c_res.contacts);
            g_contacts_publisher.publish(contacts_msg);

            // Publish vector to nearest collision
            std::pair<bool, collision_detection::Contact> pair = getNearestContact(g_c_res.contacts);
            bool success = pair.first;
            collision_detection::Contact nearest_contact = pair.second;
            if (success)
            {
                geometry_msgs::Point nearest_msg = contactToPoint(nearest_contact);
                g_nearest_publisher.publish(nearest_msg);
            }

            // Publish contact map as special msg type
            morpheus_msgs::ContactMap contactmap_msg;
            for (auto const& key_value : g_c_res.contacts)
            {
                auto key = key_value.first;
                auto value = key_value.second[0]; // Only get nearest contact

                morpheus_msgs::StringPair msg_key;
                msg_key.first = key.first;
                msg_key.second = key.second;

                moveit_msgs::ContactInformation msg_value;

                geometry_msgs::Point msg_pos;
                msg_pos.x = value.pos[0];
                msg_pos.y = value.pos[1];
                msg_pos.z = value.pos[2];
                msg_value.position = msg_pos;

                geometry_msgs::Vector3 msg_normal;
                msg_normal.x = value.normal[0];
                msg_normal.y = value.normal[0];
                msg_normal.z = value.normal[0];
                msg_value.normal = msg_normal;

                msg_value.depth = value.depth;
                
                msg_value.contact_body_1 = value.body_name_1;
                msg_value.body_type_1 = value.body_type_1;

                msg_value.contact_body_2 = value.body_name_2;
                msg_value.body_type_2 = value.body_type_2;

                contactmap_msg.keys.push_back(msg_key);
                contactmap_msg.values.push_back(msg_value);
            }
            g_contactmap_publisher.publish(contactmap_msg);

        }

        bool isRobotObstaclePair(std::pair<std::string, std::string> pair)
        {
            if 
            (
                (
                    (A_BOT_LINK_SET.find(pair.first) != A_BOT_LINK_SET.end()) and
                    (OBSTACLE_SET.find(pair.second) != OBSTACLE_SET.end())
                )
                or 
                (
                    (OBSTACLE_SET.find(pair.first) != OBSTACLE_SET.end()) and
                    (A_BOT_LINK_SET.find(pair.second) != A_BOT_LINK_SET.end())
                )
            )
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        std::string contactMapToString(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Convert the key list to a string
            std::stringstream key_value_list_str;
            // Iterate over key-value pairs of contact_map
            for (const auto& key_value : contact_map) 
            {
                // Separate keys from values for readability
                const std::pair<std::string, std::string>& key = key_value.first;
                const std::vector<collision_detection::Contact>& value = key_value.second;

                // Enforce condition
                if (isRobotObstaclePair(key))
                {
                    // First add the keys, each of which is a pair of link names, for links in contact
                    key_value_list_str << "Contact: (" << key.first << ", " << key.second << "), Vector: [";
                    // Optionally add the depth, normal, and position associated of the Contact object, i.e. a distance vector
                    // for (const collision_detection::Contact& contact : value) 
                    // {
                    //     key_value_list_str << "{depth: " << contact.depth << ", normal: " << contact.normal << ", pos: " << contact.pos << "}, ";
                    // }
                    // Just publish the first (smallest) depth so we can see the pairwise nearest distances
                    const collision_detection::Contact& contact = value[0];
                    key_value_list_str << contact.depth;
                    // End entry
                    key_value_list_str << "]" << '\\';
                }
            }

            // Convert result to string type
            std::string result = key_value_list_str.str();

            return result;
        }

        std::pair<bool, collision_detection::Contact> getNearestContact(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Instantiate loop variables
            double nearest_distance = std::numeric_limits<double>::infinity();
            collision_detection::Contact nearest_contact;
            bool assigned = false;

            // Loop over the key-value pairs of contact map
            for (const auto& key_value : contact_map) 
            {
                // Separate keys from values for readability
                const std::pair<std::string, std::string>& key = key_value.first;
                const std::vector<collision_detection::Contact>& value = key_value.second;

                // Enforce condition
                if (isRobotObstaclePair(key))
                {
                    const collision_detection::Contact contact = value[0]; // Get the nearest contact only
                    double distance = contact.depth;
                    if (distance < nearest_distance)
                    {
                        nearest_distance = distance;
                        nearest_contact = contact;
                        assigned = true;
                    }
                }
            }
            nearest_contact = setContactDirection(nearest_contact);

            std::pair<bool, collision_detection::Contact> pair;
            pair.first = assigned;
            pair.second = nearest_contact;
            return pair;
        }

        // If contact points from Obstacle to Robot, flip it
        collision_detection::Contact setContactDirection(collision_detection::Contact contact)
        {
            if ((OBSTACLE_SET.find(contact.body_name_1) != OBSTACLE_SET.end()) and
                (A_BOT_LINK_SET.find(contact.body_name_2) != A_BOT_LINK_SET.end()))
            {
                std::swap(contact.body_name_1, contact.body_name_2);
                std::swap(contact.body_type_1, contact.body_type_2);
                contact.pos = contact.pos + (contact.normal * contact.depth);
                contact.normal = contact.normal * (-1);
            }
            return contact;
        }

        geometry_msgs::Point contactToPoint(collision_detection::Contact contact)
        {
            Eigen::Vector3d vector = contact.normal * contact.depth;
            geometry_msgs::Point point;
            point.x = vector[0];
            point.y = vector[1];
            point.z = vector[2];
            return point;
        }

        void visualize(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Set a color for the visualization markers
            std_msgs::ColorRGBA color;
            color.r = 1.0;
            color.g = 0.0;
            color.b = 0.0;
            color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::MarkerArray markers;
            // The function below works for any contact map, but can only create sphere markers
            /* 
            collision_detection::getCollisionMarkersFromContacts(markers, "world", contact_map, color,
                                                                ros::Duration(),  // remain until deleted
                                                                0.01);            // radius
            */

            // Iterate over key-value pairs of contact_map
            std::map<std::string, unsigned> ns_counts;

            for (const auto& key_value : contact_map)
            {
                // Separate keys from values for readability
                const std::pair<std::string, std::string>& key = key_value.first;
                const std::vector<collision_detection::Contact>& value = key_value.second;
                const collision_detection::Contact orig_contact = value[0]; // Get the nearest contact only

                collision_detection::Contact contact = setContactDirection(orig_contact);
                
                // Enforce condition: Only visualize distances between robot and obstacle
                if (isRobotObstaclePair(key))
                {
                    std::vector<Eigen::Vector3d> vec;
                    Eigen::Vector3d p0 = contact.pos; // p0 is the position reported by the Contact
                    Eigen::Vector3d p1; // p1 is p0 + depth * normal
                    double d = contact.depth; // get depth value for readibility
                    Eigen::Vector3d n = contact.normal; // get normal vector for readability
                    for (int i = 0; i < p0.size(); i++) // p1 is p0 + depth * normal
                    {
                        p1[i] = p0[i] + d * n[i];
                    }
                    vec.push_back(p0);
                    vec.push_back(p1);
                    
                    std::vector<geometry_msgs::Point> points; // Put points in the array type accepted by Marker
                    for (int i = 0; i < vec.size(); i++)
                    {
                        geometry_msgs::Point point;
                        point.x = vec[i][0];
                        point.y = vec[i][1];
                        point.z = vec[i][2];
                        points.push_back(point);
                    }
                    
                    std::string ns_name = contact.body_name_1 + "=" + contact.body_name_2; // String name
                    if (ns_counts.find(ns_name) == ns_counts.end())
                        ns_counts[ns_name] = 0;
                    else
                        ns_counts[ns_name]++;
                    visualization_msgs::Marker mk; // Instantiate marker
                    mk.header.stamp = ros::Time::now(); // Timestamp
                    mk.header.frame_id = "world"; // Reference frame id
                    mk.ns = ns_name; // String name
                    mk.id = ns_counts[ns_name]; // Unique number id
                    mk.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
                    mk.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
                    mk.points = points; // Start and end points of arrow
                    mk.scale.x = 0.01; // Arrow shaft diameter
                    mk.scale.y = 0.02; // Arrow head diameter
                    mk.scale.z = 0.02; // Arrow head length
                    mk.color = color; // Color specified above
                    mk.lifetime = ros::Duration(); // Remain until deleted
                    markers.markers.push_back(mk); // Add to MarkerArray markers
                    
                }
                
            }

            publishMarkers(markers);
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // delete old markers
            if (!g_collision_points.markers.empty())
            {
                for (auto& marker : g_collision_points.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                //g_marker_array_publisher->publish(g_collision_points);
            }

            // move new markers into g_collision_points
            std::swap(g_collision_points.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_collision_points.markers.empty())
                g_marker_array_publisher->publish(g_collision_points);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        static void planningSceneMonitorCallback(const moveit_msgs::PlanningScene::ConstPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            ROS_INFO("Updating...");
        }


};

int main(int argc, char** argv)
{
    CollisionNode collision_node(argc, argv);
    collision_node.spin();
    return 0;
}
