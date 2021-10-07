#include "dofbot_kinemarics.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/LinearMath/Quaternion.h>
#include <math.h>

using namespace KDL;
using namespace std;

Dofbot dofbot = Dofbot();
const float RA2DE = 180.0f / M_PI;
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/caclab/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf";

class DofbotKinematics
{
public:
    std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface *db_ptr_; // pointer to MoveGroupInterface object
    vector<double> current_joint_pose;

    // Create constructor
    DofbotKinematics()
    {
        PLANNING_GROUP = "dofbot";
        db_ptr_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP); // instantiate MoveGroupInterface object
        db_ptr_->allowReplanning(true);
        db_ptr_->setPlanningTime(10);
        db_ptr_->setNumPlanningAttempts(10);
        db_ptr_->setGoalPositionTolerance(0.1);
        db_ptr_->setGoalOrientationTolerance(0.1);
        db_ptr_->setMaxVelocityScalingFactor(1.0);
        db_ptr_->setMaxAccelerationScalingFactor(1.0);
    }

    /* 
            get_fk function gets the forward kinematics of the robot arm, meaning it takes the joint angles
            and returns the roll, pitch, yaw, and xyz coordinates
                parameters: vector<double>angles - a vector of the joint angles 
                returns: vector<double> result - a vector of the xyz coordinates, and the roll, pitch, yaw (in this order)
        */
    vector<double> get_fk(vector<double> angles)
    {
        double joints[]{angles[0], angles[1], angles[2], angles[3], angles[4]};

        vector<double> initjoints;

        vector<double> initpos;

        for (int i = 0; i < 5; ++i)
            initjoints.push_back((joints[i] - 90) * DE2RA);

        dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
        vector<double> result = {initpos.at(0) * 100, initpos.at(1) * 100, initpos.at(2) * 100, initpos.at(3) * RA2DE, initpos.at(4) * RA2DE, initpos.at(5) * RA2DE};

        cout << fixed << "FK kinematics result : " << endl;
        cout << "X coordinates(cm)： " << result.at(0) << "\t"
             << "Y coordinates (cm)： " << result.at(1) << "\t"
             << "Z coordinates (cm)： " << result.at(2) << endl;
        cout << "Roll  (°)： " << result.at(3) << "\t"
             << "Pitch (°)： " << result.at(4) << "\t"
             << "Yaw   (°)： " << result.at(5) << endl;

        return result;
    }

    /* 
            get_ik function gets the inverse kinematics of the robot arm, meaning it takes the 
            roll, pitch, yaw, and xyz coordinates and returns the joint angles 

                parameters: vector<double> coordsxyz - a vector of X, Y, Z, coordinates (in this order)
                            vector<double> rollpitchyaw - a vector of the roll, pitch, and yaw (in this order)
                returns: vector<double> result - a vector of the joint angles 
        */
    vector<double> get_ik(vector<double> coordsxyz, vector<double> rollpitchyaw)
    {
        double Roll = rollpitchyaw[0];
        double Pitch = rollpitchyaw[1];
        double Yaw = rollpitchyaw[2];

        double x = coordsxyz[0];
        double y = coordsxyz[1];
        double z = coordsxyz[2];

        double xyz[]{x, y, z};
        double rpy[]{Roll, Pitch, Yaw};
        vector<double> outjoints;
        vector<double> targetXYZ;
        vector<double> targetRPY;

        for (int k = 0; k < 3; ++k)
            targetXYZ.push_back(xyz[k] / 100);
        for (int l = 0; l < 3; ++l)
            targetRPY.push_back(rpy[l] * DE2RA);

        dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);

        vector<double> result = {outjoints.at(0) * RA2DE + 90, outjoints.at(1) * RA2DE + 90, outjoints.at(2) * RA2DE + 90, outjoints.at(3) * RA2DE + 90, outjoints.at(4) * RA2DE + 90};

        //cout <<fixed<< "IK kinematics result : " << endl;
        // for (int i = 0; i < 5; i++) {
        //     cout << result.at(i) << "\t\t";
        // }
        //cout << "\n" << endl;

        return result;
    }

    vector<double> get_roll_pitch_yaw(vector<double> result)
    {
        vector<double> rpy{result.at(3), result.at(4), result.at(5)};
        return rpy;
    }

    vector<double> get_xyz(vector<double> result)
    {
        vector<double> xyz{result.at(0), result.at(1), result.at(2)};
        return xyz;
    }

    /* 
            get_current_joint_pose function gets the current joint position of Dofbot's arm
                returns: vector<double> current_joint_pose - the current joint pose's angles 
        */
    vector<double> get_current_joint_pose()
    {
        moveit::core::RobotStatePtr current_state = db_ptr_->getCurrentState();
        const robot_state::JointModelGroup *joint_model_group = db_ptr_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        //cout << "Current joint pose angles:" << endl;
        current_state->copyJointGroupPositions(joint_model_group, current_joint_pose);
        for (int i = 0; i < 5; i++)
        {
            current_joint_pose[i] = current_joint_pose[i] * RA2DE + 90; // converting from radians to degrees
            //cout << current_joint_pose[i]  << "\t\t";
        }
        //cout << "" << endl;
        return current_joint_pose;
    }

    /* 
            execute_joint_pose function will move the arm to the given angle positions
                parameters: vector<double> angles - angles of the joints using degrees 
        */
    void execute_joint_pose(vector<double> angles)
    {
        get_current_joint_pose(); // run this to make sure current_joint_pose is populated
        //cout << "Attemping a pose at the following angles:" << endl;

        // set joint pose
        for (int i = 0; i < 5; i++)
        {
            current_joint_pose[i] = (angles[i] - 90) * DE2RA; // convert to radians
            // cout << angles[i] << "\t\t";
        }
        //cout << "" << endl;

        // execute pose
        db_ptr_->setJointValueTarget(current_joint_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        const moveit::planning_interface::MoveItErrorCode &code = db_ptr_->plan(plan);

        if (code == code.SUCCESS)
        {
            ROS_INFO_STREAM("plan success");
            db_ptr_->execute(plan);
        }
        else
        {
            ROS_INFO_STREAM("plan error");
        }
        //db_ptr_->move();

        //cout << "Pose executed" << endl;
    }

    // void execute_pose() {
    //     db_ptr_->setStartStateToCurrentState();
    //     tf::Quaternion quaternion;

    //     const string endEffector = db_ptr_->getEndEffectorLink();

    //     namespace rvt = rviz_visual_tools;
    //     moveit_visual_tools::MoveItVisualTools visual_tools(endEffector);

    //     vector<double>js = db_ptr_->getCurrentJointValues();
    //     const moveit::core::JointModelGroup* joint_model_group =
    //     db_ptr_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //     geometry_msgs::PoseStamped ps = db_ptr_->getCurrentPose(endEffector);

    //     vector<geometry_msgs::Pose> waypoints;
    //     robot_state::RobotState start_state(*db_ptr_->getCurrentState());
    //     geometry_msgs::Pose target_pose=db_ptr_->getCurrentPose().pose;
    //     waypoints.push_back(target_pose);

    //     vector<double> circleCoords;
    //     get_current_joint_pose();
    //     vector<double>fk=get_fk(current_joint_pose);
    //     vector<double>xyz=get_xyz(fk);
    //     double radius = 0.5;
    //     double step = 0.5;
    //     double originX = xyz[0];
    //     double originY = xyz[1];
    //     for (double theta=0;theta < 2*M_PI;theta+=step) {
    //         double addX = radius*cos(theta);
    //         double addY = radius*sin(theta);
    //         cout << addX << endl;
    //         cout << addY << endl;
    //         target_pose.position.x += addX;
    //         target_pose.position.y += addY;
    //         waypoints.push_back(target_pose);
    //     }

    //     db_ptr_->setMaxVelocityScalingFactor(0.1);
    //     moveit_msgs::RobotTrajectory trajectory;
    //     double fraction = db_ptr_->computeCartesianPath(waypoints,
    //                                                 0.01,  // eef_step
    //                                                 0.0,   // jump_threshold
    //                                                 trajectory);

    //     ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
    //     fraction * 100.0);
    //     sleep(15.0);

    //     moveit::planning_interface::MoveGroupInterface::Plan plan;
    //     const moveit::planning_interface::MoveItErrorCode &code = db_ptr_->plan(plan);
    //     plan.trajectory_ = trajectory;

    //     if (code == code.SUCCESS) {
    //         ROS_INFO_STREAM("plan success");
    //         db_ptr_->execute(plan);
    //     } else {
    //         ROS_INFO_STREAM("plan error");
    //     }
    // }
};

/* 
    write_angles function writes the angles into a given text file
        parameters: string filename - the name of the file
                    vector<double> angles - the joint pose's angles 
 */
void write_angles(string filename, vector<double> angles)
{
    ofstream outfile;
    outfile.open(filename, ios_base::app);
    if (!outfile)
    {
        cout << "File could not be opened" << endl;
    }
    else
    {
        cout << "File opened" << endl;
    }
    for (int i = 0; i < 4; i++)
    {
        outfile << to_string(angles[i]) + ", ";
    }
    outfile << to_string(angles[4]) << endl;
    outfile.close();
}

int main(int argc, char **argv)
{

    // Initialize
    ros::init(argc, argv, "dofbot_kinematics");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ros::Rate rate(100);
    spinner.start();

    const string my_file = "src/dofbot_moveit/src/demo.txt";

    // Instantiate DofbotKinematics
    DofbotKinematics dofkin = DofbotKinematics();
    dofkin.get_current_joint_pose();

    vector<double> reset{45, 90, 90, 90, 90};
    write_angles(my_file, reset);
    dofkin.execute_joint_pose(reset);

    // Draw circle

    vector<double> inward{45, 90, 10, 35, 90, 170};
    vector<double> fk_result = dofkin.get_fk(inward);
    vector<double> xyz = dofkin.get_xyz(fk_result);
    xyz[2] += 2;
    vector<double> rpy = dofkin.get_roll_pitch_yaw(fk_result);
    vector<double> move_up = dofkin.get_ik(xyz, rpy);
    write_angles(my_file, move_up);
    dofkin.execute_joint_pose(move_up);

    double radius = 4.8;
    double originX = xyz[0];
    double originY = xyz[1];
    double numCoord = 30;
    float theta = (2 * M_PI) / numCoord;
    for (double j = 1; j <= numCoord + 1; j++)
    {
        float angle = theta * j;
        double x = radius * cos(angle) + originX;
        double y = radius * sin(angle) + originY;
        vector<double> fk_result = dofkin.get_fk(move_up);
        vector<double> xyz = dofkin.get_xyz(fk_result);
        xyz[0] = x;
        xyz[1] = y;
        vector<double> rpy = dofkin.get_roll_pitch_yaw(fk_result);
        vector<double> move_circle = dofkin.get_ik(xyz, rpy);
        write_angles(my_file, move_circle);
        dofkin.execute_joint_pose(move_circle);
    }

    // rake pattern

    cout << "Finished execution" << endl;
    return 0;
}
