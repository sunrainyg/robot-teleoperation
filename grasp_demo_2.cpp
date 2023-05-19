#include <grasp_demo/grasp_demo.h>
namespace grasp_demo{
    
GraspDemo::GraspDemo()
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("arm_name", arm_name_, "right_arm");
    private_nh.param<std::string>("pose_topic", pose_topic_, arm_name_ + "_driver/pose_action/tool_pose");
    private_nh.param<std::string>("gripper_topic", gripper_topic_, arm_name_ + "_driver/fingers_action/finger_positions/");
    private_nh.param<std::string>("joint_angle_topic", joint_angle_topic_, arm_name_ + "_driver/joints_action/joint_angles/");
    target_sub_ = nh_.subscribe<ar_track_alvar_msgs::AlvarMarkers>(arm_name_ + "_ar_pose_marker",1,
                                                                   boost::bind(&GraspDemo::target_callback,
                                                                   this,_1));
    tool_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(arm_name_ + "_driver/out/tool_pose",1,
                                                              boost::bind(&GraspDemo::toolpose_callback,
                                                              this,_1));
    joint_sub_ = nh_.subscribe<kinova_msgs::JointAngles>(arm_name_ + "_driver/out/joint_angles",1,
                                                        boost::bind(&GraspDemo::joint_callback,
                                                        this,_1));
    poseClient = new actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction>(pose_topic_,true);
    gripperClient = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(gripper_topic_,true);
    jointClient = new actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction>(joint_angle_topic_,true);
    data_z[10] = {0};
    index = 0;
    start_find_ = true;
    find_object_ = false;
    get_joint = false;
    //start pose  
    if(arm_name_ == "right_arm"){
        start_joint.angles.joint1 = 317.601470947;
        start_joint.angles.joint2 = 249.501754761;
        start_joint.angles.joint3 = 64.5121078491;
        start_joint.angles.joint4 = 97.9981536865;
        start_joint.angles.joint5 = 275.88848877;
        start_joint.angles.joint6 = 255.410079956;
    }
    else
    {
        start_joint.angles.joint1 = 224.782333374;
        start_joint.angles.joint2 = 219.680770874;
        start_joint.angles.joint3 = 42.4466819763;
        start_joint.angles.joint4 = 262.089477539;
        start_joint.angles.joint5 = 266.16519165;
        start_joint.angles.joint6 = 105.72882843;
    }
}
//added
void GraspDemo::neuron_delta_R(neuron_Rx_pre, neuron_Ry_pre, neuron_Rz_pre)
{
    elta_Rx = 0.8*round(neuron_Rx - neuron_Rx_pre,10)
    delta_Ry = 0.8*round(neuron_Ry - neuron_Ry_pre,10)
    delta_Rz = 0.8*round(neuron_Rz - neuron_Rz_pre,10)
    neuron_Rx_pre = neuron_Rx
    neuron_Ry_pre = neuron_Ry
    neuron_Rz_pre = neuron_Rz
    delta_all_R = (delta_Rx,delta_Ry,delta_Rz)
}
//added
void GraspDemo::neuron_dekta_L(neuron_Lx_pre, neuron_Ly_pre, neuron_Lz_pre)
{
    delta_Lx = 0.8*round(neuron_Lx - neuron_Lx_pre,10)
    delta_Ly = 0.8*round(neuron_Ly - neuron_Ly_pre,10)
    delta_Lz = 0.8*round(neuron_Lz - neuron_Lz_pre,10)
    neuron_Lx_pre = neuron_Lx
    neuron_Ly_pre = neuron_Ly
    neuron_Lz_pre = neuron_Lz
    delta_all_L = (delta_Lx,delta_Ly,delta_Lz)
}

void GraspDemo::set_arm_angle(kinova_msgs::ArmJointAnglesGoal joint_goal)
{
    // jointClient->cancelAllGoals();
    jointClient->sendGoal(joint_goal);
    jointClient->waitForResult();
    if (jointClient->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arm reached target pose!");
    }
    else
    {
        ROS_INFO("Move fail!");
    }

}

bool GraspDemo::gripper_cmd(std::string cmd)
{
    gripperClient->cancelAllGoals();
    kinova_msgs::SetFingersPositionGoal gripper_goal;
    if (cmd == "open")
    {
        gripper_goal.fingers.finger1 = 0.0;
        gripper_goal.fingers.finger2 = 0.0;
        gripper_goal.fingers.finger3 = 0.0;
    }
    else if (cmd == "close")
    {
        gripper_goal.fingers.finger1 = 6510.0;
        gripper_goal.fingers.finger2 = 6510.0;
        gripper_goal.fingers.finger3 = 6510.0;
    }
    else
    {
        ROS_ERROR("Wrong parameter for gripper_cmd(),please give \"open\" or \"close\".");
        return false;
    }
    gripperClient->sendGoal(gripper_goal);
    gripperClient->waitForResult();
    if (gripperClient->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Gripper is already %s", cmd.c_str());
        return true;
    }
    else
    {
        ROS_INFO("Gripper failed to %s", cmd.c_str());
        return false;
    }
}

void GraspDemo::target_callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& target)
{
    bool valid_data_ = false;
    if(target->markers.size() > 0){
       data_z[index] = target->markers[0].pose.pose.position.z;
       index++;
    }
    if(index == 3){
       if(fabs(data_z[2] - data_z[0]) < 0.05 and fabs(data_z[2] - data_z[1]) < 0.05){
          valid_data_ = true;
       }
       index = 0;
    }
    if(start_find_ and valid_data_){
        object_point[0] = target->markers[0].pose.pose.position.x;
        object_point[1] = target->markers[0].pose.pose.position.y;
        object_point[2] = target->markers[0].pose.pose.position.z;
        find_object_ = true;
        start_find_  = false;
    }
    
}

void GraspDemo::toolpose_callback(const geometry_msgs::PoseStamped::ConstPtr& tool_pose)
{
    current_pose = *tool_pose;
}


void GraspDemo::joint_callback(const kinova_msgs::JointAngles::ConstPtr& joint_angle)
{
    current_angles = *joint_angle;
    if(current_angles.joint6 != 0)
    {
        get_joint = true;
    }
}

void GraspDemo::grasp()
{
    //guanjie yidong
    /*if(get_joint){
        target_joint.angles.joint1 = 283.08770752;
        target_joint.angles.joint2 = 162.211425781;
        target_joint.angles.joint3 = 43.5307617188;
        target_joint.angles.joint4 = 265.169036865;
        target_joint.angles.joint5 = 257.846160889;
        target_joint.angles.joint6 = current_angles.joint6 + 2;
        GraspDemo::set_arm_angle(target_joint);
    }*/
    //moduan  yidong
     target_pose = current_pose;
     tool_goal.pose.pose.position.x = target_pose.pose.position.x;
     tool_goal.pose.pose.position.y = target_pose.pose.position.y;
     tool_goal.pose.pose.position.z -= 0.01;
     tool_goal.pose.pose.orientation.x = target_pose.pose.orientation.x;
     tool_goal.pose.pose.orientation.y = target_pose.pose.orientation.y;
     tool_goal.pose.pose.orientation.z = target_pose.pose.orientation.z;
     tool_goal.pose.pose.orientation.w = target_pose.pose.orientation.w;
     tool_goal.pose.header.frame_id = arm_name_ + "_link_base";
     poseClient->sendGoal(tool_goal);
     poseClient->waitForResult();
     if (poseClient->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
     {
         ROS_INFO("ok!");
     }

     //zhuaqu
     /*if(find_object_)
     {   
         target_pose = current_pose;
         target_pose.pose.position.x -= object_point[0];
         target_pose.pose.position.y += object_point[1];
         target_pose.pose.position.z -= object_point[2];
         try{
             listener.lookupTransform(arm_name_ + "_link_base", "ar_marker_0",  
                                     ros::Time(0), transform);
             tool_goal.pose.header.frame_id = arm_name_ + "_link_base";
             tool_goal.pose.pose.position.x = transform.getOrigin().x();
             tool_goal.pose.pose.position.y = transform.getOrigin().y();
             tool_goal.pose.pose.position.z = transform.getOrigin().z();
             std::cout<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<< transform.getOrigin().z()<<std::endl;
             tool_goal.pose.pose.orientation.x = target_pose.pose.orientation.x;
             tool_goal.pose.pose.orientation.y = target_pose.pose.orientation.y;
             tool_goal.pose.pose.orientation.z = target_pose.pose.orientation.z;
             tool_goal.pose.pose.orientation.w = target_pose.pose.orientation.w;
             poseClient->sendGoal(tool_goal);
             poseClient->waitForResult();
             if (poseClient->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
             {
                 ROS_INFO("got it");
                 ros::Duration(1.0).sleep();
                 GraspDemo::gripper_cmd("close");
                 ros::Duration(1.0).sleep();
                 GraspDemo::set_arm_angle(start_joint);
                 ros::Duration(1.0).sleep();
                 GraspDemo::gripper_cmd("open");
             }
         }
         catch (tf::TransformException ex){
             // ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
         }
         start_find_ = true;
         find_object_ = false;
     }*/
}
//}

