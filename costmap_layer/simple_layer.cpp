#include</home/bastien/robotics/src/robotics2018/costmap_layer/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  update = 0;
  or_sub = nh.subscribe("/boxe", 1000, &SimpleLayer::or_callback, this);
  
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

}

void SimpleLayer::or_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("Pose: (%f, %f)", msg->pose.position.x, msg->pose.position.y);
  mark_x_ = msg->pose.position.x;
  mark_y_ = msg->pose.position.y;
  if(update == 0){
    update = 1;
  }else{
    if(msg->header.frame_id == "stop"){
      update = 0;
    }
  }
}

void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(update == 1){
    if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
      ROS_INFO("Pose: (%f, %f)", mark_x_, mark_y_);
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }
}

} // end namespace