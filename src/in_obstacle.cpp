#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <anytime_rrt_star/ObstacleAction.h>
#define RED     "\033[31m"

class ObstacleAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<anytime_rrt_star::ObstacleAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  anytime_rrt_star::ObstacleFeedback feedback_;
  anytime_rrt_star::ObstacleResult result_;

public:

  ObstacleAction(std::string name) :
    as_(nh_, name, boost::bind(&ObstacleAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ObstacleAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::ObstacleGoalConstPtr &goal)
  {
    // ros::Rate r(1);

    ROS_INFO("%s: checking if  (%i, %i) is in obstacle space", action_name_.c_str(), goal->x,goal->y);


    try
    {
      result_.in_obstacle = (goal->x)*(goal->x) + (goal->y)*(goal->y) <= (1+goal->r+goal->c)*(1+goal->r+goal->c) || 
      (goal->x-2)*(goal->x-2) + (goal->y-3)*(goal->y-3) <= (1+goal->r+goal->c)*(1+goal->r+goal->c) ||
      (goal->x-2)*(goal->x-2) + (goal->y+3)*(goal->y+3) <= (1+goal->r+goal->c)*(1+goal->r+goal->c) ||
      (goal->x+2)*(goal->x+2) + (goal->y+3)*(goal->y+3) <= (1+goal->r+goal->c)*(1+goal->r+goal->c) ||
      (goal->x>=-4.75-goal->r-goal->c && goal->x<=-3.25+goal->r+goal->c && 
        goal->y>= -0.75-goal->r-goal->c && goal->y<= 0.75+goal->r+goal->c) ||
      (goal->x>=-2.75-goal->r-goal->c && goal->x<=-1.25+goal->r+goal->c && 
        goal->y>= 2.25-goal->r-goal->c && goal->y<= 3.75+goal->r+goal->c)||
      (goal->x>=3.25-goal->r-goal->c && goal->x<=4.75+goal->r+goal->c && 
        goal->y>= -0.75-goal->r-goal->c && goal->y<= 0.75+goal->r+goal->c)  ||
      goal->x<-5-goal->r-goal->c || goal->x>5+goal->r+goal->c||goal->y<-5-goal->r-goal->c||goal->y>5+goal->r+goal->c;
      feedback_.success = true;

      as_.publishFeedback(feedback_);
      as_.setSucceeded(result_);

      if(result_.in_obstacle )
      ROS_INFO("%s: (%i, %i) is in obstacle space", action_name_.c_str(), goal->x,goal->y);
      else ROS_INFO("%s: (%i, %i) is NOT in obstacle space", action_name_.c_str(), goal->x,goal->y);
    }
    catch(const runtime_error& error)
    {
      ROS_INFO(" %s %s: (%i, %i) RUNTIME ERROR \033[0m",RED, action_name_.c_str(), goal->x,goal->y);
      feedback_.success = false;
      result_.in_obstacle = true;
      as_.publishFeedback(feedback_);
      as_.setSucceeded(result_);

    }

  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "in_obstacle");

  FibonacciAction in_obstacle("in_obstacle");
  ros::spin();

  return 0;
}