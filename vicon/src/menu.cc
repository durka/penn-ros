#include <string>
#include <vector>
#include <ros/ros.h>

#include <vicon/Names.h>
#include <vicon/Targets.h>

#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

ros::Publisher pub_targets;
vector<string> names;

void names_callback(const vicon::Names::ConstPtr& msg)
{
  if (msg->names != names)
  {
    ROS_INFO("Names changed!");
    names = msg->names;

    vector<string> categories;
    for (vector<string>::iterator i = names.begin(); i != names.end(); ++i)
    {
      size_t space = i->rfind(' ');
      string title = space == string::npos ? *i : i->substr(0, space);
      if (categories.size() == 0 || find(categories.begin(), categories.end(), title) == categories.end())
      {
        categories.push_back(title);
      }
    }

    vicon::Targets msg;
    printf("CSV filename (or \"STOP\", \"PAUSE\")> ");
    getline(cin, msg.filename);
    if (msg.filename != "STOP" && msg.filename != "PAUSE")
    {
      for (int i = 0; i < categories.size(); ++i)
      {
        printf(" [%d]: %s\n", i, categories[i].c_str());
      }
      printf("numbers, terminated by -1> ");
      int n;
      while (true)
      {
        cin >> n;
        if (n == -1) break;
        if (n < 0 || n >= categories.size())
        {
          ROS_ERROR("%d out of range", n);
        } else {
          ROS_INFO("Adding %d=%s", n, categories[n].c_str());
          msg.targets.push_back(categories[n]);
        }
      }
    }
    ROS_INFO("Publishing %d targets", msg.targets.size());
    pub_targets.publish(msg);
    ros::shutdown();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "menu");

  ros::NodeHandle n("~");

  ros::Subscriber sub_names
    = n.subscribe("/vicon/names", 100, names_callback);
  pub_targets = n.advertise<vicon::Targets>("/vicon/targets", 5);

  ros::spin();
    
  return 0;
}
