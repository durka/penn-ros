#include <string>
#include <vector>
#include <ros/ros.h>
#include <ecl/threads.hpp>
using namespace ecl;

#include <vicon/Names.h>
#include <vicon/Values.h>
#include <vicon/Targets.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <fstream>
using namespace std;

vector<string> names, targets, target_names;
vector<int> target_indices;
string filename;
ofstream output;
Mutex segfault_preventer;

void update_targets()
{
  segfault_preventer.lock();
  target_names.clear();
  target_indices.clear();
  if (output.is_open()) output.close();

  if (filename == "STOP")
  {
    ROS_INFO("Shutting down");
    ros::shutdown();
  }
  if (filename == "PAUSE"){
    ROS_INFO("Stopped recording");
    segfault_preventer.unlock();
    return;
  }

  ifstream test_file(filename.c_str());
  if (test_file.good())
  {
    test_file.close();
    ROS_ERROR("Existing file %s. Not overwriting.", filename.c_str());
  } else {
    output.open(filename.c_str());
  }

  if (output.is_open())
  {
    output << "Timestamp\t";
    for (vector<string>::iterator i = targets.begin(); i != targets.end(); ++i)
    {
      for (vector<string>::iterator j = names.begin(); j != names.end(); ++j)
      {
        if (j->find(*i) != string::npos)
        {
          target_names.push_back(*j);
          target_indices.push_back(distance(names.begin(), j));
          output << *j << "\t";
        }
      }
    }
    output << endl;
  }
  segfault_preventer.unlock();
}

void names_callback(const vicon::Names::ConstPtr& msg)
{
  if (names.size() == 0 || msg->names != names)
  {
    if (names.size() == 0)
    {
      ROS_INFO("Names received!");
    }
    else
    {
      ROS_INFO("Names changed!");
    }
    names = msg->names;
    update_targets();
  }
}

void targets_callback(const vicon::Targets::ConstPtr& msg)
{
  if (targets.size() == 0 || msg->targets != targets || filename != msg->filename)
  {
    if (targets.size() == 0)
    {
      ROS_INFO("Targets received!");
    }
    else if (msg->targets != targets)
    {
      ROS_INFO("Targets changed!");
    }
    else
    {
      ROS_INFO("Filename changed!");
    }
    targets = msg->targets;
    filename = msg->filename;
    update_targets();
  }
}

void values_callback(const vicon::Values::ConstPtr& msg)
{
  segfault_preventer.lock();
  if (output.is_open())
  {
    output << setprecision(9) << fixed << ((double)msg->header.stamp.sec + msg->header.stamp.nsec/1e9f) << "\t";
    for (int i = 0; i < target_names.size(); ++i)
    {
      //ROS_INFO("%s = %g", target_names[i].c_str(), msg->values[target_indices[i]]);
      output << msg->values[target_indices[i]] << "\t";
    }
    output << endl;
  }
  segfault_preventer.unlock();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter");

  ros::NodeHandle n("~");

  ros::Subscriber sub_names
    = n.subscribe("/vicon/names", 100, names_callback);
  ros::Subscriber sub_values
    = n.subscribe("/vicon/values", 100, values_callback);

  ros::Subscriber sub_targets
    = n.subscribe("/vicon/targets", 100, targets_callback);

  ros::spin();
    
  return 0;
}
