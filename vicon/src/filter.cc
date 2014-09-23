#include <string>
#include <vector>
#include <ros/ros.h>
#include <ecl/threads.hpp>
using namespace ecl;

#include <vicon/Names.h>
#include <vicon/Values.h>
#include <vicon/Targets.h>

#include <iostream>
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

  if (filename == "STOP") ros::shutdown();

  ifstream test_file(filename.c_str());
  if (test_file.good())
  {
    test_file.close();
    ROS_ERROR("Existing file %s. Not overwriting.", filename.c_str());
  } else {
    output.open(filename.c_str());
  }

  for (vector<string>::iterator i = targets.begin(); i != targets.end(); ++i)
  {
    for (vector<string>::iterator j = names.begin(); j != names.end(); ++j)
    {
      if (j->find(*i) != string::npos)
      {
        target_names.push_back(*j);
        target_indices.push_back(distance(names.begin(), j));
        if (output.is_open()) output << *j << "\t";
      }
    }
  }
  if (output.is_open()) output << endl;
  segfault_preventer.unlock();
}

void names_callback(const vicon::Names::ConstPtr& msg)
{
  if (names.size() == 0 || msg->names != names)
  {
    ROS_INFO("Names changed!");
    names = msg->names;
    update_targets();
  }
}

void targets_callback(const vicon::Targets::ConstPtr& msg)
{
  if (targets.size() == 0 || msg->names != targets)
  {
    ROS_INFO("Targets changed!");
    filename = msg->filename;
    targets = msg->names;
    update_targets();
  }
}

void values_callback(const vicon::Values::ConstPtr& msg)
{
  segfault_preventer.lock();
  for (int i = 0; i < target_names.size(); ++i)
  {
    ROS_INFO("%s = %g", target_names[i].c_str(), msg->values[target_indices[i]]);
    if (output.is_open()) output << msg->values[target_indices[i]] << "\t";
  }
  if (output.is_open()) output << endl;
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
