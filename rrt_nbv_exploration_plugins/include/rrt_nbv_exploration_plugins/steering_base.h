#ifndef RRT_NBV_EXPLORATION_PLUGIN_STEERING_BASE_H_
#define RRT_NBV_EXPLORATION_PLUGIN_STEERING_BASE_H_
#include "ros/ros.h"
#include "ros/console.h"

namespace steering_base
{
  class Steering_base
  {
    public:
      virtual void initialize(void) = 0;
  };
}
#endif
