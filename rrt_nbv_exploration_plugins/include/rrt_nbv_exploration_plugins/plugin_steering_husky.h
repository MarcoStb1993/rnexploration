#ifndef RRT_NBV_EXPLORATION_PLUGIN_STEERING_HUSKY_H_
#define RRT_NBV_EXPLORATION_PLUGIN_STEERING_HUSKY_H_
#include "steering_base.h"

namespace steering_plugins
{
  class Steering_husky : public steering_base::Steering_base
  {
    public:
      Steering_husky()
      {

      }

      void initialize()
      {
          ROS_INFO("Plugin Init");
      }

    private:

  };
}
#endif
