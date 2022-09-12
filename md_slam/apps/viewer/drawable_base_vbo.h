#pragma once
#include <Eigen/Core>

namespace srrg2_core {
  struct DrawableBaseVBO {
    virtual void init() = 0;
    virtual void update() = 0;
    virtual void draw(const Eigen::Matrix4f& projection,
                      const Eigen::Matrix4f& model_pose,
                      const Eigen::Matrix4f& object_pose,
                      const Eigen::Vector3f& light_direction) = 0;
    virtual ~DrawableBaseVBO() {};
  };

  template <typename T_>
  struct DrawableVBO_: public DrawableBaseVBO {
    using InstanceType = T_;
    DrawableVBO_(InstanceType instance_): _instance(instance_){}
    InstanceType _instance;
  };

}
