#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <urdf/model.h>
#include <quadruped_base/quadruped_base.h>

namespace go2_gait {
struct RobotModel {
  std::array<std::string,12> names;
  std::array<double,12> lower{}, upper{}, velocity{};
  void load(champ::QuadrupedBase &base, const std::string &xml,
            const std::array<std::string,4> &legs, const std::string &root) {
    urdf::Model model;
    if(!model.initString(xml)) throw std::runtime_error("Invalid robot_description URDF");
    for(size_t l=0;l<4;++l) {
      std::string parent=root;
      const std::array<std::string,4> parts{"hip", "thigh", "calf", "foot"};
      for(size_t j=0;j<4;++j) {
        auto joint=model.getJoint(legs[l]+"_"+parts[j]+"_joint");
        if(!joint || joint->parent_link_name!=parent)
          throw std::runtime_error("Missing or unexpected Go2 joint chain: "+legs[l]+"_"+parts[j]);
        const auto &origin=joint->parent_to_joint_origin_transform;
        double r,p,y; origin.rotation.getRPY(r,p,y);
        if(std::abs(r)+std::abs(p)+std::abs(y)>1e-5)
          throw std::runtime_error("CHAMP Go2 adapter requires unrotated joint origins");
        base.legs[l]->joint_chain[j]->setTranslation(origin.position.x,origin.position.y,origin.position.z);
        if(j<3) {
          if(joint->type!=urdf::Joint::REVOLUTE || !joint->limits)
            throw std::runtime_error("Expected revolute joint with limits");
          const auto &a=joint->axis;
          if((j==0 && (std::abs(a.x-1)+std::abs(a.y)+std::abs(a.z)>1e-5)) ||
             (j>0 && (std::abs(a.x)+std::abs(a.y-1)+std::abs(a.z)>1e-5)))
            throw std::runtime_error("Unexpected Go2 joint axis");
          size_t i=l*3+j; names[i]=joint->name;
          lower[i]=joint->limits->lower;upper[i]=joint->limits->upper;velocity[i]=joint->limits->velocity;
        } else if(joint->type!=urdf::Joint::FIXED) {
          throw std::runtime_error("Expected fixed foot joint");
        }
        parent=joint->child_link_name;
      }
    }
  }
};
}
