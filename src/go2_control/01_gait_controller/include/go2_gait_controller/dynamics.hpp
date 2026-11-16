#pragma once
#include <Eigen/Dense>
#include <urdf/model.h>
#include <array>
#include <map>
#include <functional>
#include <string>
#include <vector>
namespace go2_gait {
using Vec=Eigen::Vector3d;using Mat=Eigen::Matrix3d;
struct Dynamics {
  struct Link {int parent=-1,joint=-1;Eigen::Isometry3d origin=Eigen::Isometry3d::Identity();Vec axis=Vec::Zero(),com=Vec::Zero();Mat inertia=Mat::Zero();double mass=0;};
  std::vector<Link> links;std::array<int,4> foot{};double mass=0;
  struct State {std::array<Vec,4> feet;std::array<Mat,4> foot_rotation;Vec com=Vec::Zero();Mat inertia=Mat::Zero();double potential=0;};
  static Eigen::Isometry3d pose(const urdf::Pose &p){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation()=Vec(p.position.x,p.position.y,p.position.z);t.linear()=Eigen::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z).toRotationMatrix();return t;}
  void load(const std::string &xml,const std::array<std::string,12>&names){
    urdf::Model m;if(!m.initString(xml))throw std::runtime_error("Invalid dynamics URDF");links.clear();mass=0;
    std::map<std::string,int> joint_index;for(int i=0;i<12;++i)joint_index[names[i]]=i;
    std::map<std::string,int> index;
    std::function<void(urdf::LinkConstSharedPtr,int)> visit=[&](auto l,int parent){
      Link link;link.parent=parent;
      if(l->parent_joint){auto j=l->parent_joint;link.origin=pose(j->parent_to_joint_origin_transform);link.axis=Vec(j->axis.x,j->axis.y,j->axis.z);auto it=joint_index.find(j->name);if(it!=joint_index.end())link.joint=it->second;}
      if(l->inertial){auto in=l->inertial;link.mass=in->mass;link.com=Vec(in->origin.position.x,in->origin.position.y,in->origin.position.z);Mat I;I<<in->ixx,in->ixy,in->ixz,in->ixy,in->iyy,in->iyz,in->ixz,in->iyz,in->izz;Mat R=pose(in->origin).linear();link.inertia=R*I*R.transpose();mass+=link.mass;}
      int id=links.size();index[l->name]=id;links.push_back(link);for(auto ch:l->child_links)visit(ch,id);
    };visit(m.getRoot(),-1);for(int i=0;i<4;++i)foot[i]=index.at(std::array<std::string,4>{"FL_foot","FR_foot","RL_foot","RR_foot"}[i]);
  }
  State evaluate(const std::array<double,12>&q,const Mat &R=Mat::Identity())const{
    State s;std::vector<Eigen::Isometry3d> T(links.size());std::vector<Vec> centers(links.size());
    for(size_t i=0;i<links.size();++i){const auto &l=links[i];T[i]=l.parent<0?l.origin:T[l.parent]*l.origin;if(l.joint>=0)T[i].linear()=T[i].linear()*Eigen::AngleAxisd(q[l.joint],l.axis).toRotationMatrix();centers[i]=T[i]*l.com;s.com+=l.mass*centers[i];s.potential+=l.mass*9.81*(R*centers[i]).z();}
    s.com/=mass;for(int i=0;i<4;++i){s.feet[i]=T[foot[i]].translation();s.foot_rotation[i]=T[foot[i]].linear();}
    for(size_t i=0;i<links.size();++i){Vec d=centers[i]-s.com;Mat r=T[i].linear();s.inertia+=r*links[i].inertia*r.transpose()+links[i].mass*(d.squaredNorm()*Mat::Identity()-d*d.transpose());}return s;
  }
  std::array<Mat,4> jacobians(const std::array<double,12>&q)const{
    std::array<Mat,4> J;for(int i=0;i<12;++i){auto a=q,b=q;a[i]+=1e-5;b[i]-=1e-5;J[i/3].col(i%3)=(evaluate(a).feet[i/3]-evaluate(b).feet[i/3])/2e-5;}return J;
  }
  std::array<double,12> gravity(const std::array<double,12>&q,const Mat &R)const{
    std::array<double,12> tau;for(int i=0;i<12;++i){auto a=q,b=q;a[i]+=1e-5;b[i]-=1e-5;tau[i]=(evaluate(a,R).potential-evaluate(b,R).potential)/2e-5;}return tau;
  }
};
// Small projected convex QP: body wrench tracking, nonnegative normal force,
// friction pyramid, unloading limits and regularization of force changes.
inline std::array<Vec,4> distribute(const std::array<Vec,4>&arms,const Vec &force,const Vec &moment,const std::array<double,4>&caps,const std::array<Vec,4>&previous){
  Eigen::Matrix<double,6,12>A=Eigen::Matrix<double,6,12>::Zero();Eigen::Matrix<double,6,1>b;b<<force,5*moment;
  Eigen::Matrix<double,12,1>prev;for(int l=0;l<4;++l){A.block<3,3>(0,3*l)=Mat::Identity();auto r=arms[l];Mat cross;cross<<0,-r.z(),r.y(),r.z(),0,-r.x(),-r.y(),r.x(),0;A.block<3,3>(3,3*l)=5*cross;prev.segment<3>(3*l)=previous[l];}
  Eigen::Matrix<double,12,12>H=A.transpose()*A+.03*Eigen::Matrix<double,12,12>::Identity();auto g=(A.transpose()*b+.02*prev).eval();double step=1/H.cwiseAbs().rowwise().sum().maxCoeff();auto x=prev;
  for(int k=0;k<100;++k){x+=step*(g-H*x);for(int l=0;l<4;++l){double z0=x[3*l+2],ax=std::abs(x[3*l]),ay=std::abs(x[3*l+1]);
    double lo=0,hi=caps[l];for(int j=0;j<24;++j){double z=(lo+hi)*.5;double derivative=z-z0-.6*std::max(ax-.6*z,0.)-.6*std::max(ay-.6*z,0.);if(derivative>0)hi=z;else lo=z;}
    x[3*l+2]=(lo+hi)*.5;double max=.6*x[3*l+2];x[3*l]=std::clamp(x[3*l],-max,max);x[3*l+1]=std::clamp(x[3*l+1],-max,max);}}
  std::array<Vec,4> out;for(int l=0;l<4;++l)out[l]=x.segment<3>(3*l);return out;
}
}
