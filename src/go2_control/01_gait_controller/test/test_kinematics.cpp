#include <fstream>
#include <iostream>
#include <sstream>
#include "go2_gait_controller/planner.hpp"
#include "go2_gait_controller/dynamics.hpp"
void check(bool v,const char *m){if(!v)throw std::runtime_error(m);}
int main(int argc,char**argv){try{
 check(argc==2,"URDF argument missing");std::ifstream f(argv[1]);std::stringstream xml;xml<<f.rdbuf();go2_gait::Planner p;champ::GaitConfig cfg;cfg.nominal_height=.33913;cfg.com_x_translation=0;p.base.setGaitConfig(cfg);p.model.load(p.base,xml.str(),{"FL","FR","RL","RR"},"base");go2_gait::Dynamics d;d.load(xml.str(),p.model.names);
 std::array<double,12> q;for(int i=0;i<12;++i)q[i]=std::array<double,3>{0,.65,-1.3}[i%3];auto state=d.evaluate(q);check(std::abs(d.mass-16.087)<.001,"Whole-body mass was not parsed");check(state.inertia.determinant()>0,"Invalid composite inertia");
 auto J=d.jacobians(q);
 for(int l=0;l<4;++l){auto foot=state.feet[l];check(std::abs(foot.z()+.3391301)<1e-5,"Standing FK height mismatch");check(std::abs(J[l](0,1)+.3391301)<1e-5,"Independent thigh Jacobian mismatch");
  for(double lift:{0.,.02,.04}){geometry::Transformation t;t.X()=foot.x();t.Y()=foot.y();t.Z()=foot.z()+lift;champ::Kinematics::transformToHip(t,*p.base.legs[l]);float h=NAN,a=NAN,b=NAN;champ::Kinematics::inverse(h,a,b,*p.base.legs[l],t);check(std::isfinite(h)&&std::isfinite(a)&&std::isfinite(b),"Swing IK invalid");auto target=q;target[3*l]=h;target[3*l+1]=a;target[3*l+2]=b;auto actual=d.evaluate(target).feet[l];check((actual-go2_gait::Vec(foot.x(),foot.y(),foot.z()+lift)).norm()<1e-5,"Full URDF FK/IK mismatch");}
 }
 // Reachability envelope for the 0.55 m/s forward and 0.15 m/s lateral gait.
 for(int l=0;l<4;++l)for(double dx:{-.12,0.,.12})for(double dy:{-.05,0.,.05})for(double lift:{0.,.02,.04}){
  auto foot=state.feet[l]+go2_gait::Vec(dx,dy,lift);geometry::Transformation t;t.X()=foot.x();t.Y()=foot.y();t.Z()=foot.z();champ::Kinematics::transformToHip(t,*p.base.legs[l]);float h=NAN,a=NAN,b=NAN;champ::Kinematics::inverse(h,a,b,*p.base.legs[l],t);auto target=q;std::array<double,3> angles{h,a,b};
  for(int j=0;j<3;++j){int i=3*l+j;check(std::isfinite(angles[j])&&angles[j]>=p.model.lower[i]&&angles[j]<=p.model.upper[i],"High-speed/lateral target exceeds joint limits");target[i]=angles[j];}
  check((d.evaluate(target).feet[l]-foot).norm()<1e-5,"High-speed/lateral FK/IK mismatch");
 }
 std::array<go2_gait::Vec,4> previous,arms;std::array<double,4>caps{100,100,100,100};for(int l=0;l<4;++l){previous[l]=go2_gait::Vec(0,0,d.mass*9.81/4);arms[l]=state.feet[l]-state.com;}
 auto forces=go2_gait::distribute(arms,go2_gait::Vec(0,0,d.mass*9.81),go2_gait::Vec::Zero(),caps,previous);go2_gait::Vec sum=go2_gait::Vec::Zero(),moment=go2_gait::Vec::Zero();for(int l=0;l<4;++l){sum+=forces[l];moment+=arms[l].cross(forces[l]);}check(std::abs(sum.z()-d.mass*9.81)<1,"QP gravity support mismatch");check(moment.norm()<.3,"QP standing moment mismatch");
 caps[0]=0;forces=go2_gait::distribute(arms,go2_gait::Vec(150,-150,d.mass*9.81),go2_gait::Vec(4,4,3),caps,forces);
 for(int l=0;l<4;++l){check(forces[l].z()>=0&&forces[l].z()<=caps[l]+1e-6,"QP normal force bound violated");check(std::abs(forces[l].x())<=.6*forces[l].z()+1e-6&&std::abs(forces[l].y())<=.6*forces[l].z()+1e-6,"QP friction constraint violated");}check(forces[0].norm()<1e-6,"Swing leg still has support force");
 std::cout<<"URDF mass/inertia, independent FK/Jacobian, swing IK, support wrench and friction/unloaded-foot constraints passed\n";
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}return 0;}
