#include "dummy_motion_client/manual_panel_profile.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>
#include <sstream>
#include <iomanip>
using namespace dummy_motion_client;
int main(int argc,char** argv) {
 if(argc==1){
   try {load_manual_panel_deployment("/missing","", "/missing");return 1;}
   catch(const std::invalid_argument&){return 0;}
 }
 assert(argc==4);
 auto bad=[&](const std::string& root,const std::string& pin){try {load_manual_panel_deployment(root,pin,argv[3]);return false;}catch(const std::exception&){return true;}};
 assert(bad(argv[1],"")); assert(bad(argv[1],std::string(64,'0')));
 auto p=load_manual_panel_deployment(argv[1],argv[2],argv[3]);
 assert(!p.profile.bench_only && !p.profile.software_only_development);
 assert(p.profile_id==201 && p.joint_mask==63);
 // GUI39真实J2边界：直接转换89.45度比REF先减工作余量的上界高1ULP。
 const double rad=3.14159265358979323846/180.;
 const double unsafe=89.45*rad;
 const double canonical=std::nextafter(90.*rad-0.55*rad,-std::numeric_limits<double>::infinity());
 assert(p.profile.joints.upper[1] <= canonical);
 assert(unsafe > p.profile.joints.upper[1]);
 // 使用实际部署加载出的profile验证公开准入函数，不把边界断言冒充轨迹拒绝证明。
 Trajectory boundary_goal;boundary_goal.joints=p.profile.joints.joints;
 std::vector<double> q;
 for(std::size_t i=0;i<p.profile.joints.lower.size();++i)
   q.push_back((p.profile.joints.lower[i]+p.profile.joints.upper[i])/2.);
 q[1]=unsafe;
 boundary_goal.points={{q,{}, {},.04},{q,{}, {},.06}};
 assert(!validate_trajectory(boundary_goal,p.profile.joints));
 const double inward_deg=std::nextafter(std::nextafter(89.45,
   -std::numeric_limits<double>::infinity()),-std::numeric_limits<double>::infinity());
 for(auto& point:boundary_goal.points)point.positions[1]=inward_deg*rad;
 assert(validate_trajectory(boundary_goal,p.profile.joints));
 // GUI40真实测量略超工作边界，但仍在名义范围内，回Home必须保留真实起点。
 auto inward=boundary_goal;
 auto actual=q;actual[1]=89.47010551691055*rad;
 auto home=actual;home[1]=0.;
 inward.points={{actual,{},{},1.},{home,{},{},4.},{home,{},{},7.}};
 Position measured{};std::copy(actual.begin(),actual.end(),measured.begin());
 assert(validate_trajectory(inward,p.profile.joints));
 assert(validate_trajectory(inward,p.profile.joints,&measured));
 auto quantized_measurement=measured;quantized_measurement[1]=89.4699*rad;
 assert(validate_trajectory(inward,p.profile.joints,&quantized_measurement));
 auto discontinuous_measurement=measured;discontinuous_measurement[1]-=.0031;
 assert(!validate_trajectory(inward,p.profile.joints,&discontinuous_measurement));

 auto outward=inward;outward.points.insert(outward.points.begin()+1,{actual,{},{},2.});
 outward.points[1].positions[1]+=1e-6;
 assert(!validate_trajectory(outward,p.profile.joints));
 auto beyond=inward;beyond.points.front().positions[1]=90.01*rad;
 assert(!validate_trajectory(beyond,p.profile.joints));
 auto ordinary=p.profile.joints;ordinary.manual_inward_start=false;
 assert(!validate_trajectory(inward,ordinary));
 auto leave_again=inward;leave_again.points.insert(leave_again.points.begin()+2,{actual,{},{},5.});
 assert(!validate_trajectory(leave_again,p.profile.joints));
 auto lower_inward=inward;lower_inward.points.front().positions[1]=-72.47*rad;
 assert(validate_trajectory(lower_inward,p.profile.joints));
 auto explicit_outside=inward;
 for(auto& point:explicit_outside.points)point.velocities.assign(6,0.);
 assert(!validate_trajectory(explicit_outside,p.profile.joints));
 auto wrong_measured=measured;wrong_measured[1]=90.01*rad;
 assert(!validate_trajectory(inward,p.profile.joints,&wrong_measured));
 assert(inward.points.front().positions[1]==actual[1]);
 auto band_inside=inward;
 band_inside.points.front().positions[1]=p.profile.joints.upper[1]+.0029;
 assert(validate_trajectory(band_inside,p.profile.joints));
 auto band_outside=inward;
 band_outside.points.front().positions[1]=p.profile.joints.upper[1]+.0031;
 assert(!validate_trajectory(band_outside,p.profile.joints));




 for(double tolerance:p.profile.joints.path_tolerance)assert(tolerance==0);
 for(double tolerance:p.profile.joints.goal_tolerance)assert(tolerance==.003);
 assert(p.transport.control_period_us==20000 && p.transport.state_publish_period_us==20000);
 assert(p.reference.max_feedback_age_us==p.transport.max_state_age_us);
 assert(p.transport.position_min[0]==-3.2 && p.transport.velocity_max[0]>1.09);
 assert(p.reference.manual_limit_position[1]<p.profile.joints.lower[1]);
 assert(p.reference.manual_limit_position[2]>p.profile.joints.upper[2]);
 auto tmp=std::filesystem::temp_directory_path()/"dummy-manual-profile-tamper";
 std::filesystem::remove_all(tmp);std::filesystem::create_directories(tmp);
 std::filesystem::copy_file(std::filesystem::path(argv[1])/"deployment.json",tmp/"deployment.json");
 std::ofstream(tmp/"panel-config.json")<<"{}";
 assert(bad(tmp.string(),argv[2]));std::filesystem::remove_all(tmp);
 // 临时原件按完整清单重新钉住，仅验证配置语义；绝不改变实际部署。
 auto sha=[](const std::string& text){unsigned char hash[32];SHA256(reinterpret_cast<const unsigned char*>(text.data()),text.size(),hash);std::ostringstream out;for(auto c:hash)out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);return out.str();};
 std::filesystem::copy(argv[1],tmp,std::filesystem::copy_options::recursive);
 auto original=YAML::LoadFile((tmp/"panel-config.json").string());
 auto manifest=YAML::LoadFile((tmp/"deployment.json").string());
 auto write_cfg=[&](YAML::Node cfg){const auto text=YAML::Dump(cfg);std::ofstream(tmp/"panel-config.json")<<text;manifest["files"]["panel-config.json"]=sha(text);const auto m=YAML::Dump(manifest);std::ofstream(tmp/"deployment.json")<<m;return sha(m);};
 for(unsigned i=0;i<5;++i){auto cfg=YAML::Clone(original);switch(i){
 case 0:cfg.remove("working_margin_deg");break;
 case 1:cfg["working_margin_deg"]=std::numeric_limits<double>::quiet_NaN();break;
 case 2:cfg["joints"][1].remove("nominal_limits_deg");break;
 case 3:cfg["joints"][1]["nominal_limits_deg"][1]=std::numeric_limits<double>::infinity();break;
 case 4:cfg["joints"][1]["nominal_limits_deg"][0]=91.;break;
 }assert(bad(tmp.string(),write_cfg(cfg)));}
 std::filesystem::remove_all(tmp);

}
