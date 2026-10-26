#include "dummy_motion_client/manual_panel_profile.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
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
}
