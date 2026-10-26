#include "dummy_motion_client/manual_panel_profile.hpp"
#include <yaml-cpp/yaml.h>
#include <openssl/sha.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <limits>
#include <algorithm>
#include <stdexcept>
namespace dummy_motion_client {
namespace {
namespace fs=std::filesystem;
constexpr double rad=3.14159265358979323846/180.0;
void require(bool b,const std::string& m){if(!b)throw std::invalid_argument("manual deployment: "+m);}
std::string bytes(const fs::path& p){require(fs::is_regular_file(p)&&!fs::is_symlink(p),"regular file required: "+p.string());std::ifstream f(p,std::ios::binary);require(bool(f),"read failed");std::ostringstream s;s<<f.rdbuf();return s.str();}
std::string digest(const std::string& s){unsigned char d[32];SHA256(reinterpret_cast<const unsigned char*>(s.data()),s.size(),d);std::ostringstream o;for(auto c:d)o<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);return o.str();}
bool hex(const std::string& s,std::size_t n){return s.size()==n&&s.find_first_not_of("0123456789abcdef")==std::string::npos&&s!=std::string(n,'0');}
template<std::size_t N> std::array<std::uint8_t,N> decode(const std::string& s){require(hex(s,N*2),"invalid hex identity");std::array<std::uint8_t,N> a{};for(std::size_t i=0;i<N;++i)a[i]=std::stoul(s.substr(i*2,2),nullptr,16);return a;}
YAML::Node object(const std::string& s){auto n=YAML::Load(s);require(n.IsMap(),"object required");return n;}
}
ManualPanelDeployment load_manual_panel_deployment(const std::string& root,const std::string& pin,const std::string& sdk){
 require(hex(pin,64),"external manifest SHA256 pin required");
 const auto fixed=fs::canonical(root);require(fs::is_directory(fixed),"directory required");
 const auto manifest_bytes=bytes(fixed/"deployment.json");require(digest(manifest_bytes)==pin,"manifest digest mismatch");
 const auto manifest=object(manifest_bytes);require(manifest["format"].as<std::string>()=="dummy_manual_panel_deployment_v1","format mismatch");
 require(manifest["files"].IsMap(),"files required");
 // 检查清单每个原件，仅哈希；绝不执行Native或导入部署解码器。
 for(const auto& entry:manifest["files"]){auto relative=entry.first.as<std::string>();fs::path path(relative);require(!path.is_absolute()&&!relative.empty(),"relative payload path required");for(const auto& part:path)require(part!=".."&&part!=".","unsafe payload path");auto full=fixed/path;require(fs::canonical(full).string().rfind(fixed.string()+"/",0)==0,"payload escapes release");require(digest(bytes(full))==entry.second.as<std::string>(),"payload digest mismatch: "+relative);}
 require(bool(manifest["files"]["panel-config.json"])&&bool(manifest["files"]["firmware.bin"]),"required payload missing");
 auto cfg=object(bytes(fixed/"panel-config.json"));auto identity=manifest["identity"];
 require(cfg["deployment_manifest"].as<std::string>()=="deployment.json","manifest name mismatch");
 std::map<std::string,std::string> args;auto argv=cfg["native_arguments"];require(argv.IsSequence(),"bound deployment metadata required");
 for(std::size_t i=0;i<argv.size();++i){auto key=argv[i].as<std::string>();if(key.rfind("--",0)!=0)continue;if(key=="--execute")continue;require(i+1<argv.size()&&!args.count(key),"invalid bound metadata");args[key]=argv[++i].as<std::string>();}
 require(args["--scope"]=="manual-home-empty-supported"&&args["--profile"]=="201"&&args["--mask"]=="63","unsupported scope/profile/mask");
 require(cfg["wire_schema_major"].as<unsigned>()==2&&cfg["wire_schema_minor"].as<unsigned>()==5,"wire version mismatch");
 require(cfg["manual_limit_reference_enabled"].as<bool>()&&cfg["calibration_mode"].as<std::string>()=="manual_limits","manual reference disabled");
 ManualPanelDeployment out;out.deployment_root=fixed.string();out.endpoint=args["--endpoint"];out.device_id=args["--device"];out.client_id=args["--client"];out.profile_id=201;out.joint_mask=63;
 require(out.endpoint.rfind("serial//dev/serial/by-id/",0)==0&&out.endpoint.find("#baudrate=115200")!=std::string::npos,"by-id serial endpoint required");
 auto& t=out.transport;t.endpoint=out.endpoint;t.device_id=decode<16>(out.device_id);t.client_instance_id=decode<16>(out.client_id);t.profile_id=201;t.configured_mask=63;
 auto& p=out.profile;
 const auto contract=identity["contract_sha256"].as<std::string>(),model=identity["model_sha256"].as<std::string>(),policy=identity["profile_sha256"].as<std::string>(),firmware=identity["firmware_sha256"].as<std::string>();
 require(args["--contract"]==contract&&args["--model"]==model&&args["--profile-sha256"]==policy&&args["--firmware"]==firmware,"argument identity mismatch");
 // 本适配只覆盖这一实际编译策略；未知策略必须新增绑定实现，不能猜测seed能力。
 require(policy=="650cde3ea8f690b65f2a4273603053499ba786f7d108d80d9088ccd9d6bef996","unsupported live policy");
 require(digest(bytes(fixed/"firmware.bin"))==firmware,"firmware identity mismatch");
 const auto sdk_root=fs::canonical(sdk);const auto sdk_manifest_bytes=bytes(sdk_root/"manifest.json");
 require(digest(sdk_manifest_bytes)==DUMMY_INSTALLED_ROBOT_MANIFEST_SHA256,"compiled SDK manifest mismatch");
 auto sdk_manifest=object(sdk_manifest_bytes);require(sdk_manifest["release_id"].as<std::string>()=="dummy-wire-v2.5","SDK release mismatch");
 for(const auto& entry:sdk_manifest["payloads"]){auto rel=entry["path"].as<std::string>();fs::path path(rel);require(!path.is_absolute()&&rel.find("..") == std::string::npos,"unsafe SDK payload");require(digest(bytes(sdk_root/path))==entry["sha256"].as<std::string>(),"SDK payload mismatch");}
 p.model_hash=decode<32>(model);require(digest(bytes(sdk_root/"model/model.yaml"))==model,"installed SDK model mismatch");
 p.bundle_hash=decode<32>(digest(sdk_manifest_bytes));p.candidate_profile_payload_hash=decode<32>(digest(bytes(sdk_root/"profiles/f405_6dof_bench.yaml")));
 p.contract_hash=decode<32>(contract);p.profile_hash=decode<32>(policy);p.policy_hash=p.profile_hash;p.firmware_hash=decode<32>(firmware);p.firmware_bound=true;
 t.contract_hash=p.contract_hash;t.model_hash=p.model_hash;t.profile_hash=p.profile_hash;t.policy_hash=p.policy_hash;t.firmware_hash=p.firmware_hash;
 out.reference.deployment_sha256=decode<32>(pin);
 auto model_node=YAML::Load(bytes(fs::path(sdk)/"model/model.yaml"));
 require(cfg["joints"].size()==6&&model_node["joints"].size()==6,"six axes required");
 require(cfg["motion_speed_max_deg_s"].as<double>()==63.0&&cfg["motion_acceleration_max_rad_s2"].as<double>()==4.0,"live dynamic policy mismatch");
 const double working_margin_deg=cfg["working_margin_deg"].as<double>();
 require(std::isfinite(working_margin_deg)&&working_margin_deg==0.55,"working margin mismatch");
 // 与Native一致依赖REF按真实CAN SENT/反馈时间的连续跟踪监督；不额加未SENT本地期望的位置门。
 for(std::size_t i=0;i<6;++i){auto j=cfg["joints"][i],m=model_node["joints"][i];auto name="Joint"+std::to_string(i+1);require(j["name"].as<std::string>()==name&&m["name"].as<std::string>()==name,"joint order mismatch");
 out.direction[i]=j["direction"].as<double>();out.gear_ratio[i]=j["gear_ratio"].as<double>();require(out.direction[i]==m["direction"].as<double>()&&out.gear_ratio[i]==m["gear_ratio"].as<double>(),"transmission mismatch");
 auto lo=j["limits_deg"][0].as<double>(),hi=j["limits_deg"][1].as<double>();auto nlo=j["nominal_limits_deg"][0].as<double>(),nhi=j["nominal_limits_deg"][1].as<double>();
 require(nlo==m["legacy_limits_deg"][0].as<double>()&&nhi==m["legacy_limits_deg"][1].as<double>()&&std::abs(lo-nlo-0.55)<1e-10&&std::abs(nhi-hi-0.55)<1e-10,"motion bounds mismatch");
 require(std::isfinite(lo)&&std::isfinite(hi)&&std::isfinite(nlo)&&std::isfinite(nhi)&&lo<hi&&nlo<nhi,"invalid motion bounds");
 // REF先在名义弧度边界上加减工作余量；直接转换裁剪后的度值可能向外舍入1ULP。
 // 先将权威工作区向内取相邻浮点，再与已声明度制范围求交；交集端点不能重复内收。
 double lower=std::max(lo*rad,std::nextafter(nlo*rad+working_margin_deg*rad,std::numeric_limits<double>::infinity()));
 double upper=std::min(hi*rad,std::nextafter(nhi*rad-working_margin_deg*rad,-std::numeric_limits<double>::infinity()));
 require(std::isfinite(lower)&&std::isfinite(upper)&&lower<upper,"empty motion bounds intersection");
 p.joints.manual_inward_start=true;p.joints.nominal_lower.push_back(nlo*rad);p.joints.nominal_upper.push_back(nhi*rad);
 p.joints.joints.push_back(name);p.joints.lower.push_back(lower);p.joints.upper.push_back(upper);p.joints.velocity.push_back(1.0995574287564276);p.joints.acceleration.push_back(4.0);p.joints.path_tolerance.push_back(0.0);p.joints.goal_tolerance.push_back(0.003);
 t.position_min[i]=m["model_limits_rad"][0].as<double>();t.position_max[i]=m["model_limits_rad"][1].as<double>();t.velocity_max[i]=1.0995574287564276;t.acceleration_max[i]=4.0;
 out.reference.position_min[i]=t.position_min[i];out.reference.position_max[i]=t.position_max[i];
 // 已钉住650策略：63deg/s与8.75 motor rev/s经实测传动比均与2deg/s取交集。
 out.reference.rest_velocity_max[i]=std::min({2.0*rad,1.0995574287564276,8.75*2.0*3.14159265358979323846/out.gear_ratio[i]});
 out.reference.manual_limit_position[i]=cfg["calibration_pose_deg"][i].as<double>()*rad;out.reference.rest_position[i]=cfg["rest_deg"][i].as<double>()*rad;
 require(std::isfinite(out.reference.manual_limit_position[i])&&out.reference.rest_position[i]>=lo*rad&&out.reference.rest_position[i]<=hi*rad,"invalid reference/REST");
 }
 // configure_control的F405策略只覆写J1，J2..6保持模型绝对位置域。
 out.reference.position_min[0]=-3.2;out.reference.position_max[0]=3.2;
 t.position_min[0]=-3.2;t.position_max[0]=3.2;t.control_period_us=20000;t.state_publish_period_us=20000;t.max_state_age_us=250000;t.heartbeat_stop_budget_us=250000;
 out.control_period_us=20000;out.state_period_us=20000;p.motion_permit_max_us=3550000000ULL;
 out.reference.max_feedback_age_us=t.max_state_age_us;t.manual_reference_mode=true;
 return out;
}
}
