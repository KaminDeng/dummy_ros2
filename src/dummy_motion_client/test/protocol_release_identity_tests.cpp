/** @brief 当前SDK机器人包的运行wire身份、来源摘要及原件拒绝检查；无设备。 */
#include "dummy_motion_client/device_profile.hpp"
#include "dp_protocol_wire_contract.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <functional>
#include <vector>
#include <yaml-cpp/yaml.h>

std::string hex(const std::array<std::uint8_t,32>& value) {
  constexpr char digits[]="0123456789abcdef";
  std::string result;
  for (auto byte:value) { result+=digits[byte>>4U]; result+=digits[byte&15U]; }
  return result;
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const auto identity=dummy_motion_client::inspect_device_bundle_identity(argv[1],"sil_6dof");
  if(hex(identity.contract_hash)!=DP_PROTOCOL_WIRE_CONTRACT_SHA256 ||
     identity.contract_hash==identity.bundle_hash) {
    std::cerr << "运行wire身份必须等中央wire SHA且区别于来源聚合\n";
    return 3;
  }
  const auto copy=std::filesystem::temp_directory_path()/
    ("current_protocol_identity_"+std::to_string(getpid()));
  std::filesystem::remove_all(copy);
  std::filesystem::copy(argv[1],copy,std::filesystem::copy_options::recursive);
  const auto original=YAML::LoadFile((copy/"manifest.json").string());
  using Mutation=std::pair<const char*,std::function<void(YAML::Node&)>>;
  const std::vector<Mutation> mutations{
    {"schema_major",[](auto& m){m["schema_major"]=3;}},
    {"schema_minor",[](auto& m){m["schema_minor"]=4;}},
    {"release_id",[](auto& m){m["release_id"]="dummy-wire-v2.4";}},
    {"source_snapshot",[](auto& m){m["protocol_release"]["source_snapshot"]["kind"]="foreign";}},
    {"selection",[](auto& m){m["protocol_release"]["selection"]["codec"]="tlv-lite";}},
    {"runtime_contract",[](auto& m){m["runtime_contract_sha256"]=std::string(64,'1');}},
    {"removed_markers",[](auto& m){m.remove("contract_sha256");m.remove("wire_contract");m.remove("protocol_release");}},
  };
  for(const auto& [name,mutate]:mutations) {
    auto manifest=YAML::Clone(original);mutate(manifest);
    {std::ofstream output(copy/"manifest.json");output<<manifest;}
    bool rejected=false;
    try {(void)dummy_motion_client::inspect_device_bundle_identity(copy.string(),"sil_6dof");}
    catch(const std::invalid_argument&) {rejected=true;}
    if(!rejected) {std::cerr<<"wrapper元数据篡改未拒绝："<<name<<'\n';std::filesystem::remove_all(copy);return 10;}
  }
  {std::ofstream output(copy/"manifest.json");output<<original;}
  { std::ofstream changed(copy/"model/model.yaml",std::ios::app); changed<<"\n# 漂移\n"; }
  bool denied=false;
  try { (void)dummy_motion_client::inspect_device_bundle_identity(copy.string(),"sil_6dof"); }
  catch(const std::invalid_argument&) { denied=true; }
  std::filesystem::remove_all(copy);
  if(!denied)return 4;
  std::cout<<"当前wire身份与来源聚合分层，模型原件漂移被拒绝\n";
}
