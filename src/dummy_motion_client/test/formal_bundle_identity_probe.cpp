#include "dummy_motion_client/device_profile.hpp"
#include <iostream>
#include <exception>
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    const auto identity=dummy_motion_client::inspect_device_bundle_identity(argv[1],"sil_6dof");
    if(identity.firmware_bound)return 3;
    std::cout << "FORMAL_BUNDLE_IDENTITY_VERIFIED_NO_RUNTIME_AUTHORITY\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';return 1;
  }
}
