#include "StorageTestSupport.hpp"
#include "CameraReceiver.hpp"
#include <atlas/ui/CameraWireProtocol.hpp>
#include <atlas/ui/WireProtocol.hpp>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include <thread>
using namespace atlas;
namespace {
void wait_for(auto predicate) {
  for (unsigned i=0;i<2000;++i) { if (predicate()) return; std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
  REQUIRE(predicate());
}
void protocol_and_transport() {
  sensors::CameraRgbFrame frame;
  frame.width=2; frame.height=1; frame.timestamp_ns=123456789;
  frame.intrinsics={2,3,1,.5}; frame.camera_from_lidar_rotation={1,0,0,0,1,0,0,0,1};
  frame.rgb={20,40,200,200,40,20}; frame.calibration_verified=true; frame.tracking_normal=true;
  const auto bytes=ui::camera_wire::encode(frame);
  REQUIRE(bytes.size()==166 && bytes[0]==std::byte{'A'} && bytes[3]==std::byte{'R'});
  REQUIRE(ui::camera_wire::payload_size(std::span(bytes).first(12))==154);
  auto decoded=ui::camera_wire::decode(std::span(bytes).subspan(12));
  REQUIRE(decoded && decoded->rgb==frame.rgb && decoded->timestamp_ns==frame.timestamp_ns);
  REQUIRE(decoded->intrinsics==frame.intrinsics && decoded->camera_from_lidar_rotation==frame.camera_from_lidar_rotation);
  auto bad=bytes; bad[8]=bad[9]=bad[10]=bad[11]=std::byte{255};
  REQUIRE(!ui::camera_wire::payload_size(std::span(bad).first(12)));
  REQUIRE(!ui::camera_wire::decode(std::span(bytes).subspan(12,153)));
  bad=bytes; bad[20]=std::byte{4}; REQUIRE(!ui::camera_wire::decode(std::span(bad).subspan(12)));
  ui::CameraReceiver receiver([&](sensors::CameraRgbFrame got) {
    return got.rgb==frame.rgb && got.timestamp_ns==frame.timestamp_ns;
  });
  int pair[2]; REQUIRE(::socketpair(AF_UNIX,SOCK_STREAM,0,pair)==0);
  receiver.replace(pair[0]);
  // Arbitrary TCP fragmentation, followed by a second full message.
  for (auto b:bytes) REQUIRE(::send(pair[1],&b,1,0)==1);
  REQUIRE(::send(pair[1],bytes.data(),bytes.size(),0)==static_cast<ssize_t>(bytes.size()));
  wait_for([&]{ return receiver.received()==2; });
  REQUIRE(::send(pair[1],bad.data(),bad.size(),0)==static_cast<ssize_t>(bad.size()));
  wait_for([&]{ return receiver.rejected()==1; });
  ::close(pair[1]);
  REQUIRE(::socketpair(AF_UNIX,SOCK_STREAM,0,pair)==0);
  receiver.replace(pair[0]);
  REQUIRE(::send(pair[1],bytes.data(),bytes.size(),0)==static_cast<ssize_t>(bytes.size()));
  wait_for([&]{ return receiver.received()==3; });
  receiver.stop(); ::close(pair[1]); // stopping an idle receiver must unblock recv
  std::vector<sensors::LidarPoint> points{{.reflectivity=90,.rgb={20,40,200},.has_color=true},{.reflectivity=50}};
  const auto cloud=ui::wire::encode_point_cloud(points,1,1,ui::wire::kPointCloudWorldFrame|ui::wire::kPointCloudRgb);
  REQUIRE(cloud[44]==std::byte{20} && cloud[45]==std::byte{40} && cloud[46]==std::byte{200} && cloud[47]==std::byte{1});
  REQUIRE(cloud[60]==std::byte{50} && cloud[63]==std::byte{0});
}
}
int main() {
  try { protocol_and_transport(); }
  catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
  std::cout<<"Camera wire bounds, fragmented transport, reconnect and RGB downlink passed\n";
}
