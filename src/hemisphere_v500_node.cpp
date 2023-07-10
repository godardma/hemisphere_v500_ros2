// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "rclcpp/time.hpp"
// #include <time.hpp>

#include <ctime>
#include <chrono>

#include <iostream>
#include<iomanip>

#include <munu_io/AsyncService.h>
#include <munu_io/SerialDevice.h>
#include <munu_io/ClientTCP.h>
#include <munu_io/codecs.h>

#include <hemisphere_gnss/HemisphereReceiver.h>

#include "hemisphere_v500/msg/stamped_string.hpp"
#include <hemisphere_gnss/conversions.h>



using namespace std;
using namespace std::chrono_literals;

using SerialDevice = munu::SerialDevice<munu::AsyncDeviceWritable, std::chrono::steady_clock>;
using Hemisphere   = hemisphere_gnss::HemisphereReceiver<SerialDevice>;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

void nmea_callback(const Hemisphere* hemisphere,
                  rclcpp::Publisher<hemisphere_v500::msg::StampedString>::SharedPtr* pub,
                  const std::string& data)
{
    hemisphere_v500::msg::StampedString msg;
    std::chrono::steady_clock::time_point beg=hemisphere->device().timestamp();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(beg);
    auto epoch = now_ms.time_since_epoch();
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    int secs=duration/1000000000;
    int nanosecs=duration-secs*1000000000;

    msg.header.stamp.sec    = secs;
    msg.header.stamp.nanosec    = nanosecs;
    msg.header.frame_id = "v500";
    msg.data            = data.substr(0, data.size() - 2);
    auto inter =*pub->get();
    inter.publish(msg);
}

void binary_callback(const Hemisphere* hemisphere,
                    rclcpp::Publisher<hemisphere_v500::msg::StampedString>::SharedPtr* pub,
                    const std::vector<char>& data)
{
    hemisphere_v500::msg::StampedString msg;
    std::chrono::steady_clock::time_point beg=hemisphere->device().timestamp();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(beg);
    auto epoch = now_ms.time_since_epoch();
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    int secs=duration/1000000000;
    int nanosecs=duration-secs*1000000000;

    msg.header.stamp.sec    = secs;
    msg.header.stamp.nanosec    = nanosecs;
    msg.header.frame_id = "v500";
    msg.data            = munu::b64encode(data.data(), data.size());
    auto inter =*pub->get();
    inter.publish(msg);
}

void binary1_callback(const Hemisphere* hemisphere,
                      rclcpp::Publisher<hemisphere_v500::msg::Binary1>::SharedPtr* pub,
                      const std::vector<char>& data)
{
    auto header = reinterpret_cast<const SBinaryMsgHeader*>(data.data());
    if(header->blockID != 1) return;

    auto msg = hemisphere_v500::to_ros(
        *reinterpret_cast<const SBinaryMsg1*>(data.data()));
    std::chrono::steady_clock::time_point beg=hemisphere->device().timestamp();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(beg);
    auto epoch = now_ms.time_since_epoch();
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    int secs=duration/1000000000;
    int nanosecs=duration-secs*1000000000;

    msg.header.stamp.sec    = secs;
    msg.header.stamp.nanosec    = nanosecs;
    msg.header.frame_id = "v500";
    auto inter =*pub->get();
    inter.publish(msg);
}

void binary2_callback(const Hemisphere* hemisphere,
                      rclcpp::Publisher<hemisphere_v500::msg::Binary2>::SharedPtr* pub,
                      const std::vector<char>& data)
{
    auto header = reinterpret_cast<const SBinaryMsgHeader*>(data.data());
    if(header->blockID != 2) return;

    auto msg = hemisphere_v500::to_ros(
        *reinterpret_cast<const SBinaryMsg2*>(data.data()));
    std::chrono::steady_clock::time_point beg=hemisphere->device().timestamp();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(beg);
    auto epoch = now_ms.time_since_epoch();
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    int secs=duration/1000000000;
    int nanosecs=duration-secs*1000000000;

    msg.header.stamp.sec    = secs;
    msg.header.stamp.nanosec    = nanosecs;
    msg.header.frame_id = "v500";
    auto inter =*pub->get();
    inter.publish(msg);
}

void binary3_callback(const Hemisphere* hemisphere,
                      rclcpp::Publisher<hemisphere_v500::msg::Binary3>::SharedPtr* pub,
                      const std::vector<char>& data)
{
    auto header = reinterpret_cast<const SBinaryMsgHeader*>(data.data());
    if(header->blockID != 3) return;

    auto msg = hemisphere_v500::to_ros(
        *reinterpret_cast<const SBinaryMsg3*>(data.data()));
    std::chrono::steady_clock::time_point beg=hemisphere->device().timestamp();
    auto now_ms = std::chrono::time_point_cast<std::chrono::nanoseconds>(beg);
    auto epoch = now_ms.time_since_epoch();
    long duration = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    int secs=duration/1000000000;
    int nanosecs=duration-secs*1000000000;

    msg.header.stamp.sec    = secs;
    msg.header.stamp.nanosec    = nanosecs;
    msg.header.frame_id = "v500";
    auto inter =*pub->get();
    inter.publish(msg);
}

void rtk_callback(munu::ClientTCP<>* tcp,
                  std::shared_ptr<SerialDevice> serial,
                  std::vector<char>* buf,
                  const boost::system::error_code& err,
                  size_t byteCount)

{
    if(err) {
        std::ostringstream oss;
        oss << "Got error on rtk correction reception " << err;
        throw std::runtime_error(oss.str());
    }
    serial->write(buf->size(), buf->data()); // this is synchronous
    // tcp->async_read(buf->size(), buf->data(),
    //                 boost::bind(rtk_callback, tcp, serial, buf, _1, _2));
}

class MinimalPublisher : public rclcpp::Node
{
  
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    nmeaPub = this->create_publisher<hemisphere_v500::msg::StampedString>("nmea", 10);
    binaryPub = this->create_publisher<hemisphere_v500::msg::StampedString>("bin", 10);
    binary1Pub = this->create_publisher<hemisphere_v500::msg::Binary1>("bin1", 10);
    binary2Pub = this->create_publisher<hemisphere_v500::msg::Binary2>("bin2", 10);
    binary3Pub = this->create_publisher<hemisphere_v500::msg::Binary3>("bin3", 10);
    // timer_ = this->create_wall_timer(
    //   500ms, std::bind(&MinimalPublisher::timer_callback, this));




  }

  // void timer_callback()
  // {
  //   auto message = std_msgs::msg::String();
  //   message.data = "Hello, world! " + std::to_string(count_++);
  //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  //   publisher_->publish(message);
  // }





  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<hemisphere_v500::msg::StampedString>::SharedPtr nmeaPub;
  rclcpp::Publisher<hemisphere_v500::msg::StampedString>::SharedPtr binaryPub;
  rclcpp::Publisher<hemisphere_v500::msg::Binary1>::SharedPtr binary1Pub;
  rclcpp::Publisher<hemisphere_v500::msg::Binary2>::SharedPtr binary2Pub;
  rclcpp::Publisher<hemisphere_v500::msg::Binary3>::SharedPtr binary3Pub;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();

      munu::AsyncService service;
    string deviceName="/dev/narval_usbl";
    int baudrate = 230400;
    string rtkServerIP="127.0.0.1";
    int rtkServerPort=32898;

    cout << "Opening device : " << deviceName << ":" << baudrate << endl;
    hemisphere_gnss::HemisphereReceiver<SerialDevice> gnss(*service.io_service());
    gnss.open(deviceName, baudrate);
    cout<<"opened"<<endl;
    gnss.add_nmea_callback(std::bind(nmea_callback, &gnss, &node->nmeaPub,
                           std::placeholders::_1));
    gnss.add_binary_callback(std::bind(binary_callback, &gnss, &node->binaryPub,
                             std::placeholders::_1));
    gnss.add_binary_callback(std::bind(binary1_callback, &gnss, &node->binary1Pub,
                             std::placeholders::_1));
    gnss.add_binary_callback(std::bind(binary2_callback, &gnss, &node->binary2Pub,
                             std::placeholders::_1));
    gnss.add_binary_callback(std::bind(binary3_callback, &gnss, &node->binary3Pub,
                             std::placeholders::_1));

    cout << "Connecting to RTK correction server : "
         << rtkServerIP << ":" << rtkServerPort << endl;
    std::vector<char> rtkBuf(256);
    munu::ClientTCP<> tcp(*service.io_service());
    // cout<<"avant start"<<endl;
    service.start();
    // cout<<"apres start"<<endl;

  rclcpp::executors::MultiThreadedExecutor executor;
//   cout<<"ici ok"<<endl;
  executor.add_node(node);
//   cout<<"ici toujours ok"<<endl;
  executor.spin();
  service.stop();
  rclcpp::shutdown();
  return 0;
}