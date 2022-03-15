#include <math.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <mav_msgs/Actuators.h>

#include "libsocket/inetclientdgram.hpp"
#include "libsocket/exception.hpp"

mav_msgs::Actuators w_msg2;
float Arr[4];

class UDPPoseStreamer {
  public:
  
  UDPPoseStreamer(const std::string & udp_address,
                  const std::string & udp_port, 
                  const float x_offset, 
                  const float y_offset) 
  : dgram_client(LIBSOCKET_IPv4), 
    ip_address(udp_address), 
    port(udp_port), 
    nh("~"),
    x_shift(x_offset),
    y_shift(y_offset) {
    global_frame="true_body";
    tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));
    sub = nh.subscribe("pose_topic", 1, &UDPPoseStreamer::PublishPose, this);
  }
  
  virtual ~UDPPoseStreamer() { dgram_client.destroy(); }
  
  //private:
  void PublishPose(const mav_msgs::Actuators& msg) {

      // TODO what is the purpose of this function?
      // note msg does not even seem to be used...
    float w[4] = {
      static_cast<float>(Arr[0]),
      static_cast<float>(Arr[1]),
      static_cast<float>(Arr[2]),
      static_cast<float>(Arr[3]),
    };
    
    bool accept = true;
    for(uint i=0;i<4;i++){
      if(std::isnan(w[i])){
          accept=false;
          break;
      }
    }

    if(accept){
        static const size_t int32_size = sizeof(uint32_t);
        static const size_t pose_size = sizeof(float) * 4;
        static const size_t packet_size = pose_size;

        uint8_t packet_data[packet_size];
        memcpy(packet_data , &w, pose_size);
        dgram_client.sndto(&packet_data, packet_size, ip_address, port);
    }else{
        std::cout <<"Received nans, not sending! \n";
    }
  }

  void PublishMarker(const geometry_msgs::PolygonStamped& msg) {
    if(msg.polygon.points.size() > (counter+1)){
            counter++;
        }else{
            counter = 0;
        }
  
    float coordinates[3] = {
      static_cast<float>(msg.polygon.points[counter].x),
      static_cast<float>(msg.polygon.points[counter].y),
      static_cast<float>(msg.polygon.points[counter].z)
    };
    
    bool accept = true;
    for(uint i=0;i<3;i++){
      if(std::isnan(coordinates[i])){
          accept=false;
          break;
      }
    }

    if(accept){
        static const size_t int32_size = sizeof(uint32_t);
        static const size_t pose_size = sizeof(float) * 3;
        static const size_t packet_size = pose_size;

        uint8_t packet_data[packet_size];
        //复制pose_size个coordinates的字节到packet_data
        memcpy(packet_data , &coordinates, pose_size);
        dgram_client.sndto(&packet_data, packet_size, ip_address, "12348");
    }else{
        std::cout <<"Received nans, not sending! \n";
    }
  }

  ros::Subscriber sub;
  libsocket::inet_dgram_client dgram_client;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  std::string ip_address;
  std::string port;
  std::string global_frame;

  float x_shift, y_shift;
  ros::NodeHandle nh;

  unsigned int counter;
};

void wCallback(const mav_msgs::Actuators& cmd)
{
  Arr[0]=cmd.angular_velocities[0];
  Arr[1]=cmd.angular_velocities[1];
  Arr[2]=cmd.angular_velocities[2];
  Arr[3]=cmd.angular_velocities[3];

  std::cout <<"Message received: "<< Arr[0] << " "  << Arr[1] << " "<< Arr [2] << " "  << Arr[3] << " " << "\n";

  return;
}


int main(int argc, char **argv)
{
  //初始化ros，创建一个叫"w_to_unity"的节点
  ros::init(argc, argv, "w_to_unity");

  ros::NodeHandle n;

  //设置一个想要的循环频率
  ros::Rate loop_rate(1000);

  std::string ip_address, port;
  float offset_x, offset_y;
  port="12346";
  ip_address="127.0.0.1";//128.30.10.229";
  offset_x=0;
  offset_y=0;
  std::cout << "This node enables send communication to Unity\n";
  
  //和unity建立联系
  UDPPoseStreamer streamer(ip_address, port, offset_x, offset_y);

  float counter=0;
  //接收无人机四轴的旋转速度
  ros::Subscriber sub = n.subscribe("rotor_speed_cmds", 1, wCallback);
  //接收受害者坐标
  ros::Subscriber markerSub = n.subscribe("victimCoord_continous_topic", 10, &UDPPoseStreamer::PublishMarker, &streamer);

  while (ros::ok())
    { 
      //利用udp协议把无人机的角度，角速度或其他指令（需自己扩展）发送给无人机
      streamer.PublishPose(w_msg2);

      ros::spinOnce();

      loop_rate.sleep();
    }
  return 0;
}
