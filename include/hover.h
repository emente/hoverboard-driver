#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <tf/transform_broadcaster.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Temperature.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#define PORT "/dev/ttyUSB1"
#define START_FRAME           0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND             20         // [ms] Sending time interval
#define DIRECTION_CORRECTION  1
#define WHEEL_RADIUS          0.0825
#define WHEEL_BASE            0.45
#define SPEED_MAX_TEST        200         // [-] Maximum speed for testing

class Hover {

public:
  Hover();
  ~Hover();

  void loop();
  void send_speed_to_bot(int16_t uSteer, int16_t uSpeed);

private:

  typedef struct{
    uint16_t	start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
  } SerialCommand;

  typedef struct{
    uint16_t start;
    int16_t 	cmd1;
    int16_t 	cmd2;
    int16_t 	speedR_meas; //100*km/h
    int16_t 	speedL_meas;
    int16_t 	batVoltage; //100*V
    int16_t 	boardTemp; //degC
    uint16_t cmdLed;
    uint16_t checksum;
  } SerialFeedback;

  ros::NodeHandle nh;
  uint8_t idx = 0;                        // Index for new data pointer
  uint16_t bufStartFrame;                 // Buffer Start Frame
  char *p;                                // Pointer declaration for the new received data
  char incomingByte;
  char incomingBytePrev;

  SerialCommand Command;
  SerialFeedback Feedback;
  SerialFeedback NewFeedback;

  // position in xy plane
  double self_x=0;
  double self_y=0;
  double self_th=0;

  uint16_t skipcnt=0;
  int port_fd = -1;

  ros::Publisher odom_pub;
  ros::Publisher battery_pub;
  ros::Publisher temp_pub;
  ros::Subscriber cmdvel_sub;
  tf::TransformBroadcaster odom_broadcaster;
  sensor_msgs::BatteryState battery_msg;
  sensor_msgs::Temperature temp_msg;
  ros::Time current_time, last_time;
  
  void read_from_bot();
  void publish_board_temp();
  void publish_battery();
  void publish_odom();

  void cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg);
};