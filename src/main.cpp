#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define PORT "/dev/ttyUSB01"
#define DIRECTION_CORRECTION 1
#define WHEEL_RADIUS 0.0825
#define WHEEL_BASE 0.45

#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           20         // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // [-] Maximum speed for testing

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
char *p;                                // Pointer declaration for the new received data
char incomingByte;
char incomingBytePrev;

typedef struct{
   uint16_t	start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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
SerialFeedback Feedback;
SerialFeedback NewFeedback;

int port_fd = -1;

ros::NodeHandle nh;
ros::Publisher odom_pub;
ros::Publisher battery_pub;
ros::Subscriber cmd_vel_sub;
tf::TransformBroadcaster odom_broadcaster;
sensor_msgs::BatteryState battery_msg;

// position in xy plane
double self_x=0;
double self_y=0;
double self_th=0;

ros::Time current_time, last_time;

void send_speed_to_bot(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  int r = ::write(port_fd, &Command, sizeof(Command)); 
}

void cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    send_speed_to_bot(msg->linear.x - msg->angular.z * (WHEEL_BASE / 2), msg->linear.x + msg->angular.z * (WHEEL_BASE / 2)); 
}

void publish_odom()
{
    double x, y, th, d, d_left, d_right;
    double elapsed = (current_time - last_time).toSec();

    //convert km/h to m/s
    d_left  = (Feedback.speedL_meas*1000)/3600;
    d_right = (Feedback.speedL_meas*1000)/3600;

    //distance traveled as average of both wheels
    d = (d_left + d_right) / 2;
    th = (d_right - d_left) / WHEEL_BASE; 

    //calculate velocities
    double dx = d / elapsed;
    double dr = th / elapsed;

    //calculate distance traveled and final position
    if (d != 0)
    {
        //calculate distance traveled
        x = cos(th) * d;
        y = -sin(th) * d;
        //calculate final position
        self_x = self_x + (cos(self_th) * x - sin(self_th) * y);
        self_y = self_y + (sin(self_th) * x + cos(self_th) * y);
    }
    if (th != 0)
    {
        self_th = self_th + th;
    }

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(self_th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = self_x;
    odom_trans.transform.translation.y = self_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = self_x;
    odom.pose.pose.position.y = self_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = dr;
    odom_pub.publish(odom);
}


void publish_battery() 
{
    battery_msg.voltage = Feedback.batVoltage/100;
    battery_pub.publish(battery_msg);
}


void read_from_bot()
{
    int r = ::read(port_fd, &incomingByte, 1);
    if (r<0) {
        return;
    }

    bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	  // Construct the start frame		

	// Copy received data
	if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
		p 		= (char *)&NewFeedback;
    *p++  = incomingBytePrev;
		*p++ 	= incomingByte;
		idx 	= 2;	
	} else if (idx >= 2 && idx < sizeof(SerialFeedback)) {	// Save the new received data
		*p++ 	= incomingByte; 
		idx++;
	}	
	
	// Check if we reached the end of the package
	if (idx == sizeof(SerialFeedback)) {  	
		uint16_t checksum;
		checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
					^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
	
		// Check validity of the new data
		if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
			// Copy the new data
			memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
		  
            publish_battery();
            publish_odom();
		} else {
		  ROS_ERROR("hoverboard: wrong checksum or non-valid data received");
		}
		idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
	}
 	
	// Update previous states
	incomingBytePrev 	= incomingByte;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard_driver");

    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL("Cannot open serial port to hoverboard");
        exit(-1);
    }
    
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 1);
    cmd_vel_sub = nh.subscribe("cmd_vel",1,cmdvel_callback);

    ros::Time prev_time = ros::Time::now();
    while (ros::ok()) {
        read_from_bot();
    }

    close(port_fd);
    return 0;
}
