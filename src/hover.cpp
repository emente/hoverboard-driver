#include "hover.h"

Hover::Hover()
{
    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        ROS_FATAL_STREAM("hoverboard: cannot open serial port " << PORT);
        exit(-1);
    }
    
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery", 1);
    temp_pub = nh.advertise<sensor_msgs::Temperature>("driver_temp", 1);
    cmdvel_sub = nh.subscribe("cmd_vel", 1, &Hover::cmdvel_callback, this);

    last_time = ros::Time::now();
}

Hover::~Hover() {
  close(port_fd);
}

void Hover::loop()
{
  current_time = ros::Time::now();
  read_from_bot();
}

void Hover::send_speed_to_bot(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  int r = ::write(port_fd, &Command, sizeof(Command)); 
}

void Hover::cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    send_speed_to_bot(msg->linear.x - msg->angular.z * (WHEEL_BASE / 2), msg->linear.x + msg->angular.z * (WHEEL_BASE / 2)); 
}

void Hover::publish_odom()
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


void Hover::publish_battery() 
{
    battery_msg.present = true;
    battery_msg.voltage = Feedback.batVoltage/100;
    battery_pub.publish(battery_msg);
}

void Hover::publish_board_temp()
{
    temp_msg.temperature = Feedback.boardTemp/100;
    temp_pub.publish(temp_msg);
}


void Hover::read_from_bot()
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
            publish_odom();
            if (skipcnt++>100) {
                publish_battery();
                publish_board_temp();
                skipcnt=0;
            }

		} else {
		  ROS_ERROR("hoverboard: wrong checksum or non-valid data received");
		}
		idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
	}
 	
	// Update previous states
	incomingBytePrev 	= incomingByte;
}
