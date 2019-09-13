#include <ros/ros.h>
#include <race/drive_param.h>
#include <race/drive_values.h>
#include <std_msgs/Int32.h>
#include <ros/callback_queue.h>

#include <ctime>
#include <chrono>
#include <thread>
using namespace std;

static int arduino_map(double x, int in_min, int in_max, int out_min, int out_max){
	return (int)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void callback(const boost::shared_ptr<const race::drive_param> &data, ros::Publisher &pub){
	double velocity = data.get()->velocity;
	double angle = data.get()->angle;
	cout << "Velocity: " << velocity << " Angle: " << angle << endl;

	race::drive_values msg;
	msg.pwm_drive = arduino_map(velocity,-100,100,6554,13108);
	msg.pwm_angle = arduino_map(angle,-100,100,6554,13108);
	pub.publish(msg);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "serial_talker");
	cout << "Serial talker initialized" << endl;

	int pub_queue_size = 10;
	int em_pub_queue_size = 10;
	int sub_queue_size = 1;
	int choice = 0;
	int given_time = 100;
	int hz = 10;

	cout << "Choose spinning method(0--Robust Spin, 1--Dynamic Spin, 2--Static Spin): ";
	cin >> choice;
	if(choice == 1){
		cout << "Enter Execution Time in milliseconds: ";
		cin >> given_time;
	}
	if(choice == 2){
		cout << "Enter Frequency in HZ: ";
		cin >> hz;
	}


	ros::NodeHandle n;
	
	ros::Publisher em_pub = n.advertise<std_msgs::Int32>("eStop", em_pub_queue_size);
	//em_pub.publish(0);

	ros::Publisher pub = n.advertise<race::drive_values>("drive_pwm", pub_queue_size);
	ros::Subscriber sub = n.subscribe<race::drive_param>("drive_parameters", sub_queue_size, boost::bind(callback, _1, pub));
	
	// Robust Spin
	if(choice == 0){
		ros::spin();
	}

	// Dynamic Spin
	else if(choice == 1){
		while(ros::ok()){
			// record the time in millisecond that used for the callback function
			auto start = std::chrono::high_resolution_clock::now();

			ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.01));

			auto end = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = end - start;
			int sleep_time = (int)(given_time - (double)elapsed.count()*1000);

			// Dynamically sleep
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		}
	}

	// Static Spin
	else if(choice == 2){
		ros::Rate r(hz);
		while (ros::ok()){
			ros::spinOnce();
			r.sleep();
		}
	}
	return 0;
}
