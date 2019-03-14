#include <ros/ros.h>
#include <serial/serial.h> 
#include <std_msgs/String.h>
#include <std_msgs/Empty.h> 
#include <nmea_msgs/Gpgga.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <string>
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

#define PI 3.141592653589793
serial::Serial ser; 
template <typename Type>  
Type stringToNum(const string &str)  
{  
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

template <typename Type>
string numToString (const Type &num)
{
	stringstream ss;
	string s;

	ss << num;
	s = ss.str();
	return s;
}

void gpggaManager(nmea_msgs::Gpgga &gpgga_msg, nav_msgs::Odometry &msg_gnssodometry,sensor_msgs::NavSatFix &msg_navsatfix, string &serial_data)
{

	string serialHeader;

	vector<int> separator_pos;

	struct SeparatorFormat {
		int first;
		int headerLength ; 
		int totalCommas;
	}gpgga,bestxyza,gptra;   //record the locate of every comma;

	gpgga.first = 6;
	gpgga.totalCommas =14;
	bestxyza.first = 9;
	bestxyza.totalCommas = 36; // BESTXYZ NOTICE: 9,s + 1; +  27,s
	gptra.first = 6;
	gptra.totalCommas = 8;

	separator_pos.push_back(serial_data.find(",",0));	

	serialHeader.assign(serial_data,1,separator_pos[0]-1);

	vector<string> ggaHeader; 

	ggaHeader.push_back("GPGGA");
	ggaHeader.push_back("BESTXYZA");
	ggaHeader.push_back("GPTRA");

	if (strcmp(serialHeader.c_str(),ggaHeader[0].c_str()) == 0)  //GPGGA
	{	
		// cout << serialHeader.c_str() << endl;

		for(int i=1;i<=gpgga.totalCommas-1;i++)
		{			
			separator_pos.push_back(serial_data.find(",",separator_pos[i-1]+1));
		}

		gpgga_msg.header.stamp = ros::Time::now();
		msg_navsatfix.header.stamp = ros::Time::now();

		string temp_gpgga;
		temp_gpgga.assign(serial_data,separator_pos[0]+1 ,separator_pos[1]-separator_pos[0]-1);
		gpgga_msg.utc_seconds = stringToNum<float>(temp_gpgga);

		temp_gpgga.assign(serial_data,separator_pos[1]+1 ,separator_pos[2]-separator_pos[1]-1);
		gpgga_msg.lat = stringToNum<float>(temp_gpgga);
		msg_navsatfix.latitude = stringToNum<float>(temp_gpgga);

		gpgga_msg.lat_dir = temp_gpgga.assign(serial_data,separator_pos[2]+1 ,separator_pos[3]-separator_pos[2]-1);

		temp_gpgga.assign(serial_data,separator_pos[3]+1 ,separator_pos[4]-separator_pos[3]-1);
		gpgga_msg.lon = stringToNum<float>(temp_gpgga);
		msg_navsatfix.longitude = stringToNum<float>(temp_gpgga);

		gpgga_msg.lon_dir = temp_gpgga.assign(serial_data,separator_pos[4]+1 ,separator_pos[5]-separator_pos[4]-1);

		temp_gpgga.assign(serial_data,separator_pos[5]+1 ,separator_pos[6]-separator_pos[5]-1);
		gpgga_msg.gps_qual = stringToNum<int>(temp_gpgga);

		temp_gpgga.assign(serial_data,separator_pos[6]+1 ,separator_pos[7]-separator_pos[6]-1);
		gpgga_msg.num_sats = stringToNum<int>(temp_gpgga);	

		temp_gpgga.assign(serial_data,separator_pos[7]+1 ,separator_pos[8]-separator_pos[7]-1);
		gpgga_msg.hdop = stringToNum<float>(temp_gpgga);	

		temp_gpgga.assign(serial_data,separator_pos[8]+1 ,separator_pos[9]-separator_pos[3]-1);
		gpgga_msg.alt = stringToNum<float>(temp_gpgga);
		msg_navsatfix.altitude = stringToNum<float>(temp_gpgga);

		gpgga_msg.altitude_units = temp_gpgga.assign(serial_data,separator_pos[9]+1 ,separator_pos[10]-separator_pos[9]-1);

		temp_gpgga.assign(serial_data,separator_pos[10]+1 ,separator_pos[11]-separator_pos[10]-1);//error of horizonal level
		gpgga_msg.undulation = stringToNum<float>(temp_gpgga);

		gpgga_msg.undulation_units = temp_gpgga.assign(serial_data,separator_pos[11]+1 ,separator_pos[12]-separator_pos[11]-1);

		temp_gpgga.assign(serial_data,separator_pos[12]+1 ,separator_pos[13]-separator_pos[12]-1);
		gpgga_msg.diff_age = stringToNum<int>(temp_gpgga);

		gpgga_msg.station_id = temp_gpgga.assign(serial_data,separator_pos[13]+1,4);

	}else if (strcmp(serialHeader.c_str(),ggaHeader[1].c_str()) == 0) //BESTXYZA
	{
		// cout << serialHeader.c_str() << endl;	

		for(int i=1;i<=bestxyza.totalCommas-1;i++)
		{			
			separator_pos.push_back(serial_data.find(",",separator_pos[i-1]+1));
		}
		msg_gnssodometry.header.stamp = ros::Time::now();
		msg_gnssodometry.header.frame_id = "gnss";

		string temp_bestxyza;

		temp_bestxyza.assign(serial_data,separator_pos[10]+1 ,separator_pos[11]-separator_pos[10]-1);  //x in ECEF
	    msg_gnssodometry.pose.pose.position.x = stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[11]+1 ,separator_pos[12]-separator_pos[11]-1);  //y in ECEF
	    msg_gnssodometry.pose.pose.position.y = stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[12]+1 ,separator_pos[13]-separator_pos[12]-1);  //z in ECEF
	    msg_gnssodometry.pose.pose.position.z = stringToNum<float>(temp_bestxyza); 
		std::vector<float> v2(36,0);

		temp_bestxyza.assign(serial_data,separator_pos[13]+1 ,separator_pos[14]-separator_pos[13]-1);  //std variance x in ECEF
	    msg_gnssodometry.pose.covariance[0] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza) ; 
		msg_navsatfix.position_covariance[0] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 


		temp_bestxyza.assign(serial_data,separator_pos[14]+1 ,separator_pos[15]-separator_pos[14]-1);  //std variance y in ECEF
	    msg_gnssodometry.pose.covariance[7] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 
	    msg_navsatfix.position_covariance[4] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[15]+1 ,separator_pos[16]-separator_pos[15]-1);  //std variance z in ECEF
	    msg_gnssodometry.pose.covariance[14] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 
	    msg_navsatfix.position_covariance[8] = stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza);

		temp_bestxyza.assign(serial_data,separator_pos[18]+1 ,separator_pos[19]-separator_pos[18]-1);  //vx in ECEF
		msg_gnssodometry.twist.twist.linear.x = stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[19]+1 ,separator_pos[20]-separator_pos[19]-1);  //vy in ECEF
		msg_gnssodometry.twist.twist.linear.y = stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[20]+1 ,separator_pos[21]-separator_pos[20]-1);  //vz in ECEF
		msg_gnssodometry.twist.twist.linear.z = stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[21]+1 ,separator_pos[22]-separator_pos[21]-1);  //std variance vx in ECEF
		msg_gnssodometry.twist.covariance[0]= stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 

		temp_bestxyza.assign(serial_data,separator_pos[22]+1 ,separator_pos[23]-separator_pos[22]-1);  //std variance vy in ECEF
		msg_gnssodometry.twist.covariance[7]= stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza); 
	
		temp_bestxyza.assign(serial_data,separator_pos[23]+1 ,separator_pos[24]-separator_pos[23]-1);  //std variance vz in ECEF
		msg_gnssodometry.twist.covariance[14]= stringToNum<float>(temp_bestxyza) * stringToNum<float>(temp_bestxyza);	
	
	    msg_navsatfix.position_covariance_type = 3;

	}else if (strcmp(serialHeader.c_str(),ggaHeader[2].c_str()) == 0)   //GPTRA
	{
		for(int i=1;i<=gptra.totalCommas-1;i++)
		{			
			separator_pos.push_back(serial_data.find(",",separator_pos[i-1]+1));
		}
		msg_gnssodometry.header.stamp = ros::Time::now();
		msg_gnssodometry.header.frame_id = "gnss";

		string temp_gptra;
		temp_gptra.assign(serial_data,separator_pos[1]+1 ,separator_pos[2]-separator_pos[1]-1);
    if (!temp_gptra.empty())
    {
		  float gnss_heading = stringToNum<float>(temp_gptra) / 180 * PI;

		  temp_gptra.assign(serial_data,separator_pos[2]+1 ,separator_pos[3]-separator_pos[2]-1);
		  float gnss_pitch= stringToNum<float>(temp_gptra) / 180 * PI;

		  temp_gptra.assign(serial_data,separator_pos[3]+1 ,separator_pos[4]-separator_pos[3]-1);
		  float gnss_roll= stringToNum<float>(temp_gptra) / 180 * PI;

		  Eigen::Vector3d ea0(gnss_heading,gnss_pitch,gnss_roll);
	      Eigen::Matrix3d R;
      	R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

          Eigen::Quaterniond q;
          q = R;    
	      cout << q.x() << endl << endl;
	      cout << q.y() << endl << endl;
	      cout << q.z() << endl << endl;
	      cout << q.w() << endl << endl;

	      msg_gnssodometry.pose.pose.orientation.x = q.x();
	      msg_gnssodometry.pose.pose.orientation.y = q.y();
	      msg_gnssodometry.pose.pose.orientation.z = q.z();
	      msg_gnssodometry.pose.pose.orientation.w = q.w();
    }
    else
    {
      msg_gnssodometry.pose.pose.orientation.x = 0.0;
      msg_gnssodometry.pose.pose.orientation.y = 0.0;
      msg_gnssodometry.pose.pose.orientation.z = 0.0;
      msg_gnssodometry.pose.pose.orientation.w = 1.0;
      msg_gnssodometry.pose.covariance[21] = -1;
      msg_gnssodometry.pose.covariance[28] = -1;
      msg_gnssodometry.pose.covariance[35] = -1;
    }

		// Eigen::Matrix3d Rx = q.toRotationMatrix();
	 //    Eigen::Vector3d ea1 = Rx.eulerAngles(2,1,0);     
  //   	cout << ea1/PI*180 - ea0 << endl << endl;

	}

	return;
}

// void writeCallback(const std_msgs::String::ConstPtr& msg) 
// { 
//     ROS_INFO_STREAM("Writing to serial port  " <<msg->data); 
    
//     ser.write(msg->data+ "\r\n"); //发送串口数据 

//     cout << msg->data << endl;
// }


int main (int argc, char** argv) { 
 
	ros::init(argc, argv, "serial_node"); 
	 
	ros::NodeHandle nh; 
	ros::NodeHandle param_nh("~");
	string port;
	int Baudrate,time_out,gpgga_freq,gptra_freq,bestxyza_freq;
	bool gpgga_enable,gptra_enable,bestxyza_enable,Unlogall_enable;

	param_nh.param("gnss_port",port,string("/dev/ttyUSB0"));
	param_nh.param("gnss_Baudrate",Baudrate,115200);
	param_nh.param("serial_timeout",time_out,10);
	param_nh.param("serial_timeout",time_out,10);
	param_nh.param("gpgga_enable",gpgga_enable,true);
	param_nh.param("gptra_enable",gptra_enable,true);
	param_nh.param("bestxyza_enable",bestxyza_enable,true);
	param_nh.param("Unlogall_enable",Unlogall_enable,false);
	param_nh.param("gpgga_freq",gpgga_freq,1);
	param_nh.param("gptra_freq",gptra_freq,1);
	param_nh.param("bestxyza_freq",bestxyza_freq,1);


	// ros::Subscriber write_sub = nh.subscribe("writeToSerial", 1, writeCallback); 


	ros::Publisher read_pub = nh.advertise<nmea_msgs::Gpgga>("gpgga", 1); 
	ros::Publisher read_pub2 = nh.advertise<nav_msgs::Odometry>("navOdometry", 1); 	
	ros::Publisher read_pub3 = nh.advertise<sensor_msgs::NavSatFix>("navSatFix", 1); 	

	//设置串口属性，并打开串口 
	ser.setPort(port); 
	ser.setBaudrate(Baudrate); 
	serial::Timeout to = serial::Timeout::simpleTimeout(time_out); 
	ser.setTimeout(to);
	try 
	{ 
		ser.open(); 
		} 
		catch (serial::IOException& e) 
		{ 
		ROS_ERROR_STREAM("Unable to open port "); 
		return -1; 
	} 

	if(ser.isOpen()) 
	{ 

		ROS_INFO_STREAM("Serial Port initialized"); 
		} 
		else 
		{
		 return -1; 
	} 

	if(gpgga_enable)
	{	
		cout << "GPGGA enable" << endl;
	    ser.write("log com1 gpgga ontime "+numToString(gpgga_freq)+"\r\n"); //发送串口数据 
	}

	if(gptra_enable)
	{
		cout << "GPTRA enable" << endl;
	    ser.write("log com1 gptra onchanged\r\n"); //发送串口数据 
	}
	if(bestxyza_enable)
	{
		cout << "BESTXYZA enable" << endl;
	    ser.write("log com1 bestxyza ontime "+numToString(bestxyza_freq)+"\r\n"); //发送串口数据 
	}
	if(Unlogall_enable)
	{

	    ser.write("Unlogall\r\n"); //发送串口数据 
	}

	ros::Rate loop_rate(50); //set the interval time for publishing message, if not set the rate, ros will try to publish as much as possible.
	ser.flush();

	while(ros::ok()) 
	{
		if(ser.available())
		{ 

			// ROS_INFO_STREAM("Reading from serial port\n"); 

			vector<std::string> vs;
			nmea_msgs::Gpgga msg_gpgga;
			nav_msgs::Odometry msg_gnssodometry;
			sensor_msgs::NavSatFix msg_navsatfix;

			// cout<<"avali before read: "<<ser.available()<<endl;
			vs = ser.readlines();
			// cout<<"avali after read: "<<ser.available()<<endl;
			// cout<<"read size: " << vs.size() <<endl;

			for(int i = 0 ; i<vs.size(); i++)
			{
				// cout<<"read contents: " << vs[i] <<endl;
				gpggaManager(msg_gpgga,msg_gnssodometry,msg_navsatfix,vs[i]);
			}
			
			read_pub.publish(msg_gpgga); 

			read_pub2.publish(msg_gnssodometry); 

			read_pub3.publish(msg_navsatfix);

			ser.flush();

		} 
	ros::spinOnce(); 

	loop_rate.sleep(); 

	}


} 

