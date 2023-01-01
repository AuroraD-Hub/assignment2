//#include "aruco.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>
#include <iostream>
//#include <opencv2/highgui/highgui.hpp>
#include <assignment2/RoomConnection.h>
#include <assignment2/RoomInformation.h>
//#include <assignment2/CreateMap.h>
//#include <armor_api/armor_client/ArmorClient.h>

ros::ServiceClient client;
assignment2::RoomInformation info;
ros::Publisher pub_scan;
cv::Mat image;
aruco::MarkerDetector MDetector;
/*armor_api::armor_client::ArmorClient ont_client;

void ManipulateOntology(assignment2::RoomInformation info){
	ont_client.manipulation.add
	
	client.manipulation.disj_inds_of_class('LOCATION')
        client.manipulation.disj_inds_of_class('ROOM')
        client.manipulation.disj_inds_of_class('CORRIDOR')
        client.manipulation.disj_inds_of_class('URGENT')
        client.manipulation.disj_inds_of_class('DOOR')
	ont_client.utils.sync_buffered_reasoner()
}*/

bool ReadMarker(bool detected){

	//detect
	std::vector<aruco::Marker> markers=MDetector.detect(image);
	if (markers){
		std::cout << "Marker detected" << std::endl;
		//print info to console (to service)
		for(size_t i=0;i<markers.size();i++){ 
			std::cout<<markers[i]<<std::endl;
		}
		cv::imshow("image",image);
		cv::waitKey(0);
		//info.request.id = ;
		//client.call(info);
		//ManipulateOntology(info);
		
		detected = 1;
	}
	else{
		std::cout << "No marker detected" << std::endl;
		detected = 0;
	}
	
	return detected;
}

void DetectMarker(){
	int count=0;
	bool detected = 0;
	std_msgs::Float64 pos;
	
	while(count<8){
		detected = ReadMarker(detected);
		if(detected==1){
			count+=1;
			std::cout << "Marker found: " << count << std::endl;
		}
		pos = 0.0;
		pub_scan.publish(pos);
	}
}

/*void Callback_marker(assignment2::CreateMap::Request &req, assignment2::CreateMap::Response &res){
	if (req.create_map){
		DetectMarker();
		res.msg = "Ontology map correctly created" << std::endl;
	}
	else{
		res.msg = "Cannot create ontology map" << std::endl;
	}
}*/

int main(int argc,char **argv){
	if (argc != 2 ){
		std::cerr<<"Usage: inimage"<<std::endl;
		return -1;
	}
	
	ros::init(argc, argv, "markers");
	ros::NodeHandle nh;	
	client = nh.serviceClient<assignment2::RoomInformation>("/room_info");
	pub_scan = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command",1);
	image=cv::imread(argv[1]); 
	MDetector.setDictionary("ARUCO_MIP_36h12");
	
	//ros::ServiceServer oracle = nh.advertiseService("/markers_detection", Callback_marker);
	
	//Detect markers
	DetectMarker();
	
	ros::spin();
	return 0;
}
