#include <iostream>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization_evaluator");
  ros::NodeHandle n;

	tf::TransformListener listener;
	float err_x;
	float err_y;
	float err_z;
	float mean_err_x = 0;
	float mean_err_y = 0;
	float mean_err_z = 0;
	float max_err_x = 0;
	float max_err_y = 0;
	float max_err_z = 0;
	float delta_err_x; 
	float delta_err_y; 
	float delta_err_z;
	float var_err_x = 0; 
	float var_err_y = 0; 
	float var_err_z = 0;

	float count = 0;
	ros::Rate r(15);
	float wait_time = 30;
	float start = ros::Time::now().toSec();
	float finish = start+wait_time;

	while(ros::Time::now().toSec() < finish && ros::ok()){
		tf::StampedTransform transform;
		try{
			listener.lookupTransform("/world","/base_frame",ros::Time(0),transform);
		}
		catch (tf::TransformException ex){
			if(count == 0){
				continue;
			}
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		// if(ros::Time::now().toSec()-start < 2){
		// 	continue;
		// }
		count++;
		err_x = transform.getOrigin().x();
		err_y = transform.getOrigin().y();
		err_z = transform.getOrigin().z();
		
		delta_err_x = (fabs(err_x) - mean_err_x);
		delta_err_y = (fabs(err_y) - mean_err_y);
		delta_err_z = (fabs(err_z) - mean_err_z);


		mean_err_x += delta_err_x/count;
		mean_err_y += delta_err_y/count;
		mean_err_z += delta_err_z/count;

		var_err_x += delta_err_x*(fabs(err_x)-mean_err_x);
		var_err_y += delta_err_y*(fabs(err_x)-mean_err_y);
		var_err_z += delta_err_z*(fabs(err_x)-mean_err_z);


		// if(fabs(err_x)>max_err_x){
		// 	max_err_x = fabs(err_x);
		// }
		// if(fabs(err_y)>max_err_y){
		// 	max_err_y = fabs(err_y);
		// }
		// if(fabs(err_z)>max_err_z){
		// 	max_err_z = fabs(err_z);
		// }
		// if(max_err_x>2 || max_err_y>2 || max_err_z>2){
		// 	break;
		// }

		r.sleep();
	}
	if(count < 2){
		ROS_ERROR("NO POSE DATA");
		return 0;
	}
	var_err_x = var_err_x/(count-1);
	var_err_y = var_err_y/(count-1);
	var_err_z = var_err_z/(count-1);
	ROS_INFO("%0.5f,%0.5f,%0.5f",var_err_x,var_err_y,var_err_z);
	// (*data_file) << err_x << "\t" << err_y << "\t" << err_z << "\t" << mean_err_x << "\t" << mean_err_y << "\t" << mean_err_z << "\t" << max_err_x << "\t" << max_err_y << "\t" << max_err_z << "\t" << var_err_x << "\t" << var_err_y << "\t" << var_err_z;
	// std::cout << err_x << " " << err_y << " " << err_z << " " << mean_err_x << " " << mean_err_y << " " << mean_err_z << " " << max_err_x << " " << max_err_y << " " << max_err_z << std::endl;

  return 0;
}
