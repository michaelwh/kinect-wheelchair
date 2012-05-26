/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* $Id: openni_viewer.cpp 4360 2012-02-10 01:01:11Z rusu $
*
*/

#define _USE_MATH_DEFINES

#include <boost/thread/thread.hpp>
#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/angles.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/crop_box.h>

#include <string>


#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
	do \
{ \
	static unsigned count = 0;\
	static double last = pcl::getTime ();\
	double now = pcl::getTime (); \
	++count; \
	if (now - last >= 1.0) \
	{ \
	std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
	count = 0; \
	last = now; \
	} \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
	do \
{ \
}while(false)
#endif

boost::mutex cld_mutex, img_mutex;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr g_cloud;
boost::shared_ptr<openni_wrapper::Image> g_image;

// Create the PCLVisualizer object
boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
boost::shared_ptr<pcl::visualization::ImageViewer> img;
#endif

bool status_text_set = false;

void setStatusText(const std::string& text, int viewport = 0) 
{
	if(status_text_set)
		cld->removeText3D("status_text", viewport);
	else
		status_text_set = true;

	cld->addText(text, 0, 0, "status_text", viewport);
}

void
	printHelp (int argc, char **argv)
{
	using pcl::console::print_error;
	using pcl::console::print_info;
	print_error ("Syntax is: %s <options>\n", argv[0]);
	print_info ("  where options are:\n");
	print_info ("                     -dev device_id           = device to be used\n");
	print_info ("                                                maybe \"#n\", with n being the number of the device in device list.\n");
	print_info ("                                                maybe \"bus@addr\", with bus and addr being the usb bus and address where device is connected.\n");
	print_info ("                                                maybe \"serial\", with serial being the serial number of the device.\n");
	print_info ("\n");
}



struct EventHelper
{
	void 
		cloud_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud)
	{
		FPS_CALC ("callback");
		cld_mutex.lock ();
		g_cloud = cloud;
		cld_mutex.unlock ();
	}

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
	void
		image_callback (const boost::shared_ptr<openni_wrapper::Image>& image)
	{
		FPS_CALC ("image callback");
		img_mutex.lock ();
		g_image = image;
		img_mutex.unlock ();
	}
#endif  
};
// Simple callbacks.
void 
	keyboard_callback (const pcl::visualization::KeyboardEvent& event, void* cookie)
{
	std::string* message = (std::string*)cookie;
	cout << (*message) << " :: ";
	if (event.getKeyCode())
		cout << "the key \'" << event.getKeyCode() << "\' (" << (int)event.getKeyCode() << ") was";
	else
		cout << "the special key \'" << event.getKeySym() << "\' was";
	if (event.keyDown())
		cout << " pressed" << endl;
	else
		cout << " released" << endl;
}

void 
	mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void* cookie)
{
	std::string* message = (std::string*) cookie;
	if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
	{
		cout << (*message) << " :: " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
	}
}

pcl::ModelCoefficients make_plane(float angle, float height) {
	float xa(0.0), yb(0.0), zc(0.0), d(0.0), x0(0.0), y0(0.0), z0(0.0);
	//printf("Angle: %f, zb: %f\n", angle,  sin(angle));
	pcl::ModelCoefficients modelCoeff;
	modelCoeff.values.resize(4);
					
	y0 = height;

	yb = cos(angle); // y (b)
	zc = sin(angle); // z (c)

	yb = yb / sqrt((yb * yb) + (zc *  zc));
	zc = zc / sqrt((yb * yb) + (zc *  zc));
											
	d = -(xa * x0) - (yb * y0) - (zc * z0); // d

	//d = ((float)height) / (sin(curr_angle) + ((cos(curr_angle) * cos(curr_angle)) / sin(curr_angle)));

	modelCoeff.values[0] = xa; // x (a)
	modelCoeff.values[1] = yb;
	modelCoeff.values[2] = zc;
	modelCoeff.values[3] = d;

	return modelCoeff;

}

void detect_ground_plane_brute_force(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud, Eigen::VectorXf& best_plane_coeffs) {
	const float angle_min = 0;
	const float angle_max = M_PI/6;
	const int i_max = 10;

	float best_height(0.0), best_angle(0.0);
	int best_count(0);
			
	// sensible height range for sensor on zpler lvl 3 desk
	for(float height = 0.25; height < 1.05; height += 0.1) {
		for (int i = 0; i < i_max; i++) {
		//int i = 0;
			const float curr_angle = angle_min + (angle_max - angle_min) * ((float)i / (float)i_max);	
			pcl::ModelCoefficients modelCoeff = make_plane(curr_angle, height);
						
			Eigen::VectorXf model_coeff_eigen(4);
			model_coeff_eigen[0] = modelCoeff.values[0];
			model_coeff_eigen[1] = modelCoeff.values[1];
			model_coeff_eigen[2] = modelCoeff.values[2];
			model_coeff_eigen[3] = modelCoeff.values[3];
						
			pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> sample_plane(g_cloud);
			int count_in_dist = sample_plane.countWithinDistance(model_coeff_eigen, 0.1);
			printf("Angle: %f Height: %f Count: %d\n", curr_angle, height, count_in_dist);

			if(count_in_dist > best_count) {
				best_count = count_in_dist;
				best_height = height;
				best_angle = curr_angle;
			}

			//char name[50];
			//sprintf(name, "gtp_%d_%f", i, height);
			//printf("Name: %s\n==============\n", name);
			//cld->addPlane(modelCoeff, name);
		}
	}

	//char best_name[50];
	//sprintf(best_name, "gtp_%f_%f", best_angle, best_height);
	//printf("Name: %s\n==============\n", name);
	pcl::ModelCoefficients best_model_coeff = make_plane(best_angle, best_height);
	//cld->addPlane(best_model_coeff, best_name);

	
	best_plane_coeffs.resize(4);
	best_plane_coeffs[0] = best_model_coeff.values[0];
	best_plane_coeffs[1] = best_model_coeff.values[1];
	best_plane_coeffs[2] = best_model_coeff.values[2];
	best_plane_coeffs[3] = best_model_coeff.values[3];

}


/* ---[ */
int
	main (int argc, char** argv)
{
	if (argc > 1)
	{
		for (int i = 1; i < argc; i++)
		{
			if (std::string (argv[i]) == "-h")
			{
				printHelp (argc, argv);
				return (-1);
			}
		}
	}

	EventHelper event_helper;
	std::string device_id = "";
	pcl::console::parse_argument (argc, argv, "-dev", device_id);

	pcl::Grabber* viewer = new pcl::OpenNIGrabber (device_id);

	cld.reset (new pcl::visualization::PCLVisualizer (argc, argv, "OpenNI Viewer"));

	std::string mouseMsg3D ("Mouse coordinates in PCL Visualizer");
	std::string keyMsg3D ("Key event for PCL Visualizer");
	cld->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg3D));    
	cld->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg3D));
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f = boost::bind (&EventHelper::cloud_cb, &event_helper, _1);
	boost::signals2::connection c1 = viewer->registerCallback (f);

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
	img.reset (new pcl::visualization::ImageViewer ("OpenNI Viewer"));
	// Register callbacks
	std::string keyMsg2D ("Key event for image viewer");
	std::string mouseMsg2D ("Mouse coordinates in image viewer");
	img->registerMouseCallback (&mouse_callback, (void*)(&mouseMsg2D));
	img->registerKeyboardCallback(&keyboard_callback, (void*)(&keyMsg2D));
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&) > image_cb = boost::bind (&EventHelper::image_callback, &event_helper, _1);
	boost::signals2::connection image_connection = viewer->registerCallback (image_cb);
	unsigned char* rgb_data = 0;
	unsigned rgb_data_size = 0;
#endif 

	viewer->start();

	Eigen::VectorXf ground_plane_model_coeffs(4);

	bool cld_init = false;
	// Loop
	while (!cld->wasStopped ())
	{
		// Render and process events in the two interactors
		cld->spinOnce ();
#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
		img->spinOnce ();
#endif
		FPS_CALC ("drawing");

		// Add the cloud
		if (g_cloud && cld_mutex.try_lock ())
		{
			if (!cld_init)
			{
				// first run
				cld->getRenderWindow ()->SetSize (g_cloud->width, g_cloud->height);
				cld->getRenderWindow ()->SetPosition (g_cloud->width, 0);
				
				//detect_ground_plane_brute_force(*g_cloud, ground_plane_model_coeffs);

				//cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(15.0), 0.0, 1.0, 0.0), 0.1, 3.0, 10.0, "cube_test_ang_max");
				//cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(-15.0), 0.0, 1.0, 0.0), 0.1, 3.0, 10.0, "cube_test_ang_min");
				//{
					//int name_i = 0;
					//for (float angle = -15.0; angle < 15; angle += 30.0/20.0) {
						//char name[50];
						//sprintf(name, "decimation_cube_test_%d", name_i);
						//cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(angle), 0.0, 1.0, 0.0), 0.5, 3.0, 10.0, name);
						//name_i++;
					//}
				//}

				cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(90.0), 0.0, -1.0, 0.0), 0.5, 3.0, 10.0, "best_cube");

				cld_init = !cld_init;
			}

			//setStatusText("Calculating ground points...");
			//std::vector<int> inliers;
			//pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> sample_plane(g_cloud);
			//sample_plane.selectWithinDistance(ground_plane_model_coeffs, 0.08, inliers);
			
			pcl::copyPointCloud(*g_cloud, *new_cloud);

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_detection_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>()); // the point cloud to be used for the ground plane detection

			// -------------------------------
			// filter cloud to cut out ceiling
			// -------------------------------
			pcl::PassThrough<pcl::PointXYZRGBA> pt_filter(false); // we don't need extracted indicies, if we did we would give true to constructor
			pt_filter.setInputCloud(new_cloud);
			pt_filter.setFilterFieldName("y");
			pt_filter.setFilterLimits(-0.8, 10.0);
			pt_filter.setFilterLimitsNegative(false);
			pt_filter.filter(*ground_detection_cloud);
			//pt_filter.filter(*temp_cloud);
			//pcl::IndicesConstPtr no_ceil_indicies = pt_filter.getRemovedIndices();
			// ------------------------------------------------------------------------
			// filter cloud to cut out close things that mess with our ground detection
			// ------------------------------------------------------------------------
			pt_filter.setInputCloud(ground_detection_cloud);
			pt_filter.setFilterFieldName("z");
			pt_filter.setFilterLimits(0.0, 1.0);
			pt_filter.setFilterLimitsNegative(true);
			pt_filter.filter(*ground_detection_cloud);
			
			
			
			// some code here taken from: http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
			// and http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
			// downsample the data to speed stuff up
			//pcl::VoxelGrid<pcl::PointXYZRGBA> vox_filter;
			//vox_filter.setInputCloud(new_cloud);
			//vox_filter.setLeafSize(0.1, 0.1, 0.1);
			//vox_filter.filter(*new_cloud);

			pcl::ModelCoefficients floor_coefficients;
			pcl::PointIndices::Ptr ground_cloud_floor_inliers(new pcl::PointIndices());
			// Create the segmentation object
			pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
			// Optional
			seg.setOptimizeCoefficients (true);
			// Mandatory
			seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (0.04);
			seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
			seg.setEpsAngle(pcl::deg2rad(10.0));

			seg.setInputCloud(ground_detection_cloud);
			//seg.setIndices(no_ceil_indicies);
			seg.segment(*ground_cloud_floor_inliers, floor_coefficients);

			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_detection_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
			

			if (ground_cloud_floor_inliers->indices.size () == 0)
			{
				PCL_ERROR ("Could not estimate a planar model for the given dataset.");
				pcl::copyPointCloud(*new_cloud, *object_detection_cloud);
			} else {
				/*pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
				extract.setInputCloud(ground_detection_cloud);
				extract.setIndices(ground_cloud_floor_inliers);
				extract.setNegative(false);
				extract.filter(*ground_cloud);*/
				
				
				pcl::SampleConsensusModelPlane<pcl::PointXYZRGBA> ground_plane_model(new_cloud);

				std::vector<int> new_cloud_floor_inliers;
				Eigen::VectorXf floor_coefficients_xf;
				floor_coefficients_xf.resize(4);
				for(int i = 0; i < floor_coefficients.values.size(); i++)
					floor_coefficients_xf[i] = floor_coefficients.values[i];

				ground_plane_model.selectWithinDistance(floor_coefficients_xf, 0.04, new_cloud_floor_inliers);

				for(int i = 0; i < new_cloud_floor_inliers.size(); i++) {
					new_cloud->points[new_cloud_floor_inliers[i]].r = 255;
					new_cloud->points[new_cloud_floor_inliers[i]].g = 0;
					new_cloud->points[new_cloud_floor_inliers[i]].b = 0;
				}

				
			
				pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
				extract.setInputCloud(new_cloud);
				pcl::PointIndices::Ptr new_cloud_floor_inliers_ptr(new pcl::PointIndices());
				new_cloud_floor_inliers_ptr->indices = new_cloud_floor_inliers;
				extract.setIndices(new_cloud_floor_inliers_ptr);
				extract.setNegative(true);
				extract.filter(*object_detection_cloud);
				extract.setNegative(false);
				extract.filter(*ground_cloud);


				// later we should try to transform the point cloud to lie flat (i.e. rotate it)
				// what about this http://steve.hollasch.net/cgindex/math/rotvecs.html
			}



		
			//setStatusText("Setting ground points...");

			

			
			//std::vector<int> cloud_indicies(new_cloud->size());
			//for(int i = 0; i < new_cloud->size(); i++)
			//	cloud_indicies[i] = i;

			
			//pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA> ground_plane_model(new_cloud);
			//ground_plane_model.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
			//ground_plane_model.setEpsAngle(pcl::deg2rad(15.0));
			//ground_plane_model.computeModelCoefficients(cloud_indicies, ground_plane_model_coeffs);
			//std::vector<int> ground_inliers;
			//ground_plane_model.selectWithinDistance(ground_plane_model_coeffs, 0.01, ground_inliers);


			
		

			
		//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tr(new pcl::PointCloud<pcl::PointXYZRGBA>());
		
			
			{
				
				int most_indicies = 0;
				float best_angle = 0;
				
				int name_i = 0;
				for (float angle = -15.0; angle < 15; angle += 30.0/20.0) {
					char name[50];
					sprintf(name, "decimation_cube_test_%d", name_i);
					//cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(angle), 0.0, 1.0, 0.0), 0.5, 3.0, 10.0, name);

					Eigen::Vector4f boxmin;//(-0.25, -(3.0/2.0), -5.0);
					boxmin.resize(3);
					boxmin[0] = -0.25;
					boxmin[1] = -(3.0/2.0);
					boxmin[2] = -5.0;
					Eigen::Vector4f boxmax;
					boxmax.resize(3);
					boxmax[0] = 0.25;
					boxmax[1] = 3.0/2.0;
					boxmax[2] = 5.0;

					Eigen::Vector3f boxrotate;
					boxrotate.resize(3);
					boxrotate[0] = 0.0;
					boxrotate[1] = pcl::deg2rad(angle * 2.0); // for some reason this needs to be multiplied by 2 to correspond to the cld->addCube quaterion angle
					boxrotate[2] = 0.0;
 
					std::vector<int> cropped_indicies;
					pcl::CropBox<pcl::PointXYZRGBA> chair_crop_box;
					//chair_crop_box.setInputCloud(object_detection_cloud);
					chair_crop_box.setInputCloud(ground_cloud);
					chair_crop_box.setMin(boxmin);
					chair_crop_box.setMax(boxmax);
					chair_crop_box.setRotation(boxrotate);
					chair_crop_box.filter(cropped_indicies);
			
					//if(name_i == 2) {
					//	for(int i = 0; i < cropped_indicies.size(); i++) {
					//		ground_cloud->points[cropped_indicies[i]].r = 0;
					//		if(name_i % 2 == 0) {
					//			ground_cloud->points[cropped_indicies[i]].g = 0;
					//			ground_cloud->points[cropped_indicies[i]].b = 255;
					//		} else {
					//			ground_cloud->points[cropped_indicies[i]].g = 255;
					//			ground_cloud->points[cropped_indicies[i]].b = 0;
					//		}	
					//	}
					//	cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(angle), 0.0, -1.0, 0.0), 0.5, 3.0, 10.0, name);
					//}

					printf("%d indicies at %f degrees\n", cropped_indicies.size(), angle);

					if(cropped_indicies.size() > most_indicies) {
						most_indicies = cropped_indicies.size();
						best_angle = angle;
					}
					
					
					//cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(angle), 0.0, -1.0, 0.0), 0.5, 3.0, 10.0, name);
					

					name_i++;
				}

				printf("BEST: %d indicies at %f degrees\n", most_indicies, best_angle);

				cld->removeShape("best_cube"); // we should have already added one right at the beginning so it should be ok to remove it here
				cld->addCube(Eigen::Vector3f(0.0, 0.0, 0.0), Eigen::Quaternionf(pcl::deg2rad(best_angle), 0.0, -1.0, 0.0), 0.5, 3.0, 10.0, "best_cube");
				
			}



			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> handler (new_cloud);
			if (!cld->updatePointCloud (new_cloud, handler, "OpenNICloud"))
			{
				cld->addPointCloud (new_cloud, handler, "OpenNICloud");
				cld->resetCameraViewpoint ("OpenNICloud");
			}
			cld_mutex.unlock ();
		}

#if !((VTK_MAJOR_VERSION == 5)&&(VTK_MINOR_VERSION <= 4))
		// Add the image
		if (g_image && img_mutex.try_lock ())
		{
			if (g_image->getEncoding() == openni_wrapper::Image::RGB)
				img->showRGBImage (g_image->getMetaData ().Data (), 
				g_image->getWidth (), g_image->getHeight ());
			else
			{
				if (rgb_data_size < g_image->getWidth () * g_image->getHeight ())
				{
					rgb_data_size = g_image->getWidth () * g_image->getHeight ();
					rgb_data = new unsigned char [rgb_data_size * 3];
				}
				g_image->fillRGB (g_image->getWidth (), g_image->getHeight (), rgb_data);
				img->showRGBImage (rgb_data, g_image->getWidth (), g_image->getHeight ());
			}
			img_mutex.unlock ();
		}
#endif
		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}

	viewer->stop ();
}
/* ]--- */
