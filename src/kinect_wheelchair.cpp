#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
//#ifdef _WIN32
// # define sleep(x) Sleep((x)*1000)
// #endif


// the base franework for this from http://www.pointclouds.org/documentation/tutorials/openni_grabber.php
// and http://pointclouds.org/documentation/tutorials/cloud_viewer.php
class SimpleOpenNIViewer
{
public:

	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") {}

	void viewerOneOff(pcl::visualization::PCLVisualizer& pclViewer)	
	{
		pcl::ModelCoefficients plane_coeff;
		plane_coeff.values.resize(4);
		plane_coeff.values[0] = 0.0; // x
		plane_coeff.values[1] = -1.0; // y
		plane_coeff.values[2] = 0.0; // z
		plane_coeff.values[3] = 0.0; // d
		pclViewer.
		pclViewer.addPlane(plane_coeff, "ground_test_plane", 0);
	}

	void viewerIteration(pcl::visualization::PCLVisualizer& pclViewer)
	{

	}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{

		if (!viewer.wasStopped()) {
			//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

			//pcl::PassThrough<pcl::PointXYZ> pass;
			//pass.setInputCloud(cloud);
			//pass.setFilterFieldName("z");
			//pass.setFilterLimits(0.0, 1.0);
			//pass.filter(*cloud_filtered);
			viewer.showCloud(cloud);			//viewer.showCloud(cloud_filtered);

		}
	}

	void run()
	{
		pcl::Grabber* grabber = new pcl::OpenNIGrabber();

		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		boost::function<void (pcl::visualization::PCLVisualizer&)> fVOnOff =
			boost::bind(&SimpleOpenNIViewer::viewerOneOff, this, _1);

		boost::function<void (pcl::visualization::PCLVisualizer&)> fVIteration =
			boost::bind(&SimpleOpenNIViewer::viewerIteration, this, _1);
	
		
		viewer.runOnVisualizationThreadOnce(fVOnOff);
		viewer.runOnVisualizationThread(fVIteration);

		grabber->registerCallback(f);
		
		grabber->start();

		


		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		grabber->stop();
	}

	pcl::visualization::CloudViewer viewer;
};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}