#include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>
 #include <pcl/filters/passthrough.h>
 //#ifdef _WIN32
// # define sleep(x) Sleep((x)*1000)
// #endif

 class SimpleOpenNIViewer
 {
   public:
	
     SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer") {}

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