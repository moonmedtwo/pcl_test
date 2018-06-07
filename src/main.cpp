#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <iostream>

#define FILTERED

#ifdef FILTERED
#include <pcl/filters/passthrough.h>
#endif

using namespace std;
using namespace pcl;

double filter_z = 10.0;

PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), noColor(false);                                // Program control.

void
printUsage(const char* programName)
{
	cout << "Usage: " << programName << " [options]"
		 << endl
		 << endl
		 << "Options:\n"
		 << endl
		 << "\t<none>     start capturing from an OpenNI device.\n"
		 << "\t-v FILE    visualize the given .pcd file.\n"
		 << "\tspace	  save cloud.\n"
		 << "\t--z 	  filter_z.\n"
		 << "\t-h         shows this help.\n";
}

// This function is called every time the device has new data.
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
        
#ifdef FILTERED
        pcl::PointCloud<PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::PassThrough<pcl::PointXYZRGBA> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0,filter_z);
        pass.filter(*cloud_filtered);

	if (! viewer->wasStopped())
		viewer->showCloud(cloud_filtered);

	if (saveCloud)
	{
		stringstream stream;
		stream << "./SingleCan/singleCan_" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud_filtered, true) == 0)
		{
			filesSaved++;
			cout << "saved " << filename << "." << endl;
		}
		else PCL_ERROR("problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}

#else

	if (! viewer->wasStopped())
		viewer->showCloud(cloud);

	if (saveCloud)
	{
		stringstream stream;
		stream << "./SingleCan/singleCan_" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "saved " << filename << "." << endl;
		}
		else PCL_ERROR("problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}
#endif
}

// For detecting when SPACE is pressed.
void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
					  void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<visualization::CloudViewer> v
	(new visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);

	return (v);
}

int
main(int argc, char** argv)
{
	if (console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return -1;
	}

	bool justVisualize(false);
	string filename;
	if (console::find_argument(argc, argv, "-v") >= 0)
	{
		// if (argc != 3)
		// {
		// 	printUsage(argv[0]);
		// 	return -1;
		// }

		filename = argv[2];
		justVisualize = true;
	}
	else if (argc != 1)
	{
		// printUsage(argv[0]);
		// return -1;
	}
	pcl::console::parse_argument (argc, argv, "--z", filter_z);

	// First mode, open and show a cloud from disk.
	if (justVisualize)
	{
		// Try with color information...
		try
		{
			io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);
		}
		catch (PCLException e1)
		{
			try
			{
				// ...and if it fails, fall back to just depth.
				io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
			}
			catch (PCLException e2)
			{
				return -1;
			}

			noColor = true;
		}

		cout << "Loaded " << filename << "." << endl;
		if (noColor)
			cout << "This cloud has no RGBA color information present." << endl;
		else cout << "This cloud has RGBA color information present." << endl;
	}
	// Second mode, start fetching and displaying frames from the device.
	else
	{
		openniGrabber = new OpenNIGrabber();
		if (openniGrabber == 0)
			return -1;
		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&grabberCallback, _1);
		openniGrabber->registerCallback(f);
	}

	viewer = createViewer();

	if (justVisualize)
	{
		if (noColor)
			viewer->showCloud(fallbackCloud);
		else viewer->showCloud(cloudptr);
	}
	else openniGrabber->start();

	// Main loop.
	while (! viewer->wasStopped())
		boost::this_thread::sleep(boost::posix_time::seconds(1));

	if (! justVisualize)
		openniGrabber->stop();
}
