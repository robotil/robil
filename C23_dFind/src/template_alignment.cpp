#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include<C23_dFind/perceptionTransform.h>
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include <ros/package.h>
using namespace C0_RobilTask;
//#include <move_hand/matrix.h>
#define _USE_MATH_DEFINES
ros::ServiceServer ch;
	  Eigen::Matrix3f rotation;
Eigen::Vector3f translation;
//---------------------------------------------------------
class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      std::cout<< pcl::io::loadPCDFile (pcd_file, *xyz_)<<endl;
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }
public:
  PointCloud::Ptr xyz_;
  private:
    // Point cloud data
  
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }
      std::cout<<best_template<<std::endl;
      std::cout<<results.size()<<std::endl;
      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

std::vector<FeatureCloud>* object_templates;



// Align a collection of object templates to a sample point cloud

class dFind
{
public:

   ros::Subscriber cloud_sub;
	// Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;//(new pcl::PointCloud<pcl::PointXYZ>);
  dFind(ros::NodeHandle);
 FeatureCloud template_cloud;
  bool perceive(C23_dFind::perceptionTransform::Request &req,C23_dFind::perceptionTransform::Response &res);
private:
  float best_view;
 TemplateAlignment::Result best_alignment;
  void  cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
ros::NodeHandle nh;
  // percieve()
};
void dFind::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	// Load the target cloud PCD file

  pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud);
  // std::cout<<"lkjsdlkfsdfkjsdf;kjsd\n";
         std::cout<<template_cloud.xyz_<<endl;
	
	static tf::StampedTransform transform;

 static tf::TransformListener listener;
	tf::Transform trans;
		try{
		  	  ros::Time now =ros::Time::now();
		   listener.waitForTransform("pelvis","head",now,ros::Duration(3,0));
		  listener.lookupTransform("pelvis","head",now, transform);
		}
		catch (tf::TransformException ex){
		  //ROS_ERROR("cv_bridge exception: %s", ex.what());
		  //have a problem that this doesn't work the first time so used continue as a workaround
		  //  best_alignment.fitness_score=1;
		  //  continue;
		}
	
	trans.setOrigin(tf::Vector3(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ()));
		trans.setRotation(tf::Quaternion(transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ(),transform.getRotation().getW()));
		tf::Transform trans3;
		trans3.setOrigin(tf::Vector3(0.0,-0.002, 0.035 ));
		trans3.setRotation(tf::Quaternion(-M_PI/2,M_PI,M_PI/2));
		Eigen::Matrix4f sensorToHead,headTopelvis;
		pcl_ros::transformAsMatrix(trans3, sensorToHead);
		pcl_ros::transformAsMatrix(trans, headTopelvis);
		// transform pointcloud from sensor frame to fixed robot frame
  // Preprocess the cloud by...
	  // ...removing distant points
	  const float depth_limit = 2.5;
	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cloud);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (0, depth_limit);
	  pass.filter (*cloud);

	  // ... and downsampling the point cloud
	  const float voxel_grid_size = 0.005f;
	  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	  vox_grid.setInputCloud (cloud);
	  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);

	  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
	  vox_grid.filter (*tempCloud);
	  cloud = tempCloud;

		pcl::transformPointCloud(*cloud, *cloud, sensorToHead);
		pcl::transformPointCloud(*cloud,*cloud, headTopelvis);
	  // Assign to the target FeatureCloud
	  FeatureCloud target_cloud;
	  target_cloud.setInputCloud (cloud);

	  // Set the TemplateAlignment inputs
	  TemplateAlignment template_align;
	  for (size_t i = 0; i < object_templates->size(); ++i)
	  {

	    template_align.addTemplateCloud (object_templates->at(i));
	  }	std::cout<<"exist\n";
	  template_align.setTargetCloud (target_cloud);

	
	  int best_index = template_align.findBestAlignment (best_alignment);
	  const FeatureCloud &best_template = object_templates->at(best_index);

	  // Print the alignment fitness score (values less than 0.00002 are ideal we use less than 0.000035)

	  printf ("Best fitness score: %f\n", best_alignment.fitness_score);
	  if(best_alignment.fitness_score<0.00003)
	    {
	      rotation = best_alignment.final_transformation.block<3,3>(0, 0);
	      translation = best_alignment.final_transformation.block<3,1>(0, 3);
	      best_view=best_alignment.fitness_score;
	    }
// Save the aligned template for visualization
	  //pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	  //pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
	  //pcl::transformPointCloud(transformed_cloud, transformed_cloud, sensorToHead);
	  //	pcl::transformPointCloud(transformed_cloud,transformed_cloud, headTopelvis);


		//pcl::io::savePCDFile (fName, transformed_cloud,false);
// sprintf(fName,"%d.%dscene.pcd",msg->header.stamp.sec,msg->header.stamp.nsec);
// pcl::io::savePCDFileBinary (fName, *cloud);
}

dFind::dFind (ros::NodeHandle _nh)
{
  nh =_nh;
  best_view=1;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudtemp(new pcl::PointCloud<pcl::PointXYZ>);
  cloud=cloudtemp;
  /*if (argc < 2)
  {
    printf ("No target PCD file given!\n");
    return (-1);
  }*/

 
  ch=nh.advertiseService<C23_dFind::perceptionTransform::Request,C23_dFind::perceptionTransform::Response>("perceptionTransform",boost::bind(&dFind::perceive,this,_1,_2));

 cloud_sub=nh.subscribe("/multisense_sl/camera/points2",1,&dFind::cloudCallback,this);

  // Load the object templates specified in the object_templates.txt file
 char basePath[1000],templateFiles[1000];

            sprintf(basePath,"%s/mod/%c",ros::package::getPath("C23_dFind").c_str(),'\0');

            sprintf(templateFiles,"%ste.txt%c",basePath,'\0');
	    std::cout<<templateFiles<<endl;
	    // std::cout<<buffer<<endl;
	    
  std::ifstream input_stream (templateFiles);
  object_templates=new std::vector<FeatureCloud>;
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;
    
    char buffer[1000];
    sprintf(buffer,"%s%s",basePath,pcd_filename.c_str());
 std::cout<<buffer<<"\n";
 string tmp(buffer);
    template_cloud.loadInputCloud (tmp);
    object_templates->push_back (template_cloud);
  }
  input_stream.close ();
  


}
 class dFindServer:public RobilTask
{
 protected:
   string _name;
   dFind *_finder;
   ros::NodeHandle nh;

 public:
  dFindServer(dFind &finder):
       RobilTask("/dFind"),
       _finder(&finder),
       _name("/dFind")
  {
    ros::spin();
  }
  TaskResult task(const string& name, const string& uid, Arguments& args)
   {
     if(isPreempt()){
       return TaskResult::Preempted();
   }
      while(true)
	sleep(1000);
   }
 };

bool dFind::perceive(C23_dFind::perceptionTransform::Request &req,C23_dFind::perceptionTransform::Response &res)
{

  // Find the best template alignment
	 

	  // Print the rotation matrix and translation vector


	  printf ("best view%f", best_view);

	  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	  printf ("\n");
	  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
	  //get a new point cloud and try again
	  //ros::spinOnce();
 
//while(best_alignment.fitness_score>0.000037);	


  res.transMat[0]=rotation(0,0);
		res.transMat[1]=rotation(0,1);
		res.transMat[2]=rotation(0,2);
		res.transMat[3]=translation(0);
		res.transMat[4]=rotation(1,0);
		res.transMat[5]=rotation(1,1);
		res.transMat[6]=rotation(1,2);
		res.transMat[7]=translation(1);
		res.transMat[8]=rotation(2,0);
		res.transMat[9]=rotation(2,1);
		res.transMat[10]=rotation(2,2);
		res.transMat[11]=translation(2);
		res.transMat[12]=0;
		res.transMat[13]=0;
		res.transMat[14]=0;
		res.transMat[15]=1;
		if(best_alignment.fitness_score<0.5)
		  return true;
		else 
		  return false;
}

int main(int argc, char **argv)
{  ros::init(argc, argv,"dFind");

ros::NodeHandle nh; 
 dFind d(nh);
 // dtest=&d;
  dFindServer task1(d);
  
  return 0;
}
