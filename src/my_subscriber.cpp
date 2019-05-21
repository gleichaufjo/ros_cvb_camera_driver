#include <iostream>
#include <cvb/cvb/device_factory.hpp>
#include <cvb/cvb/utilities/system_info.hpp>
#include <cvb/cvb/driver/stream.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void callback(const sensor_msgs::ImageConstPtr& image)
{
	std::cout << "received an image" << std::endl;
};


sensor_msgs::Image toImageMsg(const Cvb::ImagePtr& cvbImg)
{
  // sensor msg
  sensor_msgs::Image rosImg;
  rosImg.header.frame_id = "camera";
  rosImg.height = cvbImg->Height();
  rosImg.width = cvbImg->Width();
  rosImg.encoding = sensor_msgs::image_encodings::RGB8; // workaround, needs proper encoding values.
  rosImg.is_bigendian = false; // always depends on the pfnc dataformat

  auto dataAccess = cvbImg->Plane(0).LinearAccess();

  rosImg.step = dataAccess.YInc();

//  size_t size = dataAccess.YInc();
  size_t size = rosImg.width * 3 * rosImg.height;
  unsigned char * start = reinterpret_cast<unsigned char*>(dataAccess.BasePtr());
  rosImg.data = std::vector<unsigned char>(start, start + size);
  return rosImg;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "something");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

    image_transport::Subscriber sub = it.subscribe("camera/image", 1, callback);

  try
  {
std::cout << " Try " << std::endl; 
    auto stream = Cvb::DeviceFactory::Open(Cvb::ExpandPath(CVB_LIT("%CVB%/drivers/GenICam.vin")))->Stream();
    stream->Start();

   sensor_msgs::Image image;
   cv_bridge::CvImagePtr frame;
   cv::Mat Input;
   cv::Mat cropped;
   cv::Mat Output;
   while(ros::ok())
   {
      // wait for an image with a timeout of 10 seconds
      auto waitResult = stream->WaitFor(std::chrono::milliseconds(10000));
      if (waitResult.Status == Cvb::WaitStatus::Ok)
      {
    	image = toImageMsg(waitResult.Image);
    	frame = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
    	Input = frame->image;
    	// remove black stripes from fish eye camera image
    	cv::Rect cropArea(195, 0 , 1546, 1216);
    	cropped = Input(cropArea);
    	cv::resize(cropped, Output, cv::Size(), 1, 1);
    	cv_bridge::CvImage resizeRos;
    	resizeRos.encoding = "rgb8";
    	resizeRos.image = Output;
    	sensor_msgs::ImagePtr imagePtr = resizeRos.toImageMsg();
    	imagePtr->header.stamp = ros::Time::now();
    	pub.publish(imagePtr);
//        pub.publish(toImageMsg(waitResult.Image));
        ros::spinOnce();
      }
      else
      {
        throw std::runtime_error(std::string("cvb acq error ") + std::to_string(static_cast<int>(waitResult.Status)));
      }
    }
    // synchronously stop the stream
    stream->Stop();
  }
  catch (const std::exception& error)
  {
    std::cout << error.what() << std::endl;
  }
}
