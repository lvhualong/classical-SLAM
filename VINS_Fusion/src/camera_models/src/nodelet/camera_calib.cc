#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <camodocal/chessboard/Chessboard.h>
#include <camodocal/calib/CameraCalibration.h>
#include <camodocal/gpl/gpl.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
namespace camera_models {


class CameraCalibration : public nodelet::Nodelet {
  public:
    CameraCalibration():_modelType(camodocal::Camera::KANNALA_BRANDT), _camera_model("kannala-brandt"), _boardSize(11, 8), _square_size(0.03), _use_opencv(false), _calibrate_nr(50), _mono_camera_file("."), _do_rectify(false){
      data_params.reserve(50);
      param_range = cv::Mat(std::vector<float>{0.5, 0.5, 0.4, 0.5}, true);
      _process = cv::Mat(std::vector<float>{0., 0., 0., 0.}, true);
    }
    virtual ~CameraCalibration() {}
    virtual void onInit() {
      ros::NodeHandle &nh = getNodeHandle();
      ros::NodeHandle &pnh = getPrivateNodeHandle();
      
      int boradsize_width = 11;
      int boradsize_height = 8;

      pnh.param("boradsize_width", boradsize_width, boradsize_width);
      pnh.param("boradsize_height", boradsize_height, boradsize_height);
      pnh.param("square_size", _square_size, _square_size);
      pnh.param("camera_model", _camera_model, _camera_model);
      pnh.param("use_opencv", _use_opencv, _use_opencv);
      pnh.param("calibrate_nr", _calibrate_nr, _calibrate_nr);
      pnh.param("camera_file", _mono_camera_file, _mono_camera_file);
      // mono image topic
      std::string mono_topic = "mono_topic_is_not_correct";
      pnh.param("mono_topic", mono_topic, mono_topic);

      _boardSize = cv::Size(boradsize_width, boradsize_height);

      {
        NODELET_INFO_STREAM("camera_model: "<< _camera_model);
        NODELET_INFO_STREAM("boradsize_width: " << boradsize_width);
        NODELET_INFO_STREAM("boradsize_height: " << boradsize_height);
        NODELET_INFO_STREAM("square_size: " << _square_size);
        NODELET_INFO_STREAM("use_opencv: " << _use_opencv);
        NODELET_INFO_STREAM("calibrate_nr: " << _calibrate_nr);
	NODELET_INFO_STREAM("mono_topic: " << mono_topic);
	NODELET_INFO_STREAM("mono_camera_file: " << _mono_camera_file);
      }

      //        check camera model, default:KANNALA_BRANDT
      if (boost::iequals(_camera_model, "kannala-brandt")) {
        _modelType = camodocal::Camera::KANNALA_BRANDT;
        NODELET_INFO_STREAM("# INFO: Camera model: " << _camera_model);
      } else if (boost::iequals(_camera_model, "mei")) {
        _modelType = camodocal::Camera::MEI;
        NODELET_INFO_STREAM("# INFO: Camera model: " << _camera_model);
      } else if (boost::iequals(_camera_model, "pinhole")) {
        _modelType = camodocal::Camera::PINHOLE;
        NODELET_INFO_STREAM("# INFO: Camera model: " << _camera_model);
      } else if (boost::iequals(_camera_model, "scaramuzza")) {
        _modelType = camodocal::Camera::SCARAMUZZA;
        NODELET_INFO_STREAM("# INFO: Camera model: " << _camera_model);
      } else {
        NODELET_ERROR_STREAM("# ERROR: Unknown camera model: " << _camera_model);
        ros::shutdown();
      }

      image_transport::ImageTransport it(nh);
      _mono_sub = it.subscribe(mono_topic, 1, &CameraCalibration::processMono, this);
    }

  private:

  void processMono(const sensor_msgs::ImageConstPtr &img_ptr)
  {
    cv::Mat mono_img = cv_bridge::toCvCopy(img_ptr)->image; 
    if (0 == _mono_calibration_ptr.get())
    {
       _mono_calibration_ptr.reset(new camodocal::CameraCalibration(_modelType, _camera_model, mono_img.size(), _boardSize, _square_size));
       _mono_calibration_ptr->setVerbose(true);
    }
    camodocal::Chessboard mono_chessboard(_boardSize, mono_img);
    mono_chessboard.findCorners(_use_opencv); 

    std::stringstream ss;
    ss << "p_x: " << _process.at<float>(0) << ", p_y: " << _process.at<float>(1)
       <<", size: " << _process.at<float>(2) << ", skew: " << _process.at<float>(3);
    if (mono_chessboard.cornersFound() && !_do_rectify)
    {
      mono_img = mono_chessboard.getSketch();
      if (is_good_samples(mono_chessboard.getCorners(), cv::Size(mono_img.cols, mono_img.rows)))
      {
        _mono_calibration_ptr->addChessboardData(mono_chessboard.getCorners()); 
        NODELET_INFO_STREAM("# INFO: Had selected  " << _mono_calibration_ptr->sampleCount()
                                   << " imgs");
	if (_mono_calibration_ptr->sampleCount() >= _calibrate_nr || compute_goodenough())
	{
          _do_rectify = true;
	  NODELET_INFO_STREAM("start do rectify...");
	  double startTime = camodocal::timeInSeconds();
	  if (!_mono_calibration_ptr->calibrate())
	  {
            NODELET_INFO_STREAM("calibrate failture, restart calibrate...");
            _mono_calibration_ptr->clear();
            _calibrate_nr = 0;
            _do_rectify = false;
	    data_params.clear();
            return;
	  }
          NODELET_INFO_STREAM("complete rectify.");
	  _mono_calibration_ptr->writeParams(_mono_camera_file);
          NODELET_INFO_STREAM(
              "# INFO: Calibration took a total time of "
              << std::fixed << std::setprecision(3)
              << camodocal::timeInSeconds() - startTime << " sec.");
          NODELET_INFO_STREAM(
              "# INFO: Wrote calibration files to"
              << _mono_camera_file);
	}
      }
    }
    cv::putText(mono_img, ss.str(), cv::Point(mono_img.cols*0.1, mono_img.rows-30), 1, 1, cv::Scalar(255, 0, 0), 2);	
    try
    {
      cv::imshow("mono_img", mono_img);
    }
    catch (...)
    {
      NODELET_INFO_STREAM("imshow core dumped, plealse reference https://stackoverflow.com/questions/48333362/opencv-assert-false-in-file-qasciikey-cpp");
    }
    cv::waitKey(1);
  }

    bool is_good_samples(const std::vector<cv::Point2f>& corners, const cv::Size& img_size)
    {
      bool is_good = true;

      const int &width = img_size.width; 
      const int &height = img_size.height;
      const int xdim = _boardSize.width;
      const int ydim = _boardSize.height;

      // compute the four corners of the board as a whole, as (up_left, up_right, down_right, down_left).
      const cv::Point2f &up_left = corners[0];
      const cv::Point2f &up_right = corners[xdim-1];
      const cv::Point2f &down_left = corners[xdim*(ydim-1)+0];
      const cv::Point2f &down_right = corners[xdim*ydim-1];
      //ROS_INFO("up_left:[ %f, %f], up_right:[%f, %f], down_left:[%f, %f], down_right:[%f, %f]", up_left.x, up_left.y, up_right.x, up_right.y, down_left.x, down_left.y, down_right.x, down_right.y);
      // The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
      //    |p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.
      cv::Point2f a = up_right - up_left;
      cv::Point2f b = down_right - up_right;
      cv::Point2f c = down_left - down_right;
      cv::Point2f p = b + c;
      cv::Point2f q = a + b;
      float area = (float)std::abs(p.cross(q))/2;
      float border = std::sqrt(area);
      //NODELET_INFO_STREAM("area: " << area);
      //NODELET_INFO_STREAM("border: " << border);
      //NODELET_INFO_STREAM("w, h: " << width << ", " << height);
      //we "shrink" the image all around by approx. half the board size.Otherwise large boards are penalized because you can't get much X/Y variation.
      float Xs = 0, Ys = 0;
      for (auto& p : corners)
      {
        Xs += p.x;
	Ys += p.y;
      }
      Xs /= corners.size();
      Ys /= corners.size();

      float p_x = std::min(1.0f, std::max(0.0f, (Xs - border/2)/(width - border)));
      float p_y = std::min(1.0f, std::max(0.0f, (Ys - border/2)/(width - border)));
      float p_size = std::sqrt(area/(width*height));
      float skew = std::min(1.0f, static_cast<float>(2. * std::abs(3.1415926/2. - angle(up_left, up_right, down_right))));
      std::vector<float> params{p_x, p_y, p_size, skew};
      if (data_params.empty())
      {
        data_params.insert(data_params.end(), params);
        return is_good;
      }
      cv::Mat cv_params = cv::Mat(params, false);
      double min_d = DBL_MAX;
      cv::Mat abs_diff; 
      for (auto& v_params : data_params){
        cv::Mat cv_v_params = cv::Mat(v_params, false); 
        double d = cv::norm(cv_params, cv_v_params, cv::NORM_INF);
	if (d < min_d)
	  min_d = d;
      } 
     // ROS_INFO("min_d: %f\n", min_d);
     // ROS_INFO("p_x:%f, p_y:%f, p_size:%f, skew:%f\n", p_x, p_y, p_size, skew);
      if (min_d > 0.2)
      {
        data_params.insert(data_params.end(), params);
	return is_good;
      }
      else
      {
        return !is_good;
      }
    }

    std::size_t get_data_params() const
    {
      return data_params.size();
    }

    float angle(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c)
    {
       cv::Point2f ab = a - b;
       cv::Point2f cb = c - b;
       return std::acos(ab.dot(cb)/(cv::norm(ab)*cv::norm(cb)));
    }

    bool compute_goodenough()
    {
      cv::Mat min_params = cv::Mat(data_params[0], true);   
      cv::Mat max_params = cv::Mat(data_params[0], true);
      for (auto& elem: data_params)
      {
        cv::Mat cv_elem = cv::Mat(elem, false);
	min_params = cv::min(min_params, cv_elem);
	max_params = cv::max(max_params, cv_elem);
      }
      min_params.rowRange(2, 4).setTo(0);
      cv::Mat process = (max_params - min_params)/param_range;
      bool process_flag = true;
      for (int i  = 0; i < process.rows; i++)
        for (int j = 0; j < process.cols; j++)
	{
	  float& val = process.at<float>(i, j);
	  val = std::min(val, 1.0f);
	  if (std::abs(val-1.0f) >= 0.001)
	  {
	    process_flag = false;
	  }
	}
      process.copyTo(_process);
      return process_flag;
    }

    bool is_good_enough(const std::vector<cv::Point2f>& corners, const cv::Size& img_size)
    {
      return is_good_samples(corners, img_size) && compute_goodenough();
    }

  private:
    image_transport::Subscriber _mono_sub;
    boost::shared_ptr<camodocal::CameraCalibration> _mono_calibration_ptr;
    camodocal::Camera::ModelType _modelType;
    std::string _camera_model;
    cv::Size _boardSize;
    double _square_size;
    bool _use_opencv;
    int _calibrate_nr;
    std::string _mono_camera_file;
    bool _do_rectify;
    std::vector<std::vector<float>> data_params;
    cv::Mat param_range;
    cv::Mat _process;
};
PLUGINLIB_EXPORT_CLASS(
    camera_models::CameraCalibration, nodelet::Nodelet);
}
