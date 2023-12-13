#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"
#include "config.h"
#include "utilities.h"
#include "selectScanPoints.h"
#include "calcCamPose.h"
#include "LaseCamCalCeres.h"
#include <iomanip>

double timestamp_tosave = 1635527677.854322448;
std::ofstream outputfile_scan;
std::ofstream outputfile_tag;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  std::cout << name << std::endl;
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LaserCamCal");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  std::string config_file;
  config_file = readParam<std::string>(pnh, "config_file");
  readParameters(config_file);

  rosbag::Bag bag_input;
  bag_input.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(scan_topic_name);
  topics.push_back(img_topic_name);
  rosbag::View views(bag_input, rosbag::TopicQuery(topics));

  // Load apritag pose
  std::vector<CamPose> tagpose;
  LoadCamPoseFromTxt(savePath + "apriltag_pose.txt", tagpose);
  std::cout << "Load apriltag pose size: " << tagpose.size() << std::endl;

  if (tagpose.size() < 10)
  {
    std::cout << "apriltag pose less than 10." << std::endl;
    return 0;
  }

  /// Select keyframe to calibrating
  // std::vector<CamPose> sparseTagpose;
  // CamPose older = tagpose.at(0);
  // sparseTagpose.push_back(older);
  // double dist_min = 0.20; // 10cm
  // double theta_min = 3.1415926 * 10 / 180.;
  // for (int j = 1; j < tagpose.size(); ++j)
  // {
  //   CamPose newer = tagpose.at(j);
  //   double dist = (older.twc - newer.twc).norm();
  //   double theta = 2 * std::acos(((older.qwc.inverse() * newer.qwc)).w());
  //   if ((dist > dist_min) || (fabs(theta) > theta_min))
  //   {
  //     older = newer;
  //     sparseTagpose.push_back(older);
  //   }
  // }
  // tagpose = sparseTagpose;

  // 准备标定数据
  std::vector<Oberserve> obs;
  // 处理激光数据
  int ii_cnt = 10;

  for (rosbag::MessageInstance const m : views)
  {
    if (m.getTopic() == scan_topic_name)
    {
      sensor_msgs::LaserScan::Ptr scan = m.instantiate<sensor_msgs::LaserScan>();
      std::vector<Eigen::Vector3d> Points;
      TranScanToPoints(*scan, Points);

      //      ii_cnt++;
      //      if(ii_cnt % 20 != 0) continue;

      double timestamp = scan->header.stamp.toSec();

      // save the "timestamp_tosave" frame of scan
      if (fabs(timestamp_tosave - timestamp) <= 0.01)
      {
        outputfile_scan.open("/home/lyd/output/lv_calibr/pointcloud_scan.txt");
        std::cout<<"find the frame: "<<std::fixed<<std::setprecision(10)<<timestamp<<std::endl;
        for (size_t i = 0; i < Points.size(); ++i)
        {
          outputfile_scan << i << " " << Points[i][0] << " " << Points[i][1] << " " << Points[i][2] << std::endl;
        }
        outputfile_scan.close();
      }

      std::vector<Eigen::Vector3d> points;

      points = AutoGetLinePts(Points);

      // 检测到了直线
      if (points.size() > 0)
      {
        // 在 camera 里找时间戳最近的一个 pose
        double min_dt = 10000;
        CamPose colsetTagPose;
        for (int i = 0; i < tagpose.size(); ++i)
        {
          CamPose tmp = tagpose.at(i);
          double t = fabs(tmp.timestamp - timestamp);
          if (t < min_dt)
          {
            min_dt = t;
            colsetTagPose = tmp;
          }
        }
              if (fabs(timestamp_tosave - colsetTagPose.timestamp) <= 0.05)
      {
        outputfile_tag.open("/home/lyd/output/lv_calibr/pointcloud_tag.poly");
        std::cout<<std::fixed<<std::setprecision(10)<<"find the frame pose: "<<colsetTagPose.timestamp<<std::endl;
        //outputfile_tag<<std::fixed<<std::setprecision(10)<<colsetTagPose.timestamp<<std::endl;
        Eigen::Vector3d orig(0.0265+0.0165,0.0265+0.0165,0);
            Eigen::Vector3d p1m( 0, 0, 0);
            Eigen::Vector3d p2m( 0.5, 0, 0);
            Eigen::Vector3d p3m( 0., 0.5, 0);
            Eigen::Vector3d p4m( 0.5, 0.5, 0);
            p1m -= orig;
            p2m -= orig;
            p3m -= orig;
            p4m -= orig;
            // 旋转到相机坐标系
            Eigen::Vector3d p1c = colsetTagPose.qwc.toRotationMatrix().inverse() * (p1m -  colsetTagPose.twc);
            Eigen::Vector3d p2c = colsetTagPose.qwc.toRotationMatrix().inverse() * (p2m -  colsetTagPose.twc);
            Eigen::Vector3d p3c = colsetTagPose.qwc.toRotationMatrix().inverse() * (p3m -  colsetTagPose.twc);
            Eigen::Vector3d p4c = colsetTagPose.qwc.toRotationMatrix().inverse() * (p4m -  colsetTagPose.twc);
            outputfile_tag<<p1c[0]<<" "<<p1c[1]<<" "<<p1c[2]<<std::endl;
            outputfile_tag<<p2c[0]<<" "<<p2c[1]<<" "<<p2c[2]<<std::endl;
            outputfile_tag<<p4c[0]<<" "<<p4c[1]<<" "<<p4c[2]<<std::endl;
            outputfile_tag<<p3c[0]<<" "<<p3c[1]<<" "<<p3c[2]<<std::endl;
            outputfile_tag<<p1c[0]<<" "<<p1c[1]<<" "<<p1c[2]<<std::endl;
            // outputfile_tag<<std::fixed<<std::setprecision(10)<<"1 "<<p1c[0]<<" "<<p1c[1]<<" "<<p1c[2]<<std::endl;
            // outputfile_tag<<std::fixed<<std::setprecision(10)<<"2 "<<p2c[0]<<" "<<p2c[1]<<" "<<p2c[2]<<std::endl;
            // outputfile_tag<<std::fixed<<std::setprecision(10)<<"3 "<<p3c[0]<<" "<<p3c[1]<<" "<<p3c[2]<<std::endl;
            // outputfile_tag<<std::fixed<<std::setprecision(10)<<"4 "<<p4c[0]<<" "<<p4c[1]<<" "<<p4c[2]<<std::endl;

        outputfile_tag.close();
      }

        if (min_dt < 0.02) // 20ms
        {

          //          std::cout << "scan and tag time: "<<std::fixed<<std::setprecision(18)
          //                    <<timestamp<<" "<<colsetTagPose.timestamp<<std::endl;
          /

          Eigen::Vector2d line;
          LineFittingCeres(points, line);
          std::vector<Eigen::Vector3d> points_on_line;

          // 激光所在直线不能垂直于某个轴
          double x_start(points.begin()->x()), x_end(points.end()->x());
          double y_start(points.begin()->y()), y_end(points.end()->y());
          if (fabs(x_end - x_start) > fabs(y_end - y_start))
          {
            y_start = -(x_start * line(0) + 1) / line(1);
            y_end = -(x_end * line(0) + 1) / line(1);
          }
          else // 可能垂直于 x 轴，采用y值来计算 x
          {
            x_start = -(y_start * line(1) + 1) / line(0);
            x_end = -(y_end * line(1) + 1) / line(0);
          }

          points_on_line.push_back(Eigen::Vector3d(x_start, y_start, 0));
          points_on_line.push_back(Eigen::Vector3d(x_end, y_end, 0));

          Oberserve ob;
          ob.tagPose_Qca = colsetTagPose.qwc.inverse();
          ob.tagPose_tca = -ob.tagPose_Qca.toRotationMatrix() * colsetTagPose.twc;
          ob.points = points;
          ob.points_on_line = points_on_line;
          obs.push_back(ob);
        }
      }
    }
  }

  if (obs.size() < 5)
  {
    std::cout << "Valid Calibra Data Less" << std::endl;
    bag_input.close();
    return 0;
  }
  std::cout << "obs size: " << obs.size() << std::endl;

  // Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
  // CamLaserCalClosedSolution(obs,Tlc_initial);

  // hand computation for initialization --liyida 2021.10.27
  Eigen::Matrix4d Tlc_initial;
  Tlc_initial <<
  // -1.3124710880100743e-01, 9.3463583566847663e-01, 3.3049969911584742e-01, -5.5852932629091622e-03,
  //     -1.5956766335713890e-01, 3.0911958766939568e-01, -9.3754106114337521e-01, -1.2906010084882734e-02,
  //     -9.7842340387079885e-01, -1.7578661838544923e-01, 1.0856660423209767e-01, 7.1673734951464345e-02,
  //     0., 0., 0., 1.;
  // best parameters
    -2.0534250949794348e-02, 9.3632715066297922e-01,3.5052790398094774e-01, -6.2470572003489024e-02,
    -1.6987064830015094e-01, 3.4223670240822079e-01,-9.2413094438549825e-01, -2.9679393942842452e-02,
    -9.8525240795646707e-01, -7.8520739019019389e-02,1.5202692577267340e-01, -10.1246090843195932e-02,
    0., 0., 0., 1.;
  theoretical parameters
  //  0., 0.9396,0.342020, -0.06,
  //  0., 0.342020,-0.9396, -0.5,
  //  -1.0, 0.,0., -0.07,
  //  0., 0., 0., 1. ;

  Eigen::Matrix4d Tcl = Tlc_initial.inverse();
  CamLaserCalibration(obs, Tcl, false, true);
  // CamLaserCalibration(obs,Tcl, true);

  std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n"
            << std::endl;
  Eigen::Matrix4d Tlc = Tcl.inverse();
  std::cout << Tlc << std::endl;

  std::cout << "\n----- Transform from Camera to Laser, euler angles and translations are: -----\n"
            << std::endl;
  Eigen::Matrix3d Rlc(Tlc.block(0, 0, 3, 3));
  Eigen::Vector3d tlc(Tlc.block(0, 3, 3, 1));
  EulerAngles rpy = ToEulerAngles(Eigen::Quaterniond(Rlc));
  std::cout << "   roll(rad): " << rpy.roll << " pitch(rad): " << rpy.pitch << " yaw(rad): " << rpy.yaw << "\n"
            << "or roll(deg): " << rpy.roll * 180. / M_PI << " pitch(deg): " << rpy.pitch * 180. / M_PI << " yaw(deg): " << rpy.yaw * 180. / M_PI << "\n"
            << "       tx(m): " << tlc.x() << "  ty(m): " << tlc.y() << "   tz(m): " << tlc.z() << std::endl;

  // save to yaml file
  cv::Mat cvTlc;
  cv::eigen2cv(Tlc, cvTlc);
  std::string fn = savePath + "result.yaml";
  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  fs << "extrinsicTlc" << cvTlc;
  cv::Mat cvrpy;
  cv::eigen2cv(Eigen::Vector3d(rpy.roll, rpy.pitch, rpy.yaw), cvrpy);
  cv::Mat cvtlc;
  cv::eigen2cv(tlc, cvtlc);
  fs << "RollPitchYaw" << cvrpy;
  fs << "txtytz" << cvtlc;
  fs.release();

  std::cout << "\n Result file : " << fn << std::endl;
  std::cout << "\n-------------- Calibration Code End --------------\n"
            << std::endl;

  ros::spin();
  
  outputfile_tag.close();
}

