#include <QMutexLocker>

#include "ROSThread.h"

using namespace std;

ROSThread::ROSThread(QObject *parent, QMutex *th_mutex) :
    QThread(parent), mutex_(th_mutex)
{
  processed_stamp_ = 0;
  play_rate_ = 1.0;
  loop_flag_ = false;
  stop_skip_flag_ = true;
  search_bound_ = 10;
  reset_process_stamp_flag_ = false;
  auto_start_flag_ = true;
  stamp_show_count_ = 0;
  imu_data_version_ = 0;
  prev_clock_stamp_ = 0;
}

ROSThread::~ROSThread()
{
  data_stamp_thread_.active_ = false;
  pose_thread_.active_ = false;
  imu_thread_.active_ = false;
  livox_thread_.active_ = false;
  cam_thread_.active_ = false;

  usleep(100000);

  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();

  pose_thread_.cv_.notify_all();
  if(pose_thread_.thread_.joinable()) pose_thread_.thread_.join();
  
  imu_thread_.cv_.notify_all();
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();

  livox_thread_.cv_.notify_all();
  if(livox_thread_.thread_.joinable()) livox_thread_.thread_.join();

  cam_thread_.cv_.notify_all();
  if(cam_thread_.thread_.joinable()) cam_thread_.thread_.join();

}

void ROSThread::ros_initialize(ros::NodeHandle &n)
{
  nh_ = n;

  pre_timer_stamp_ = ros::Time::now().toNSec();
  timer_ = nh_.createTimer(ros::Duration(0.0001), boost::bind(&ROSThread::TimerCallback, this, _1));

  start_sub_  = nh_.subscribe<std_msgs::Bool>("/file_player_start", 1, boost::bind(&ROSThread::FilePlayerStart, this, _1));
  stop_sub_    = nh_.subscribe<std_msgs::Bool>("/file_player_stop", 1, boost::bind(&ROSThread::FilePlayerStop, this, _1));

  pose_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/pose/position", 1000);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu", 1000);
  livox_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 1000);
  cam_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/color/image", 1000);
  cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1000);
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
}

void ROSThread::run()
{
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

void ROSThread::Ready()
{
  data_stamp_thread_.active_ = false;
  data_stamp_thread_.cv_.notify_all();
  if(data_stamp_thread_.thread_.joinable())  data_stamp_thread_.thread_.join();
  
  pose_thread_.active_ = false;
  pose_thread_.cv_.notify_all();
  if(pose_thread_.thread_.joinable()) pose_thread_.thread_.join();
  
  imu_thread_.active_ = false;
  imu_thread_.cv_.notify_all(); 
  if(imu_thread_.thread_.joinable()) imu_thread_.thread_.join();
  
  livox_thread_.active_ = false;
  livox_thread_.cv_.notify_all();
  if(livox_thread_.thread_.joinable()) livox_thread_.thread_.join();
  
  cam_thread_.active_ = false;
  cam_thread_.cv_.notify_all();
  if(cam_thread_.thread_.joinable()) cam_thread_.thread_.join();
  
  //check path is right or not
  ifstream f((data_folder_path_+"/data_stamp.csv").c_str());
  if(!f.good()){
     cout << "Please check file path. Input path is wrong" << endl;
     return;
  }
  f.close();

  //Read CSV file and make map
  FILE *fp;
  int64_t stamp;
  //data stamp data load

  fp = fopen((data_folder_path_+"/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
    data_stamp_.insert( multimap<int64_t, string>::value_type(stamp, data_name));
  }
  cout << "Stamp data are loaded." << endl;
  fclose(fp);

  initial_data_stamp_ = data_stamp_.begin()->first - 1;
  last_data_stamp_ = prev(data_stamp_.end(),1)->first - 1;
  // std::cout << initial_data_stamp_ << " " << last_data_stamp_ << std::endl;

  //Read GPS data
  fp = fopen((data_folder_path_+"/pose.csv").c_str(),"r");
  double x, y, z;
  geometry_msgs::PointStamped pose_data;
  pose_data_.clear();
  while(fscanf(fp,"%ld,%lf,%lf,%lf",&stamp,&x,&y,&z) == 4){
    pose_data.header.stamp.fromNSec(stamp);
    pose_data.header.frame_id = "imu_link";
    pose_data.point.x = x;
    pose_data.point.y = y;
    pose_data.point.z = z;

    pose_data_[stamp] = pose_data;
  }
  // std::cout << gps_data_.size() << std::endl;
  cout << "Pose data are loaded." << endl;
  fclose(fp);

  //Read IMU data
  fp = fopen((data_folder_path_+"/imu.csv").c_str(),"r");
  double q_x,q_y,q_z,q_w,w_x,w_y,w_z,a_x,a_y,a_z;
  sensor_msgs::Imu imu_data;
  imu_data_.clear();

  while(1){
    int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&stamp,&q_x,&q_y,&q_z,&q_w,&w_x,&w_y,&w_z,&a_x,&a_y,&a_z);
    if(length != 11) break;
    if(length == 11){
      imu_data.header.stamp.fromNSec(stamp);
      imu_data.header.frame_id = "imu_link";
      imu_data.orientation.x = q_x;
      imu_data.orientation.y = q_y;
      imu_data.orientation.z = q_z;
      imu_data.orientation.w = q_w;
      imu_data.angular_velocity.x = w_x;
      imu_data.angular_velocity.y = w_y;
      imu_data.angular_velocity.z = w_z;
      imu_data.linear_acceleration.x = a_x;
      imu_data.linear_acceleration.y = a_y;
      imu_data.linear_acceleration.z = a_z;
      imu_data_[stamp] = imu_data;
      imu_data_version_ = 1;
    }
  }
  // std::cout << imu_data_.size() << std::endl;
  cout << "IMU data are loaded." << endl;
  fclose(fp);

  livox_file_list_.clear();
  GetDirList(data_folder_path_ + "/LiDAR", livox_file_list_);

  cam_file_list_.clear();
  GetDirList(data_folder_path_ + "/Camera", cam_file_list_);

  // Loda CameraInfo
  camera_nh_ = ros::NodeHandle(nh_, "cam");
  cam_cinfo_ = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(camera_nh_,"/camera/color"));

  string cam_yaml_file_path = "file://" + data_folder_path_ + "/Calibration/cameraParameter.yaml";

  if(cam_cinfo_->validateURL(cam_yaml_file_path)){
      cam_cinfo_->loadCameraInfo(cam_yaml_file_path);
      cout << "Success to load camera info" << endl;
      cam_info_ = cam_cinfo_->getCameraInfo();
  }
  
  data_stamp_thread_.active_ = true;
  imu_thread_.active_ = true;
  pose_thread_.active_ = true;
  livox_thread_.active_ = true;
  cam_thread_.active_ = true;
  
  data_stamp_thread_.thread_ = std::thread(&ROSThread::DataStampThread,this);
  imu_thread_.thread_ = std::thread(&ROSThread::ImuThread,this);
  pose_thread_.thread_ = std::thread(&ROSThread::PoseThread,this);
  livox_thread_.thread_ = std::thread(&ROSThread::LivoxThread,this);
  cam_thread_.thread_ = std::thread(&ROSThread::CamThread,this);
}

void ROSThread::DataStampThread()
{
  auto stop_region_iter = stop_period_.begin();

  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++){
    auto stamp = iter->first;

    while((stamp > (initial_data_stamp_+processed_stamp_))&&(data_stamp_thread_.active_ == true)){
      if(processed_stamp_ == 0){
          iter = data_stamp_.begin();
          stop_region_iter = stop_period_.begin();
          stamp = iter->first;
      }
      usleep(1);
      if(reset_process_stamp_flag_ == true) break;
      //wait for data publish
    }

    if(reset_process_stamp_flag_ == true){
      auto target_stamp = processed_stamp_ + initial_data_stamp_;
      //set iter
      iter = data_stamp_.lower_bound(target_stamp);
      iter = prev(iter,1);
      //set stop region order
      auto new_stamp = iter->first;
      stop_region_iter = stop_period_.upper_bound(new_stamp);

      reset_process_stamp_flag_ = false;
      continue;
    }

    //check whether stop region or not
    if(stamp == stop_region_iter->first){
      if(stop_skip_flag_ == true){
        cout << "Skip stop section!!" << endl;
        iter = data_stamp_.find(stop_region_iter->second);  //find stop region end
        iter = prev(iter,1);
        processed_stamp_ = stop_region_iter->second - initial_data_stamp_;
      }
      stop_region_iter++;
      if(stop_skip_flag_ == true){
        continue;
      }
    }

    if(data_stamp_thread_.active_ == false) return;
    // std::cout << iter->second << std::endl;
    if(iter->second.compare("imu") == 0){
      imu_thread_.push(stamp);
      imu_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("pose") == 0){
      pose_thread_.push(stamp);
      pose_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("livox") == 0){
      livox_thread_.push(stamp);
      livox_thread_.cv_.notify_all();
    }
    else if(iter->second.compare("cam") == 0){
      cam_thread_.push(stamp);
      cam_thread_.cv_.notify_all();
    }
    stamp_show_count_++;
    if(stamp_show_count_ > 100){
      stamp_show_count_ = 0;
      emit StampShow(stamp);
    }

    if(prev_clock_stamp_ == 0 || (stamp - prev_clock_stamp_) > 10000000){
        rosgraph_msgs::Clock clock;
        clock.clock.fromNSec(stamp);
        clock_pub_.publish(clock);
        prev_clock_stamp_ = stamp;
    }

    if(loop_flag_ == true && iter == prev(data_stamp_.end(),1)){
        iter = data_stamp_.begin();
        stop_region_iter = stop_period_.begin();
        processed_stamp_ = 0;
    }
    if(loop_flag_ == false && iter == prev(data_stamp_.end(),1)){
        play_flag_ = false;
        while(!play_flag_){
            iter = data_stamp_.begin();
            stop_region_iter = stop_period_.begin();
            processed_stamp_ = 0;
            usleep(10000);
        }
    }
  }
  cout << "Data publish complete" << endl;

}

void ROSThread::PoseThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(pose_thread_.mutex_);
    pose_thread_.cv_.wait(ul);
    if(pose_thread_.active_ == false) return;
    ul.unlock();
    while(!pose_thread_.data_queue_.empty()){
      auto data = pose_thread_.pop();
      //process
      if(pose_data_.find(data) != pose_data_.end()){
        pose_pub_.publish(pose_data_[data]);
      }

    }
    if(pose_thread_.active_ == false) return;
  }
}

void ROSThread::ImuThread()
{
  while(1){
    std::unique_lock<std::mutex> ul(imu_thread_.mutex_);
    imu_thread_.cv_.wait(ul);
    if(imu_thread_.active_ == false) return;
    ul.unlock();

    while(!imu_thread_.data_queue_.empty()){
      auto data = imu_thread_.pop();
      //process
      if(imu_data_.find(data) != imu_data_.end()){
        imu_pub_.publish(imu_data_[data]);
        // std::cout << "IMU published" << std::endl;
      }
    }
    if(imu_thread_.active_ == false) return;
  }
}

void ROSThread::TimerCallback(const ros::TimerEvent&)
{
    int64_t current_stamp = ros::Time::now().toNSec();
    if(play_flag_ == true && pause_flag_ == false){
      processed_stamp_ += static_cast<int64_t>(static_cast<double>(current_stamp - pre_timer_stamp_) * play_rate_);
    }
    pre_timer_stamp_ = current_stamp;

    if(play_flag_ == false){
      processed_stamp_ = 0; //reset
      prev_clock_stamp_ = 0;
    }
}

void ROSThread::LivoxThread()
{
  int current_file_index = 0;
  int previous_file_index = 0;
  while(1){
    std::unique_lock<std::mutex> ul(livox_thread_.mutex_);
    livox_thread_.cv_.wait(ul);
    
    if(livox_thread_.active_ == false) return;
    ul.unlock();

    while(!livox_thread_.data_queue_.empty()){
      auto data = livox_thread_.pop();
      //process

      //publish data
      if(to_string(data) + ".bin" == livox_next_.first){
        //publish
        livox_next_.second.header.stamp.fromNSec(data);
        livox_next_.second.header.frame_id = "livox";
        livox_pub_.publish(livox_next_.second);
      }
      else
      {
        //load current data
        livox_ros_driver::CustomMsg livox_msg;
        string current_file_name = data_folder_path_ + "/LiDAR" +"/"+ to_string(data) + ".bin";
        int i = 0;
        if(1){
            ifstream file;
            file.open(current_file_name, ios::in|ios::binary);
            while(!file.eof()){
                livox_ros_driver::CustomPoint point;
                file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
                file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
                file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
                livox_msg.points.push_back(point);
                i++;
            }
            file.close();
            livox_msg.point_num = i;
            livox_msg.header.stamp.fromNSec(data);
            livox_msg.header.frame_id = "livox";
            livox_pub_.publish(livox_msg);
        }
        
        previous_file_index = 0;
      }

      //load next data
      livox_ros_driver::CustomMsg livox_msg;
      current_file_index = find(next(livox_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_file_list_.end(),to_string(data)+".bin") - livox_file_list_.begin();
      if(find(next(livox_file_list_.begin(),max(0,previous_file_index-search_bound_)),livox_file_list_.end(),livox_file_list_[current_file_index+1]) != livox_file_list_.end()){
          string next_file_name = data_folder_path_ + "/LiDAR" +"/"+ livox_file_list_[current_file_index+1];
          ifstream file;
          file.open(next_file_name, ios::in|ios::binary);
          int i = 0;
          while(!file.eof()){
              livox_ros_driver::CustomPoint point;
              file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
              file.read(reinterpret_cast<char *>(&point.reflectivity), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.tag), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.line), sizeof(uint8_t));
              file.read(reinterpret_cast<char *>(&point.offset_time), sizeof(uint32_t));
              livox_msg.points.push_back(point);
              i++;
          }
          file.close();
          livox_msg.point_num = i;
          livox_msg.header.stamp.fromNSec(data);
          livox_msg.header.frame_id = "livox";
          livox_next_ = make_pair(livox_file_list_[current_file_index+1], livox_msg);
      }

      previous_file_index = current_file_index;
    }
    if(livox_thread_.active_ == false) return;
  }
}

void ROSThread::CamThread()
{
  int current_img_index = 0;
  int previous_img_index = 0;

  while(1){
    std::unique_lock<std::mutex> ul(cam_thread_.mutex_);
    cam_thread_.cv_.wait(ul);
    if(cam_thread_.active_ == false) return;
    ul.unlock();

    while(!cam_thread_.data_queue_.empty()){
      auto data = cam_thread_.pop();
      //process
      if(cam_file_list_.size() == 0) continue;

      //publish
      if(to_string(data)+".png" == cam_next_.first && !cam_next_.second.empty()){
        cv_bridge::CvImage cam_out_msg;
        cam_out_msg.header.stamp.fromNSec(data);
        cam_out_msg.header.frame_id = "cam";
        cam_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        cam_out_msg.image    = cam_next_.second;

        cam_info_.header.stamp.fromNSec(data);
        cam_info_.header.frame_id = "/camera/color";

        cam_pub_.publish(cam_out_msg.toImageMsg());
        cam_info_pub_.publish(cam_info_);

      }else{

        string current_img_name = data_folder_path_ + "/Camera" +"/"+ to_string(data)+".png";
        cv::Mat current_image;
        current_image = imread(current_img_name);//, IMREAD_ANYDEPTH);
        // cv::imshow("1",current_image);
        // cv::waitKey();
        if(!current_image.empty()){

            cv_bridge::CvImage cam_out_msg;
            cam_out_msg.header.stamp.fromNSec(data);
            cam_out_msg.header.frame_id = "cam";
            cam_out_msg.encoding = sensor_msgs::image_encodings::BGR8;
            cam_out_msg.image    = current_image;

            cam_info_.header.stamp.fromNSec(data);
            cam_info_.header.frame_id = "/camera/color";

            cam_pub_.publish(cam_out_msg.toImageMsg());
            cam_info_pub_.publish(cam_info_);
        }
        previous_img_index = 0;
      }

      //load next image
      current_img_index = find(next(cam_file_list_.begin(), max(0,previous_img_index - search_bound_)),cam_file_list_.end(),to_string(data)+".png") - cam_file_list_.begin();
      if(current_img_index < cam_file_list_.size()-2){

          string next_img_name = data_folder_path_ + "/Camera" +"/"+ cam_file_list_[current_img_index+1];
          cv::Mat next_image;
          next_image = imread(next_img_name);//, IMREAD_ANYDEPTH);
          if(!next_image.empty()){
              cam_next_ = make_pair(cam_file_list_[current_img_index+1], next_image);
          }

      }
      previous_img_index = current_img_index;
    }
    if(cam_thread_.active_ == false) return;
  }
}

int ROSThread::GetDirList(string dir, vector<string> &files)
{

  vector<string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
      perror("scandir");
  else {
      while (n--) {
      if(string(namelist[n]->d_name) != "." && string(namelist[n]->d_name) != ".."){
        tmp_files.push_back(string(namelist[n]->d_name));
      }
      free(namelist[n]);
      }
      free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++){
    files.push_back(*iter);
  }
    return 0;
}

void ROSThread::FilePlayerStart(const std_msgs::BoolConstPtr& msg)
{
  if(auto_start_flag_ == true){
    cout << "File player auto start" << endl;
    usleep(1000000);
    play_flag_ = false;
    emit StartSignal();
  }
}

void ROSThread::FilePlayerStop(const std_msgs::BoolConstPtr& msg)
{
  cout << "File player auto stop" << endl;
  play_flag_ = true;
  emit StartSignal();
}
void ROSThread::ResetProcessStamp(int position)
{
  if(position > 0 && position < 10000){
    processed_stamp_ = static_cast<int64_t>(static_cast<float>(last_data_stamp_ - initial_data_stamp_)*static_cast<float>(position)/static_cast<float>(10000));
    reset_process_stamp_flag_ = true;
  }

}
