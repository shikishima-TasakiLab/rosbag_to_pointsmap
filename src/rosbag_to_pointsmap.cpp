#include <fileaccess.hpp>
#include <cmdline.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#define MODE_POINTS 0
#define MODE_DEPTH 1

#define POINTS_STOCK 5
#define TF_STOCK 10

class Rosbag2Pointsmap
{
    public:
        std::vector<std::string> topics_;

        Rosbag2Pointsmap(u_int mode, std::vector<std::string> rosbag_path, double_t frequency);
        int Main();

    private:
        u_int mode_;
        double_t frequency_;
        std::vector<std::string> rosbag_paths_;
        std::queue<sensor_msgs::PointCloud2> points_queue_;
        std::queue<tf2_msgs::TFMessage> tf_queue_;
        
};

Rosbag2Pointsmap::Rosbag2Pointsmap(u_int mode, std::vector<std::string> rosbag_path, double_t frequency)
{
    this->mode_ = mode;
    this->frequency_ = frequency;

    for (size_t i = 0; i < rosbag_path.size(); i++) {
        if (filedirE(rosbag_path[i]) != FILE_EXIST) {
            ROS_WARN("\"%s\" is not found.", rosbag_path[i].c_str());
            continue;
        }
        if (get_extension(rosbag_path[i]) == ".bag") {
            this->rosbag_paths_.push_back(rosbag_path[i]);
            ROS_INFO("Load: \"%s\"", rosbag_path[i].c_str());
        }
        else{
            ROS_WARN("\"%s\" is not ROSBAG(.bag).", rosbag_path[i].c_str());
        }
    }
    this->topics_.push_back("/tf");
    this->topics_.push_back("/tf_static");
}

int Rosbag2Pointsmap::Main()
{
    for (size_t i = 0; i < this->topics_.size(); i++) {
        std::cout << this->topics_[i] << std::endl;
    }
    for (size_t rosbag_index = 0; rosbag_index < this->rosbag_paths_.size(); rosbag_index++) {
        rosbag::Bag bag;
        bag.open(this->rosbag_paths_[rosbag_index], rosbag::bagmode::Read);

        std::cout << "test_1" << std::endl;

        rosbag::View view(bag, rosbag::TopicQuery(this->topics_));

        if (this->mode_ == MODE_POINTS) {
            for(rosbag::MessageInstance const message: view) {
                sensor_msgs::PointCloud2::ConstPtr points = message.instantiate<sensor_msgs::PointCloud2>();
                if (points != nullptr) {
                    std::cout << message.getTopic() << " : " 
                              << message.getTime() << " : "
                              << points->header.stamp << " : " 
                              << points->header.frame_id << " : " 
                              << points->width << std::endl;
                    std::cout << std::endl;

                    this->points_queue_.push(*points);

                    if (this->points_queue_.size() >= POINTS_STOCK) {

                    }
                }

                tf2_msgs::TFMessage::ConstPtr tf = message.instantiate<tf2_msgs::TFMessage>();
                if (tf != nullptr) {
                    for (size_t i = 0; i < tf->transforms.size(); i++) {
                        std::cout << message.getTopic() << " : " 
                                  << message.getTime() << " : "
                                  << tf->transforms.data()[i].header.stamp << " : " 
                                  << tf->transforms.data()[i].header.frame_id << " : " 
                                  << tf->transforms.data()[i].child_frame_id << std::endl;
                    }
                    std::cout << std::endl;
                    this->tf_queue_.push(*tf);
                }
            }
        }
    }

    return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    cmdline::parser cmdparser;
    cmdparser.set_program_name("rosbag_to_pointsmap");
    cmdparser.footer("ROSBAG_1.bag [ROSBAG_2.bag ...]");
    cmdparser.add<std::string>("mode", 'm', "次からモードを選択します．[points, depth]", true, "points", cmdline::oneof<std::string>("points", "depth"));
    cmdparser.add<double_t>("frequency", 'f', "TFの配信周期を指定します．", true, 20.0, cmdline::range<double_t>(0.0, 1000.0));
    cmdparser.add<std::string>("points_topic", 'p', "LiDAR点群のトピック名を指定します．", false, "");
    cmdparser.add<std::string>("depthmap_topic", 'd', "深度マップのトピック名を指定します．", false, "");
    cmdparser.add<std::string>("camerainfo_topic", 'c', "CameraInfoのトピック名を指定します．", false, "");
    cmdparser.add("help", 'h', "使い方を表示します.");
    
    // parse に失敗した場合，helpオプションが指定された場合に使い方を表示する．
    if (!cmdparser.parse(argc, argv) || cmdparser.exist("help")){
        std::cout << cmdparser.error_full() << cmdparser.usage() << std::endl;
        return EXIT_FAILURE;
    }

    //  ROSBAG ファイルのパスがない場合，終了する．
    if (cmdparser.rest().size() == 0){
        std::cout << "ROSBAG ファイル(.bag)のパスを引数に指定してください．" << cmdparser.usage() << std::endl;
        return EXIT_FAILURE;
    }

    u_int mode;
    if (cmdparser.get<std::string>("mode") == "points") {
        mode = MODE_POINTS;
        if (cmdparser.get<std::string>("points_topic") == "") {
            std::cout << "-p, --points_topic オプションを指定してください．" << cmdparser.usage() << std::endl;
            return EXIT_FAILURE;
        }
    }
    else if (cmdparser.get<std::string>("mode") == "depth") {
        mode = MODE_DEPTH;
        if (cmdparser.get<std::string>("depthmap_topic") == "") {
            std::cout << "-d, --depthmap_topic オプションを指定してください．" << cmdparser.usage() << std::endl;
            return EXIT_FAILURE;
        }
        if (cmdparser.get<std::string>("depthmap_topic") == "") {
            std::cout << "-c, --camerainfo_topic オプションを指定してください．" << cmdparser.usage() << std::endl;
            return EXIT_FAILURE;
        }
    }
    else assert("mode が選択できていません．");

    Rosbag2Pointsmap r2p(mode, cmdparser.rest(), cmdparser.get<double_t>("frequency"));
    
    if (mode == MODE_POINTS) {
        r2p.topics_.push_back(cmdparser.get<std::string>("points_topic"));
    }
    else if (mode == MODE_DEPTH) {
        r2p.topics_.push_back(cmdparser.get<std::string>("depthmap_topic"));
        r2p.topics_.push_back(cmdparser.get<std::string>("camerainfo_topic"));
    }

    return r2p.Main();
}