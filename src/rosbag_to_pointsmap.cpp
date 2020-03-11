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
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>

//  定数（定数はすべて大文字で書こう）
#define MODE_POINTS 0
#define MODE_DEPTH 1

#define POINTS_STOCK 5
#define TF_STOCK 20

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
        std::deque<sensor_msgs::PointCloud2> points_queue_;
        std::deque<geometry_msgs::TransformStamped> tf_data_queue_;
        std::deque<size_t> tf_cnt_queue_;
        pcl::PointCloud<pcl::PointXYZI> points_map_;

        int LiDAR2Pointsmap();
        
};

//  クラスの初期化
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

//  クラスのメイン関数
int Rosbag2Pointsmap::Main()
{
    //  使用するトピックの一覧を表示．
    for (size_t i = 0; i < this->topics_.size(); i++) {
        std::cout << this->topics_[i] << std::endl;
    }

    //  使用するROSBAGをすべて読み込む．
    for (size_t rosbag_index = 0; rosbag_index < this->rosbag_paths_.size(); rosbag_index++) {
        rosbag::Bag bag;
        bag.open(this->rosbag_paths_[rosbag_index], rosbag::bagmode::Read);

        rosbag::View view(bag, rosbag::TopicQuery(this->topics_));

        if (this->mode_ == MODE_POINTS) {
            for(rosbag::MessageInstance const message: view) {
                sensor_msgs::PointCloud2::ConstPtr points = message.instantiate<sensor_msgs::PointCloud2>();
                if (points != nullptr) {
                    this->points_queue_.push_back(*points);

                    if (this->points_queue_.size() >= POINTS_STOCK) {
                        this->LiDAR2Pointsmap();
                    }
                }

                tf2_msgs::TFMessage::ConstPtr tf = message.instantiate<tf2_msgs::TFMessage>();
                if (tf != nullptr) {
                    for (size_t i = 0; i < tf->transforms.size(); i++) {
                        this->tf_data_queue_.push_back(tf->transforms.data()[i]);
                    }

                    this->tf_cnt_queue_.push_back(tf->transforms.size());

                    if (this->tf_cnt_queue_.size() > TF_STOCK) {
                        for (size_t i = 0; i < tf_cnt_queue_.front(); i++) {
                            this->tf_data_queue_.pop_front();
                        }
                        this->tf_cnt_queue_.pop_front();
                    }
                }
            }
        }
    }

    //  地図の保存（仮）
    this->points_map_.header.frame_id = "/map";
    pcl::io::savePCDFileBinary("/dataset/Autoware/pointsmap.pcd", this->points_map_);

    return EXIT_SUCCESS;
}

int Rosbag2Pointsmap::LiDAR2Pointsmap()
{
    //  可変長配列
    std::vector<double_t> d_times;
    std::vector<std::deque<geometry_msgs::TransformStamped>::iterator> tf_data_queue_itrs;

    //  LiDAR点群のメッセージの時間とキューの中にあるTFのメッセージの時間の差を求め，イテレータと共に可変長配列に保存する．
    for (std::deque<geometry_msgs::TransformStamped>::iterator tf_itr = this->tf_data_queue_.begin(); tf_itr != this->tf_data_queue_.end(); tf_itr++) {
        double_t time_points = (double_t)(this->points_queue_.front().header.stamp.sec) + (double_t)(this->points_queue_.front().header.stamp.nsec) * 0.000000001;
        double_t time_tf = (double_t)(tf_itr->header.stamp.sec) + (double_t)(tf_itr->header.stamp.nsec) * 0.000000001;
        if (this->points_queue_.front().header.frame_id == tf_itr->child_frame_id) {
            d_times.push_back(fabs(time_points - time_tf));
            tf_data_queue_itrs.push_back(tf_itr);
        }
    }
    //  時間の差が最小なTFを一つ選択する．
    std::deque<geometry_msgs::TransformStamped>::iterator tf_data = tf_data_queue_itrs[std::distance(d_times.begin(), std::min_element(d_times.begin(), d_times.end()))];

    //  TFとLiDAR点群を扱いやすい型に変換する．
    tf::StampedTransform tf_w2l;
    tf::transformStampedMsgToTF(*tf_data, tf_w2l);

    pcl::PointCloud<pcl::PointXYZI> points_lidar;
    pcl::fromROSMsg(this->points_queue_.front(), points_lidar);

    //  LiDAR点群を，LiDARの座標系から地図の座標系に，TFを用いて変換する．
    for (size_t i = 0; i < points_lidar.size(); i++) {
        tf::Vector3 point_lidar = tf::Vector3(points_lidar.points[i].x, points_lidar.points[i].y, points_lidar.points[i].z);
        tf::Vector3 point_world = tf_w2l * point_lidar;
        pcl::PointXYZI point;
        point.x = point_world.x();
        point.y = point_world.y();
        point.z = point_world.z();
        this->points_map_.points.push_back(point);
    }

    //  LiDAR点群をキューから消去する．
    this->points_queue_.pop_front();

    return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    //  コマンドラインパーサの設定
    cmdline::parser cmdparser;
    cmdparser.set_program_name("rosbag_to_pointsmap");
    cmdparser.footer("ROSBAG_1.bag [ROSBAG_2.bag ...]");
    cmdparser.add<std::string>("mode", 'm', "次からモードを選択します．[points, depth]", true, "points", cmdline::oneof<std::string>("points", "depth"));
    cmdparser.add<double_t>("frequency", 'f', "TFの配信周期を指定します．", false, 20.0, cmdline::range<double_t>(0.0, 1000.0));
    cmdparser.add<std::string>("points_topic", 'p', "LiDAR点群のトピック名を指定します．", false, "");
    cmdparser.add<std::string>("depthmap_topic", 'd', "深度マップのトピック名を指定します．", false, "");
    cmdparser.add<std::string>("camerainfo_topic", 'c', "CameraInfoのトピック名を指定します．", false, "");
    cmdparser.add("help", 'h', "使い方を表示します.");
    
    //  parse に失敗した場合，helpオプションが指定された場合に使い方を表示する．
    if (!cmdparser.parse(argc, argv) || cmdparser.exist("help")){
        std::cout << cmdparser.error_full() << cmdparser.usage() << std::endl;
        return EXIT_FAILURE;
    }

    //  ROSBAG ファイルのパスがない場合，終了する．
    if (cmdparser.rest().size() == 0){
        std::cout << "ROSBAG ファイル(.bag)のパスを引数に指定してください．" << cmdparser.usage() << std::endl;
        return EXIT_FAILURE;
    }

    //  modeオプションで分岐し，使用するオプションが指定されているか確認する．
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
    else assert("mode が選択されていません．");

    //  クラス
    Rosbag2Pointsmap r2p(mode, cmdparser.rest(), cmdparser.get<double_t>("frequency"));

    //  使用するトピックを設定する．    
    if (mode == MODE_POINTS) {
        r2p.topics_.push_back(cmdparser.get<std::string>("points_topic"));
    }
    else if (mode == MODE_DEPTH) {
        r2p.topics_.push_back(cmdparser.get<std::string>("depthmap_topic"));
        r2p.topics_.push_back(cmdparser.get<std::string>("camerainfo_topic"));
    }

    //  クラスのメイン関数を実行する．
    return r2p.Main();
}