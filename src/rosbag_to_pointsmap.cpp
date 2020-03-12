//  使用するライブラリの定義
#include <fileaccess.hpp>
#include <cmdline.h>

#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

//  定数（定数はすべて大文字で書こう）
#define MODE_POINTS 0
#define MODE_DEPTH 1

#define POINTS_STOCK 5
#define TF_STOCK 20

#define SAVE_POINTSMAP 1000000

//  クラスの定義
class Rosbag2Pointsmap
{
    //  クラスの外から参照できるメンバ変数・関数．
    public:
        std::vector<std::string> topics_;
        float_t minimum_scan_range_;
        float_t leaf_size_;

        Rosbag2Pointsmap(u_int mode, std::vector<std::string> rosbag_path, double_t frequency, std::string output_path, bool ros_on);
        int Main();

    //  クラスの中でのみ参照できるメンバ変数・関数．
    private:
        u_int mode_;
        double_t frequency_;
        std::vector<std::string> rosbag_paths_;
        bool ros_on_;

        ros::NodeHandle nh_;
        ros::Publisher lider_points_;
        tf::TransformBroadcaster tf_br_;

        std::deque<sensor_msgs::PointCloud2> points_queue_;
        std::deque<geometry_msgs::TransformStamped> tf_data_queue_;
        std::deque<size_t> tf_cnt_queue_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr points_map_;
        std::string output_path_;
        size_t output_count_;

        int LiDAR2Pointsmap();
        int SavePointsmap();       
};

//  クラスの初期化
Rosbag2Pointsmap::Rosbag2Pointsmap(u_int mode, std::vector<std::string> rosbag_path, double_t frequency, std::string output_path, bool ros_on)
    :   points_map_(new pcl::PointCloud<pcl::PointXYZ>)
    ,   output_count_(0)
    ,   minimum_scan_range_(2.0f)
    ,   leaf_size_(0.2f)
{
    //  引数をクラスのメンバ変数に格納する．
    this->mode_ = mode;
    this->frequency_ = frequency;
    this->output_path_ = output_path;
    this->ros_on_ = ros_on;

    //  rosbag_path引数からROSBAGファイルのパスのみ格納する．
    for (size_t i = 0; i < rosbag_path.size(); i++) {
        if (rosbag_path[i].find(":=") != std::string::npos) {
            ROS_INFO("\"%s\" is ROS option.", rosbag_path[i].c_str());
            continue;
        }
        if (filedirE(rosbag_path[i]) != FILE_EXIST) {
            ROS_WARN("\"%s\" is not found.", rosbag_path[i].c_str());
            continue;
        }
        std::cout << get_directory(rosbag_path[i]) << std::endl;
        std::cout << get_filename(rosbag_path[i]) << std::endl;
        std::cout << path_join(get_directory(rosbag_path[i]), "/rosbag/", get_filename(rosbag_path[i]) + get_extension(rosbag_path[i])) << std::endl;
        if (get_extension(rosbag_path[i]) == ".bag") {
            this->rosbag_paths_.push_back(rosbag_path[i]);
            ROS_INFO("Load: \"%s\"", rosbag_path[i].c_str());
        }
        else{
            ROS_WARN("\"%s\" is not ROSBAG(.bag).", rosbag_path[i].c_str());
        }
    }

    //  使用するトピックを追加．
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
        //  ROSBAGファイルを開く．
        rosbag::Bag bag;
        bag.open(this->rosbag_paths_[rosbag_index], rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(this->topics_));

        //  LiDAR点群から地図を生成するモード．
        if (this->mode_ == MODE_POINTS) {
            //  メッセージを一つずつ読み込む．
            for(rosbag::MessageInstance const message: view) {
                //  メッセージの型が sensor_msgs::PointCloud2 の場合の処理．
                sensor_msgs::PointCloud2::ConstPtr points = message.instantiate<sensor_msgs::PointCloud2>();
                if (points != nullptr) {
                    //  メッセージをキューに格納．
                    this->points_queue_.push_back(*points);

                    //  キューに格納されたメッセージの数が一定量を超えたら，地図生成を開始．
                    if (this->points_queue_.size() >= POINTS_STOCK) {
                        this->LiDAR2Pointsmap();
                    }
                }

                //  トピックの型が tf2_msgs::TFMessage の場合の処理．
                tf2_msgs::TFMessage::ConstPtr tf = message.instantiate<tf2_msgs::TFMessage>();
                if (tf != nullptr) {
                    //  メッセージの中のデータをキューに格納．
                    for (size_t i = 0; i < tf->transforms.size(); i++) {
                        this->tf_data_queue_.push_back(tf->transforms.data()[i]);
                    }

                    //  メッセージの中のデータの数をキューに格納．
                    this->tf_cnt_queue_.push_back(tf->transforms.size());

                    //  キューに格納されたメッセージの数が一定量を超えたら，古いデータを削除する．
                    if (this->tf_cnt_queue_.size() > TF_STOCK) {
                        for (size_t i = 0; i < tf_cnt_queue_.front(); i++) {
                            this->tf_data_queue_.pop_front();
                        }
                        this->tf_cnt_queue_.pop_front();
                    }
                }
            }

            while (this->points_queue_.size() > 0) this->LiDAR2Pointsmap();
            this->SavePointsmap();
        }
    }

    return EXIT_SUCCESS;
}

//  LiDAR点群を重ねて三次元地図を生成する．
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

    pcl::PointCloud<pcl::PointXYZ> points_lidar;
    pcl::fromROSMsg(this->points_queue_.front(), points_lidar);

    //  LiDAR点群を，LiDARの座標系から地図の座標系に，TFを用いて変換する．
    for (size_t i = 0; i < points_lidar.size(); i++) {
        tf::Vector3 point_lidar = tf::Vector3(points_lidar.points[i].x, points_lidar.points[i].y, points_lidar.points[i].z);
        if (point_lidar.distance(tf::Vector3(0.0f, 0.0f, 0.0f)) < this->minimum_scan_range_) continue;
        tf::Vector3 point_world = tf_w2l * point_lidar;
        pcl::PointXYZ point;
        point.x = point_world.x();
        point.y = point_world.y();
        point.z = point_world.z();
        this->points_map_->points.push_back(point);
    }

    //  LiDAR点群をキューから消去する．
    this->points_queue_.pop_front();

    //  三次元地図の点の数が一定量を超えたら，いったん保存する．
    if (this->points_map_->points.size() > SAVE_POINTSMAP) {
        this->SavePointsmap();
    }

    return EXIT_SUCCESS;
}

//  三次元地図の保存．
int Rosbag2Pointsmap::SavePointsmap()
{
    if (this->points_map_->points.size() == 0) return EXIT_FAILURE;

    this->output_count_ ++;

    std::string dir_path = get_directory(this->output_path_);
    std::string file_name = get_filename(this->output_path_);
    std::string file_ext = get_extension(this->output_path_);
    std::string save_path;

    if (this->output_count_ > 1) {
        std::stringstream num_str;
        num_str << this->output_count_;
        save_path = path_join(dir_path, file_name + "_" + num_str.str() + file_ext);
    }
    else {
        save_path = this->output_path_;
    }

    pcl::PointCloud<pcl::PointXYZ> save_pointsmap;

    if (this->leaf_size_ > 0.0f) {
        pcl::VoxelGrid<pcl::PointXYZ> vgf;
        vgf.setInputCloud(this->points_map_);
        vgf.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
        vgf.filter(save_pointsmap);
    }
    else {
        save_pointsmap = *this->points_map_;
    }

    save_pointsmap.header.frame_id = "/map";
    pcl::io::savePCDFileBinary(save_path, save_pointsmap);
    std::cout << "\"" << save_path << "\" に生成した三次元地図を保存しました．" << std::endl;

    this->points_map_->points.clear();

    return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
    //  コマンドラインパーサの設定
    cmdline::parser cmdparser;
    cmdparser.set_program_name("rosbag_to_pointsmap");
    cmdparser.footer("ROSBAG_1.bag [ROSBAG_2.bag ...]");
    cmdparser.add<std::string>("mode", 'm', "次からモードを選択します．[points, depth]", true, "points", cmdline::oneof<std::string>("points", "depth"));
    cmdparser.add<std::string>("output", 'o', "出力する三次元地図のパスを指定します．", true, "/tmp/pointsmap.pcd");
    cmdparser.add<double_t>("frequency", 'f', "TFの配信周期を指定します．", false, 20.0, cmdline::range<double_t>(0.0, 1000.0));
    cmdparser.add<std::string>("points_topic", 'p', "LiDAR点群のトピック名を指定します．", false, "/points_raw");
    cmdparser.add<std::string>("depthmap_topic", 'd', "深度マップのトピック名を指定します．", false, "/depth");
    cmdparser.add<std::string>("camerainfo_topic", 'c', "CameraInfoのトピック名を指定します．", false, "/camera_info");
    cmdparser.add<float_t>("minimum_scan_range", 's', "使用するLiDARやカメラからの距離(m)の最小値を指定する．", false, 2.0f);
    cmdparser.add<float_t>("leaf_size", 'l', "点群の密度を小さくするVoxel Grid Filterのリーフサイズ(m)を設定します．", false, 0.2f);
    cmdparser.add("ros", 'r', "ROSのトピックを配信します．");
    cmdparser.add("help", 'h', "使い方を表示します.");
    
    //  parse に失敗した場合，helpオプションが指定された場合に使い方を表示する．
    if (!cmdparser.parse(argc, argv) || cmdparser.exist("help")){
        std::cout << cmdparser.error_full() << cmdparser.usage() << std::endl;
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

    //  ROSBAG ファイルのパスがない場合，終了する．
    if (cmdparser.rest().size() == 0){
        std::cout << "ROSBAG ファイル(.bag)のパスを引数に指定してください．" << cmdparser.usage() << std::endl;
        return EXIT_FAILURE;
    }

    //  三次元地図の出力先のディレクトリがない場合，終了する．
    std::string output = cmdparser.get<std::string>("output");
    if (filedirE(get_directory(output)) != DIR_EXIST) {
        std::cout << "-o, --output : 出力先のディレクトリが存在しません．" << std::endl;
        return EXIT_FAILURE;
    }
    //  出力する三次元地図のファイル名がない場合，終了する．
    if (get_filename(output) == "") {
        std::cout << "-o, --output : ファイル名が指定されていません．" << std::endl;
        return EXIT_FAILURE;
    }

    //  minimum_scan_range が0.0以上でない場合，終了する．
    float_t minimum_scan_range = cmdparser.get<float_t>("minimum_scan_range");
    if (minimum_scan_range < 0.0f) {
        std::cout << "-s, --minimum_scan_range : 0.0 以上に指定してください．" << std::endl;
        return EXIT_FAILURE;
    }

    //  leaf_size が0.0以上でない場合，終了する．
    float_t leaf_size = cmdparser.get<float_t>("leaf_size");
    if (leaf_size < 0.0f) {
        std::cout << "-l, --leaf_size : 0.0 以上に指定してください．" << std::endl;
        return EXIT_FAILURE;
    }

    bool ros_on = cmdparser.exist("ros");
    ros::init(argc, argv, "rosbag_to_pointsmap");

    //  クラス
    Rosbag2Pointsmap r2p(mode, cmdparser.rest(), cmdparser.get<double_t>("frequency"), output, ros_on);

    //  使用するトピックを設定する．    
    if (mode == MODE_POINTS) {
        r2p.topics_.push_back(cmdparser.get<std::string>("points_topic"));
    }
    else if (mode == MODE_DEPTH) {
        r2p.topics_.push_back(cmdparser.get<std::string>("depthmap_topic"));
        r2p.topics_.push_back(cmdparser.get<std::string>("camerainfo_topic"));
    }

    r2p.minimum_scan_range_ = minimum_scan_range;
    r2p.leaf_size_ = leaf_size;

    //  クラスのメイン関数を実行する．
    return r2p.Main();
}