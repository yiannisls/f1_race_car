// pointcloud_obstacle.cpp
// Full pipeline: 3D PointCloud → Voxel+PassThrough filter → Euclidean Clustering
// → Color-based wall rejection → Kalman Tracker → Phantom injection into LaserScan

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

// ─────────────────────────────────────────────────────────────────────────────
// Data Structures
// ─────────────────────────────────────────────────────────────────────────────

// A raw cluster coming out of the 3D Euclidean step
struct Cluster3D {
    float cx, cy, cz;     // 3D centroid in camera optical frame (Z=forward, X=right, Y=down)
    float width;          // Horizontal diagonal (XZ plane) in metres
    float height;         // Vertical extent (Y axis) in metres
    float range;          // cz  — straight-ahead distance from camera
    float angle;          // atan2(cx, cz) — positive = right
    int   point_count;
};

// A Kalman-tracked opponent car.
// We store everything in the *robot/LiDAR* 2-D frame (x=forward, y=left)
// so phantom injection is directly compatible with the LaserScan.
struct TrackedCar {
    int   id;
    float x,  y;           // position estimate (robot frame, metres)
    float vx, vy;          // velocity estimate (m/s)
    float px, py;          // Kalman error covariance (diagonal)
    int   age;             // consecutive frames matched
    int   lost;            // consecutive frames NOT matched
    bool  is_active;
    float confidence;      // 0–1, derived from age
};

// ─────────────────────────────────────────────────────────────────────────────
// Node
// ─────────────────────────────────────────────────────────────────────────────

class PointCloudObstacleNode : public rclcpp::Node {
public:
    PointCloudObstacleNode()
    : Node("pointcloud_obstacle_node"),
      next_id_(0)
    {
        // ── Subscriptions ────────────────────────────────────────────────
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/yellow_car/scan/point_cloud", 10,
            std::bind(&PointCloudObstacleNode::pc_callback, this, std::placeholders::_1));

        // We need the latest LaserScan to inject phantoms into.
        // The node caches the most recent one.
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_fused", 10,
            std::bind(&PointCloudObstacleNode::scan_callback, this, std::placeholders::_1));

        // ── Publishers ───────────────────────────────────────────────────
        scan_pub_   = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/yellow_car/scan_pc_phantoms", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/yellow_car/pc_cluster_markers", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloudObstacleNode ready.");
    }

private:
    // ── Scan cache (thread-safe enough for single-threaded spin) ─────────
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    // ─────────────────────────────────────────────────────────────────────
    // MAIN PIPELINE
    // ─────────────────────────────────────────────────────────────────────
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        // ── STEP 1: Convert ROS2 → PCL ────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // ADD THIS temporarily after fromROSMsg:
        if (!cloud->empty()) {
            auto& p = cloud->points[cloud->size()/2];
            RCLCPP_INFO(this->get_logger(), 
                "Sample point: x=%.3f y=%.3f z=%.3f  total=%zu", 
                p.x, p.y, p.z, cloud->size());
        }

        if (cloud->empty()) return;

        // ── STEP 2: Voxel Downsample ──────────────────────────────────
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*downsampled);

        // ── STEP 3: PassThrough on Y (remove floor & ceiling) ─────────
        // ROS camera optical frame: Z forward, X right, Y DOWN.
        // Floor is roughly at Y ≈ +0.10 m (camera is ~10 cm above ground).
        // A 1:10 car is ~12 cm tall, so its roof reaches Y ≈ -0.05 m.
        // We keep Y in [-0.30, +0.08] to catch the whole car body, drop the floor.
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(downsampled);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(pass_y_min_, pass_y_max_);
        pass.filter(*filtered);

        // Also clip depth range to avoid far noise
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered2(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.10f, max_depth_m_);
        pass.filter(*filtered2);

        if (filtered2->empty()) return;

        // ── STEP 4: Euclidean Clustering ──────────────────────────────
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(filtered2);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_m_);
        ec.setMinClusterSize(min_cluster_pts_);
        ec.setMaxClusterSize(max_cluster_pts_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(filtered2);
        ec.extract(cluster_indices);

        // ── STEP 5: Classify clusters → Cluster3D ─────────────────────
        std::vector<Cluster3D> raw_clusters;

        for (const auto& indices : cluster_indices) {
            float min_x=1e6, max_x=-1e6;
            float min_y=1e6, max_y=-1e6;
            float min_z=1e6, max_z=-1e6;
            float sum_x=0, sum_y=0, sum_z=0;

            for (int idx : indices.indices) {
                const auto& pt = filtered2->points[idx];
                sum_x += pt.x; sum_y += pt.y; sum_z += pt.z;
                min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
                min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
                min_z = std::min(min_z, pt.z); max_z = std::max(max_z, pt.z);
            }

            int n = (int)indices.indices.size();
            float cx = sum_x / n;
            float cy = sum_y / n;
            float cz = sum_z / n;

            // Width = horizontal diagonal in XZ plane (camera optical frame)
            float width  = std::sqrt(std::pow(max_x - min_x, 2) + std::pow(max_z - min_z, 2));
            float height = max_y - min_y; // Y is down, so this is always positive

            // ── Geometry filter ──────────────────────────────────────
            if (width  < min_cluster_width_  || width  > max_cluster_width_ ) continue;
            if (height < min_cluster_height_ || height > max_cluster_height_) continue;

            // ── Color filter: reject coloured walls ──────────────────
            // No RGB on this cloud — colour filter disabled
            // if (avg_r > avg_g + color_margin_ && avg_r > avg_b + color_margin_) continue;
            // if (avg_g > avg_r + color_margin_ && avg_g > avg_b + color_margin_) continue;

            float range = cz;                        // depth straight ahead
            float angle = std::atan2(cx, cz);        // positive = rightward

            raw_clusters.push_back({cx, cy, cz, width, height, range, angle, n});
        }

        RCLCPP_DEBUG(this->get_logger(),
            "Raw clusters after geometry+color: %zu", raw_clusters.size());

        // ── STEP 6: Kalman — Predict ──────────────────────────────────
        // We work in LiDAR/robot frame (x=forward, y=left).
        // Camera optical Z → robot x; camera optical -X → robot y
        // (assuming camera faces forward, standard mounting).
        // The conversion is done when we spawn/match below.

        for (auto& t : trackers_) {
            if (!t.is_active) continue;
            t.x += t.vx * dt_;
            t.y += t.vy * dt_;
            t.px += Q_;
            t.py += Q_;
            t.lost++;
        }

        // ── STEP 7: Convert raw_clusters to robot 2-D frame for association
        // camera optical: Z=fwd, X=right, Y=down  →  robot: x=fwd, y=left
        struct Meas2D { float rx, ry; };  // robot-frame 2D measurement
        std::vector<Meas2D> measurements;
        for (auto& c : raw_clusters) {
            measurements.push_back({ c.cz, -c.cx });   // robot x = cam Z, robot y = -cam X
        }

        // ── STEP 8: Data association (greedy nearest-neighbour) ───────
        std::vector<bool> meas_matched(measurements.size(), false);

        for (auto& t : trackers_) {
            if (!t.is_active) continue;

            float best_dist = max_assoc_dist_m_;
            int   best_mi   = -1;

            for (int mi = 0; mi < (int)measurements.size(); ++mi) {
                if (meas_matched[mi]) continue;
                float dx = measurements[mi].rx - t.x;
                float dy = measurements[mi].ry - t.y;
                float d  = std::sqrt(dx*dx + dy*dy);
                if (d < best_dist) { best_dist = d; best_mi = mi; }
            }

            if (best_mi >= 0) {
                // Kalman Update
                float Kx = t.px / (t.px + R_);
                float Ky = t.py / (t.py + R_);
                float ix  = measurements[best_mi].rx - t.x;
                float iy  = measurements[best_mi].ry - t.y;

                // Smooth velocity from position innovation
                t.vx += Kx * (ix / dt_ - t.vx) * vel_smooth_;
                t.vy += Ky * (iy / dt_ - t.vy) * vel_smooth_;

                t.x += Kx * ix;
                t.y += Ky * iy;
                t.px = (1.0f - Kx) * t.px;
                t.py = (1.0f - Ky) * t.py;

                t.lost = 0;
                t.age++;
                meas_matched[best_mi] = true;
            }

            if (t.lost > max_lost_frames_) t.is_active = false;
        }

        // ── STEP 9: Spawn new trackers for unmatched measurements ─────
        for (int mi = 0; mi < (int)measurements.size(); ++mi) {
            if (meas_matched[mi]) continue;
            TrackedCar t;
            t.id   = next_id_++;
            t.x    = measurements[mi].rx;
            t.y    = measurements[mi].ry;
            t.vx   = 0.0f; t.vy = 0.0f;
            t.px   = 1.0f; t.py = 1.0f;
            t.age  = 1; t.lost = 0;
            t.is_active  = true;
            t.confidence = 0.0f;
            trackers_.push_back(t);
        }

        // ── STEP 10: Compute confidence & collect confirmed cars ──────
        // confidence = clamp(age / confirm_age, 0, 1)
        std::vector<TrackedCar*> confirmed;
        for (auto& t : trackers_) {
            if (!t.is_active) continue;
            t.confidence = std::min(1.0f, (float)t.age / (float)min_age_to_trust_);
            if (t.age >= min_age_to_trust_) confirmed.push_back(&t);
        }

        // Log confirmed cars (the list you "didn't know why you wanted" — useful for a planner!)
        for (auto* t : confirmed) {
            RCLCPP_DEBUG(this->get_logger(),
                "Car #%d  pos=(%.2f, %.2f)  vel=(%.2f, %.2f)  conf=%.2f",
                t->id, t->x, t->y, t->vx, t->vy, t->confidence);
        }

        // ── STEP 11: Inject phantoms into LaserScan (ALWAYS PUBLISH) ──
        if (latest_scan_) {
            auto out_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*latest_scan_);

            for (auto* t : confirmed) {
                // One-step ahead prediction
                float pred_x = t->x + t->vx * dt_;
                float pred_y = t->y + t->vy * dt_;
                float pred_range = std::sqrt(pred_x*pred_x + pred_y*pred_y);
                float pred_angle = std::atan2(pred_y, pred_x);

                float bubble       = phantom_radius_m_ + phantom_inflate_m_;
                float angular_half = std::atan2(bubble, pred_range);

                int steps = (int)std::ceil(angular_half /
                    std::abs(out_scan->angle_increment));
                int center_idx = (int)std::round(
                    (pred_angle - out_scan->angle_min) /
                    std::abs(out_scan->angle_increment));

                float phantom_range = std::max(0.10f, pred_range - bubble * 0.5f);

                for (int di = -steps; di <= steps; ++di) {
                    int idx = center_idx + di;
                    if (idx < 0 || idx >= (int)out_scan->ranges.size()) continue;
                    if (!std::isfinite(out_scan->ranges[idx]) ||
                        out_scan->ranges[idx] > phantom_range) {
                        out_scan->ranges[idx] = phantom_range;
                    }
                }
            }
            scan_pub_->publish(*out_scan);
        }

        // ── STEP 12: Publish RViz markers ─────────────────────────────
        publish_markers(msg->header, raw_clusters);

        // ── STEP 13: Cleanup dead trackers (Memory Leak Fix) ──────────
        trackers_.erase(
            std::remove_if(trackers_.begin(), trackers_.end(),
                [](const TrackedCar& t) { return !t.is_active; }),
            trackers_.end());
    }

    // ─────────────────────────────────────────────────────────────────────
    void publish_markers(
        const std_msgs::msg::Header& header,
        const std::vector<Cluster3D>& clusters)
    {
        visualization_msgs::msg::MarkerArray ma;

        // Clear previous frame
        visualization_msgs::msg::Marker del;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(del);

        for (size_t i = 0; i < clusters.size(); ++i) {
            visualization_msgs::msg::Marker m;
            m.header      = header;
            m.header.frame_id = "laser"; // Coordinates are manually mapped to robot frame
            m.id          = (int)i;
            m.ns          = "pc_clusters";
            m.type        = visualization_msgs::msg::Marker::CYLINDER;
            m.action      = visualization_msgs::msg::Marker::ADD;

            // Camera frame: Z forward, X right, Y down
            m.pose.position.x = clusters[i].cz;   // fwd in robot frame
            m.pose.position.y = -clusters[i].cx;  // left in robot frame
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;

            m.scale.x = clusters[i].width;
            m.scale.y = clusters[i].width;
            m.scale.z = clusters[i].height;

            // Cyan: confirmed-track candidates (geometry + color passed)
            m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 0.6f;

            m.lifetime = rclcpp::Duration::from_seconds(0.15);
            ma.markers.push_back(m);
        }

        // Also draw confirmed trackers in a different colour (green)
        for (auto& t : trackers_) {
            if (!t.is_active || t.age < min_age_to_trust_) continue;
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "laser";   // adjust to your robot frame
            m.header.stamp    = this->get_clock()->now();
            m.id              = 1000 + t.id;
            m.ns              = "pc_trackers";
            m.type            = visualization_msgs::msg::Marker::ARROW;
            m.action          = visualization_msgs::msg::Marker::ADD;

            m.pose.position.x = t.x;
            m.pose.position.y = t.y;
            m.pose.position.z = 0.1;

            // Point arrow in velocity direction
            float yaw = std::atan2(t.vy, t.vx);
            m.pose.orientation.z = std::sin(yaw / 2.0f);
            m.pose.orientation.w = std::cos(yaw / 2.0f);

            float speed = std::sqrt(t.vx*t.vx + t.vy*t.vy);
            m.scale.x = std::max(0.05f, speed * 0.3f);  // arrow length ∝ speed
            m.scale.y = 0.03f;
            m.scale.z = 0.03f;

            m.color.r = 0.0f; m.color.g = 1.0f; m.color.b = 0.0f; m.color.a = 1.0f;
            m.lifetime = rclcpp::Duration::from_seconds(0.15);
            ma.markers.push_back(m);
        }

        marker_pub_->publish(ma);
    }

    // ─────────────────────────────────────────────────────────────────────
    // State
    // ─────────────────────────────────────────────────────────────────────
    int  next_id_;
    std::vector<TrackedCar> trackers_;

    // ── Tuning ────────────────────────────────────────────────────────────
    // Filters
    const float voxel_leaf_        = 0.04f;   // 4 cm voxel downsample
    const float pass_y_min_        = -0.30f;  // keep points above floor (Y=down, so negative = up)
    const float pass_y_max_        =  0.08f;  // drop floor hits
    const float max_depth_m_       =  4.0f;   // ignore far background

    // Euclidean clustering
    const float cluster_tolerance_m_ = 0.20f;
    const int   min_cluster_pts_     = 5;
    const int   max_cluster_pts_     = 400;

    // Bounding box car filter
    const float min_cluster_width_  = 0.08f;
    const float max_cluster_width_  = 0.55f;
    const float min_cluster_height_ = 0.04f;
    const float max_cluster_height_ = 0.30f;

    // Color rejection (wall suppression)
    const int   color_margin_       = 40;     // channel must exceed others by this

    // Kalman
    const float dt_          = 0.10f;   // assume 10 Hz depth camera
    const float Q_           = 0.05f;   // process noise
    const float R_           = 0.20f;   // measurement noise
    const float vel_smooth_  = 0.30f;   // velocity update smoothing

    // Tracker lifecycle
    const float max_assoc_dist_m_ = 0.60f;
    const int   max_lost_frames_  = 5;
    const int   min_age_to_trust_ = 4;    // frames before a tracker becomes "confirmed"

    // Phantom injection
    const float phantom_radius_m_  = 0.30f;
    const float phantom_inflate_m_ = 0.20f;

    // ── ROS handles ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr  pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr       scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudObstacleNode>());
    rclcpp::shutdown();
    return 0;
}