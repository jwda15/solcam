#include "rclcpp/rclcpp.hpp"
#include "ros2_tracking_node/msg/owner_pose.hpp"
#include "ros2_tracking_node/msg/detection_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ByteTrack/tracker_c_api.h"

#include <cmath>
#include <memory>
#include <limits>
#include <vector>

// --------------------------------------------------------
// TrackingNode (D435i + YOLO 버전)
//
// 입력: /detections        (ros2_tracking_node/DetectionArray)
// 입력: /camera/color/camera_info (sensor_msgs/CameraInfo)
// 출력: /owner_pose        (ros2_tracking_node/OwnerPose)
//
// 주인 추적 전략 (OwnerTracker):
//   - ByteTrack 내부 ID와 무관하게 "주인" 개념을 별도 관리
//   - 주인 초기화: 첫 트랙 등장 시 화면 중앙 가장 가까운 트랙
//   - 주인 Lost 시: grace_frames 동안 ByteTrack 내부 복구 대기
//     → 복구 안 되면 마지막 위치 기준 가장 가까운 트랙으로 재매핑
//     → max_reassign_dist_px 초과 시 재매핑 거부 (다른 사람 방지)
// --------------------------------------------------------

class TrackingNode : public rclcpp::Node
{
public:
    TrackingNode() : Node("tracking_node")
    {
        // ByteTrack 파라미터
        this->declare_parameter("frame_rate",           30);
        this->declare_parameter("track_buffer",         60);    // Lost 트랙 2초 유지
        this->declare_parameter("track_thresh",         0.45);
        this->declare_parameter("high_thresh",          0.6);
        this->declare_parameter("match_thresh",         0.85);
        this->declare_parameter("image_width",          640);
        this->declare_parameter("image_height",         480);
        this->declare_parameter("std_weight_position",  0.05);
        this->declare_parameter("std_weight_velocity",  0.00625);
        this->declare_parameter("std_weight_depth",     0.125); // 1/8: depth 변화에 관대하게

        // OwnerTracker 파라미터
        this->declare_parameter("grace_frames",         10);    // Lost 후 재매핑까지 대기 프레임
        // [0525] 동적 재매핑 거리: 주인을 놓친 직후엔 좁게(코앞만), lost가 길어질수록
        //   점점 넓혀서, 짧은 깜빡임엔 KF 외삽이 메우고 긴 가림엔 멀리서도 재매핑.
        //   eff_dist = base + growth * (lost_count - grace_frames)
        //   (freiburg3_walking 측정: 옆사람 거리 25%=292px/median=390px. base=120이면
        //    깜빡임 직후 옆사람 거부, lost~40f(1.3s)부터 점진 허용. 상한 없음.)
        this->declare_parameter("reassign_base_dist_px",        50.0); // grace 직후 허용 거리 (좁게: 옆사람 차단)
        this->declare_parameter("reassign_growth_px_per_frame",  5.0); // lost 1프레임당 거리 증가 (천천히)
        this->declare_parameter("max_reassign_dist_px", 300.0); // [deprecated] 동적거리로 대체됨. 미사용.
        this->declare_parameter("lost_reset_frames",    60);    // 이 이상 lost면 옛 owner 포기 → fresh init.
                                                                 // track_buffer와 동일 기본값. owner가 영구히
                                                                 // 사라진 뒤(BYTETracker가 lost 풀에서 제거한 뒤)
                                                                 // 옛 마지막 픽셀 위치에 묶이지 않게 함.

        // Smoothing / outlier rejection 파라미터
        this->declare_parameter("max_speed_xy_mps",   2.0);  // 좌우 최대 속도 (m/s)
        this->declare_parameter("max_speed_z_mps",    2.0);  // 전후 최대 속도 (m/s)
        // ↑ 걷는 사람 ~1.5 m/s. 여유 두고 2.0. 이 이상의 추정 속도면 outlier.
        this->declare_parameter("ema_alpha_xy",           0.35); // x,y EMA 계수
        this->declare_parameter("ema_alpha_z",            0.15); // z EMA 계수

        // [0524] 메트릭 요약 로그 주기(프레임). 0이면 요약 로그 끕.
        this->declare_parameter("metric_log_period",  150);

        int   frame_rate   = this->get_parameter("frame_rate").as_int();
        int   track_buffer = this->get_parameter("track_buffer").as_int();
        float track_thresh = (float)this->get_parameter("track_thresh").as_double();
        float high_thresh  = (float)this->get_parameter("high_thresh").as_double();
        float match_thresh = (float)this->get_parameter("match_thresh").as_double();

        image_width_  = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();

        float std_weight_position = (float)this->get_parameter("std_weight_position").as_double();
        float std_weight_velocity = (float)this->get_parameter("std_weight_velocity").as_double();
        float std_weight_depth    = (float)this->get_parameter("std_weight_depth").as_double();

        grace_frames_         = this->get_parameter("grace_frames").as_int();
        reassign_base_dist_px_       = (float)this->get_parameter("reassign_base_dist_px").as_double();
        reassign_growth_px_per_frame_= (float)this->get_parameter("reassign_growth_px_per_frame").as_double();
        max_reassign_dist_px_ = (float)this->get_parameter("max_reassign_dist_px").as_double();
        lost_reset_frames_    = this->get_parameter("lost_reset_frames").as_int();

        max_speed_xy_mps_ = (float)this->get_parameter("max_speed_xy_mps").as_double();
        max_speed_z_mps_  = (float)this->get_parameter("max_speed_z_mps").as_double();
        ema_alpha_xy_     = (float)this->get_parameter("ema_alpha_xy").as_double();
        ema_alpha_z_      = (float)this->get_parameter("ema_alpha_z").as_double();
        metric_log_period_ = (int)this->get_parameter("metric_log_period").as_int();

        tracker_ = create_tracker(frame_rate, track_buffer,
                                  track_thresh, high_thresh, match_thresh,
                                  std_weight_position, std_weight_velocity, std_weight_depth);
        if (!tracker_) {
            RCLCPP_ERROR(this->get_logger(), "BYTETracker 생성 실패");
            rclcpp::shutdown();
            return;
        }

        // 주인 마지막 위치 초기값: 화면 중앙
        owner_last_cx_ = image_width_  * 0.5f;
        owner_last_cy_ = image_height_ * 0.5f;

        RCLCPP_INFO(this->get_logger(), "BYTETracker 초기화 완료");

        owner_pub_ = this->create_publisher<ros2_tracking_node::msg::OwnerPose>(
            "/owner_pose", 10);

        det_sub_ = this->create_subscription<ros2_tracking_node::msg::DetectionArray>(
            "/detections", 10,
            std::bind(&TrackingNode::detection_callback, this, std::placeholders::_1));

        // RealSense는 BEST_EFFORT로 publish하므로 SensorDataQoS 사용 (0511 H1 수정)
        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", rclcpp::SensorDataQoS(),
            std::bind(&TrackingNode::cam_info_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TrackingNode 시작");
    }

    ~TrackingNode()
    {
        if (tracker_) destroy_tracker(tracker_);
    }

private:
    void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (intrinsic_ready_) return;
        fx_ = (float)msg->k[0];
        fy_ = (float)msg->k[4];
        cx_ = (float)msg->k[2];
        cy_ = (float)msg->k[5];
        intrinsic_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
            "Camera intrinsic: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            fx_, fy_, cx_, cy_);
    }

    // --------------------------------------------------------
    // 마지막 주인 위치에서 가장 가까운 트랙의 ID 반환
    // --------------------------------------------------------
    int find_nearest_to_last(const std::vector<CTrackResult>& tracks, int n)
    {
        if (n <= 0) return -1;   // 0511 H4 가드
        float min_d = std::numeric_limits<float>::max();
        int   best  = tracks[0].track_id;
        for (int i = 0; i < n; i++) {
            float bx = (tracks[i].x1 + tracks[i].x2) * 0.5f;
            float by = (tracks[i].y1 + tracks[i].y2) * 0.5f;
            float d  = (bx - owner_last_cx_) * (bx - owner_last_cx_)
                     + (by - owner_last_cy_) * (by - owner_last_cy_);
            if (d < min_d) { min_d = d; best = tracks[i].track_id; }
        }
        return best;
    }

    float dist_to_last(const CTrackResult& tr)
    {
        float bx = (tr.x1 + tr.x2) * 0.5f;
        float by = (tr.y1 + tr.y2) * 0.5f;
        return std::sqrt((bx - owner_last_cx_) * (bx - owner_last_cx_)
                       + (by - owner_last_cy_) * (by - owner_last_cy_));
    }

    void detection_callback(const ros2_tracking_node::msg::DetectionArray::SharedPtr msg)
    {
        // YOLO detection → CObject 변환
        std::vector<CObject> dets;
        dets.reserve(msg->detections.size());
        for (const auto& d : msg->detections) {
            if (d.label != 0) continue;
            CObject o;
            o.x = d.x; o.y = d.y; o.w = d.w; o.h = d.h;
            o.depth = d.depth; o.label = d.label; o.prob = d.score;
            dets.push_back(o);
        }

        // ByteTrack 업데이트
        std::vector<CTrackResult> results(50);
        int n_tracks = update_tracker(tracker_,
                                      dets.data(), (int)dets.size(),
                                      results.data(), 50);

        // ── OwnerTracker ──────────────────────────────────
        // 1) 주인 미초기화: 첫 트랙 등장 시 화면 중앙 기준 선택
        if (!target_initialized_ && n_tracks > 0) {
            target_id_ = find_nearest_to_last(results, n_tracks);
            target_initialized_ = true;
            lost_count_ = 0;
            RCLCPP_INFO(this->get_logger(), "주인 등록: BT-ID=%d", target_id_);
        }

        // 2) 현재 프레임에서 주인 트랙 탐색
        const CTrackResult* owner_tr = nullptr;
        for (int i = 0; i < n_tracks; i++) {
            if (results[i].track_id == target_id_) {
                owner_tr = &results[i];
                break;
            }
        }

        // 3) 주인 발견 → 위치 갱신
        if (owner_tr) {
            owner_last_cx_ = (owner_tr->x1 + owner_tr->x2) * 0.5f;
            owner_last_cy_ = (owner_tr->y1 + owner_tr->y2) * 0.5f;
            lost_count_ = 0;
        }

        // 4) 주인 Lost → grace_frames 대기 후 위치 기반 재매핑
        if (!owner_tr && target_initialized_) {
            lost_count_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "주인 Lost: %d 프레임", lost_count_);

            // Permanent loss: BYTETracker도 lost 풀에서 잊은 뒤. 옛 픽셀 위치에
            // 묶이지 않게 OwnerTracker를 fresh init 상태로 떨어뜨림.
            // (다음 callback에서 화면 중앙 nearest로 새 owner 자동 선택)
            if (lost_count_ > lost_reset_frames_) {
                RCLCPP_INFO(this->get_logger(),
                    "주인 영구 lost (%d 프레임) → 재초기화 (화면 중앙 nearest로 재선정)",
                    lost_count_);
                m_perma_lost_++;   // [0524 metric] 영구 lost 발생
                target_initialized_ = false;
                target_id_ = -1;
                owner_last_cx_ = image_width_  * 0.5f;
                owner_last_cy_ = image_height_ * 0.5f;
                lost_count_ = 0;
            }
            else if (lost_count_ > grace_frames_ && n_tracks > 0) {
                int  candidate_id   = find_nearest_to_last(results, n_tracks);
                // 후보 트랙 찾기 (dist 계산용)
                float candidate_dist = std::numeric_limits<float>::max();
                for (int i = 0; i < n_tracks; i++) {
                    if (results[i].track_id == candidate_id) {
                        candidate_dist = dist_to_last(results[i]);
                        break;
                    }
                }

                // [0525] 동적 재매핑 거리: lost가 길어질수록 허용 거리를 넓힌다.
                //   eff = base + growth * (lost_count - grace).  상한 없음.
                //   - 짧은 깜빡임(lost 작음): eff 작음 → 옆사람 거부 → KF 외삽이 메움.
                //   - 긴 가림(lost 큼): eff 큼 → 멀어진 진짜 주인도 재매핑 허용.
                float eff_reassign_dist =
                    reassign_base_dist_px_
                    + reassign_growth_px_per_frame_ * (float)(lost_count_ - grace_frames_);

                if (candidate_dist <= eff_reassign_dist) {
                    RCLCPP_WARN(this->get_logger(),
                        "주인 재매핑: BT-ID %d → %d  (거리 %.0fpx ≤ 허용 %.0fpx, lost %d프레임)",
                        target_id_, candidate_id, candidate_dist, eff_reassign_dist, lost_count_);
                    m_id_switches_++;   // [0524 metric] 재매핑 = ID switch
                    target_id_ = candidate_id;
                    lost_count_ = 0;
                    // owner_tr 갱신
                    for (int i = 0; i < n_tracks; i++) {
                        if (results[i].track_id == target_id_) {
                            owner_tr = &results[i];
                            owner_last_cx_ = (owner_tr->x1 + owner_tr->x2) * 0.5f;
                            owner_last_cy_ = (owner_tr->y1 + owner_tr->y2) * 0.5f;
                            break;
                        }
                    }
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "재매핑 후보 너무 멀어 거부 (%.0fpx > 허용 %.0fpx, lost %d프레임) → KF 외삽 유지",
                        candidate_dist, eff_reassign_dist, lost_count_);
                }
            }
        }
        // ── OwnerTracker 끝 ───────────────────────────────

        // publish
        auto out = ros2_tracking_node::msg::OwnerPose();
        out.header.stamp    = msg->header.stamp;
        out.header.frame_id = msg->header.frame_id.empty()
                                ? std::string("camera_color_optical_frame")
                                : msg->header.frame_id;
        out.is_detected = false;

        if (owner_tr) {
            float bx = (owner_tr->x1 + owner_tr->x2) * 0.5f;
            float by = (owner_tr->y1 + owner_tr->y2) * 0.5f;
            float depth_mm = match_track_to_det_depth(*owner_tr, dets);

            // ── Depth Jump Rejection (단순 z만, 1차 보호막) ──
            // EMA 단계에서 더 정밀한 속도 기반 거부를 하니, 여기선 명백한 outlier만.
            // owner_tr->depth (KF z)가 있으니 그걸로 fallback.
            // (구현은 EMA에서 통합 처리. 여기선 raw measurement 그대로 통과.)

            // 0511 H2 수정: intrinsic 없거나 depth invalid면 is_detected=false 유지하고 publish.
            if (intrinsic_ready_ && depth_mm > 1.0f) {
                float Z  = depth_mm * 0.001f;
                float sx = (bx - cx_) * Z / fx_;
                float sy = (by - cy_) * Z / fy_;
                float sz = Z;

                out.is_detected = true;
                out.spatial_x   = sx;
                out.spatial_y   = sy;
                out.spatial_z   = sz;
                out.azimuth     = std::atan2(sx, sz);
                out.distance    = std::sqrt(sx*sx + sz*sz);
                out.confidence  = owner_tr->score;
                out.track_id    = owner_tr->track_id;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "intrinsic 미수신 또는 depth 무효 (intrinsic_ready=%d, depth_mm=%.1f). is_detected=false로 publish.",
                    (int)intrinsic_ready_, depth_mm);
            }
        }

        // ── KF Lost 외삽 ─────────────────────────────────
        // owner_tr 못 찾고(=현재 프레임 detection에 주인 없음) 아직 target 살아있고
        // 재매핑도 안 일어났을 때, ByteTrack 내부 lost 풀에서 KF 외삽값을 가져와서
        // is_detected=true로 부드럽게 메움.
        // (track_buffer=60 동안만 가능. 그 후엔 BYTETracker가 lost에서 제거)
        if (!out.is_detected && target_initialized_ && target_id_ != -1
            && intrinsic_ready_)
        {
            std::vector<CTrackResult> lost_buf(MAX_LOST_BUF);
            int n_lost = get_lost_tracks(tracker_, lost_buf.data(), MAX_LOST_BUF);
            for (int i = 0; i < n_lost; i++) {
                if (lost_buf[i].track_id != target_id_) continue;

                float bx = (lost_buf[i].x1 + lost_buf[i].x2) * 0.5f;
                float by = (lost_buf[i].y1 + lost_buf[i].y2) * 0.5f;
                float depth_mm = lost_buf[i].depth;  // KF z 외삽값

                if (depth_mm > 1.0f) {
                    float Z  = depth_mm * 0.001f;
                    float sx = (bx - cx_) * Z / fx_;
                    float sy = (by - cy_) * Z / fy_;

                    out.is_detected = true;
                    out.spatial_x   = sx;
                    out.spatial_y   = sy;
                    out.spatial_z   = Z;
                    out.azimuth     = std::atan2(sx, Z);
                    out.distance    = std::sqrt(sx*sx + Z*Z);
                    // KF 외삽이라 신뢰도 절반으로 깎음 (downstream에서 구분 가능)
                    out.confidence  = 0.5f * lost_buf[i].score;
                    out.track_id    = target_id_;

                    // 외삽 위치를 last로 갱신해서 재매핑 기준도 따라가게 함
                    owner_last_cx_ = bx;
                    owner_last_cy_ = by;

                    RCLCPP_DEBUG(this->get_logger(),
                        "Lost KF 외삽 사용: id=%d, sxyz=(%.2f,%.2f,%.2f)",
                        target_id_, sx, sy, Z);
                }
                break;
            }
        }
        // ── KF Lost 외삽 끝 ──────────────────────────────

        // ── 속도 기반 Outlier Rejection + EMA 평탄화 ──────
        // 캡스톤 시나리오: 걷는 사람만 추종. 걷는 속도 ~1.5 m/s.
        // 측정값에서 추정된 속도가 max_speed_*_mps를 초과하면 outlier로 간주.
        // 거부 시: 측정 무시하고 직전 평탄화값에 max 속도만큼만 진행 (clip).
        //
        // 다음 단계로 EMA 적용해서 부드럽게.
        // ID 변경/lost 후 재진입 시 시드 보존 (점프 방지).
        if (out.is_detected) {
            rclcpp::Time now = this->now();
            double dt = ema_initialized_
                ? (now - last_pose_time_).seconds()
                : 0.0;
            if (dt <= 0.0 || dt > 1.0) dt = 1.0 / 30.0;   // 비정상 dt 보호

            float meas_x = out.spatial_x;
            float meas_y = out.spatial_y;
            float meas_z = out.spatial_z;

            if (!ema_initialized_) {
                // 첫 측정: 그대로 시드
                ema_x_ = meas_x;
                ema_y_ = meas_y;
                ema_z_ = meas_z;
                ema_initialized_ = true;
            } else {
                // 1) 측정값을 직전 평탄화값 기준으로 속도 clip
                //    (delta = meas - prev). |delta/dt| > max_speed면 max_speed*dt로 clip.
                float clipped_x = clip_by_speed(meas_x, ema_x_, max_speed_xy_mps_, (float)dt);
                float clipped_y = clip_by_speed(meas_y, ema_y_, max_speed_xy_mps_, (float)dt);
                float clipped_z = clip_by_speed(meas_z, ema_z_, max_speed_z_mps_,  (float)dt);

                // clip 발생 시 로깅 (회전 튐 잡혔는지 확인용)
                if (clipped_x != meas_x || clipped_y != meas_y || clipped_z != meas_z) {
                    m_speed_caps_++;   // [0524 metric] 속도 clip 발생 프레임
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Speed cap: meas=(%.2f, %.2f, %.2f) clipped=(%.2f, %.2f, %.2f) dt=%.3fs",
                        meas_x, meas_y, meas_z, clipped_x, clipped_y, clipped_z, dt);
                }

                // 2) EMA로 평탄화
                ema_x_ = ema_alpha_xy_ * clipped_x + (1.f - ema_alpha_xy_) * ema_x_;
                ema_y_ = ema_alpha_xy_ * clipped_y + (1.f - ema_alpha_xy_) * ema_y_;
                ema_z_ = ema_alpha_z_  * clipped_z + (1.f - ema_alpha_z_)  * ema_z_;
            }
            ema_track_id_ = out.track_id;
            ema_lost_streak_ = 0;
            last_pose_time_  = now;

            out.spatial_x = ema_x_;
            out.spatial_y = ema_y_;
            out.spatial_z = ema_z_;
            out.azimuth   = std::atan2(ema_x_, ema_z_);
            out.distance  = std::sqrt(ema_x_ * ema_x_ + ema_z_ * ema_z_);
        } else {
            // Lost: 시드 유지하되 너무 오래 lost면 리셋
            ema_lost_streak_++;
            if (ema_lost_streak_ > ema_lost_reset_frames_) {
                ema_initialized_ = false;
                ema_lost_streak_ = 0;
            }
        }
        // ── 끝 ──────────────────────────────────────────

        // ── [0524] 추적 품질 메트릭 집계 + 주기 로그 ──────────────
        // 출력(out)은 건들지 않고 분류만 함. owner_tr가 살아있으면 hit,
        // 아니더라도 is_detected면 KF 외삽, 둘 다 아니면 lost.
        m_total_frames_++;
        if (owner_tr) {
            m_hit_frames_++;
        } else if (out.is_detected) {
            m_extrap_frames_++;
        } else {
            m_lost_frames_++;
        }

        // 매 프레임 owner 상태 DEBUG (평소는 안 찍힘, --log-level debug 시면)
        if (out.is_detected) {
            RCLCPP_DEBUG(this->get_logger(),
                "owner id=%d dist=%.2fm az=%.1fdeg z=%.2fm conf=%.2f %s",
                out.track_id, out.distance,
                out.azimuth * 180.0f / 3.14159265f, out.spatial_z, out.confidence,
                owner_tr ? "[HIT]" : "[KF]");
        }

        // 주기적 요약 통계 (metric_log_period_ 프레임마다 한 줄)
        if (metric_log_period_ > 0 && (m_total_frames_ % (unsigned long)metric_log_period_) == 0) {
            double tot = (double)m_total_frames_;
            RCLCPP_INFO(this->get_logger(),
                "[metrics] frames=%lu hit=%.0f%% kf=%.0f%% lost=%.0f%% | "
                "id_switch=%lu perma_lost=%lu speed_cap=%lu",
                m_total_frames_,
                100.0 * m_hit_frames_   / tot,
                100.0 * m_extrap_frames_/ tot,
                100.0 * m_lost_frames_  / tot,
                m_id_switches_, m_perma_lost_, m_speed_caps_);
        }
        // ── 메트릭 끝 ──────────────────────────────────

        owner_pub_->publish(out);
    }

    // 측정값을 직전값 기준으로 max_speed*dt 이내로 clip.
    // |meas - prev| / dt > max_speed면 prev에서 max_speed*dt 만큼만 진행.
    static float clip_by_speed(float meas, float prev, float max_speed, float dt)
    {
        float max_delta = max_speed * dt;
        float delta = meas - prev;
        if (delta >  max_delta) return prev + max_delta;
        if (delta < -max_delta) return prev - max_delta;
        return meas;
    }

    float match_track_to_det_depth(const CTrackResult& tr,
                                   const std::vector<CObject>& dets)
    {
        float best_iou = 0.f, best_depth = 0.f;
        for (const auto& d : dets) {
            float ix1 = std::max(tr.x1, d.x);
            float iy1 = std::max(tr.y1, d.y);
            float ix2 = std::min(tr.x2, d.x + d.w);
            float iy2 = std::min(tr.y2, d.y + d.h);
            float iw  = std::max(0.f, ix2 - ix1);
            float ih  = std::max(0.f, iy2 - iy1);
            float inter  = iw * ih;
            float u = (tr.x2-tr.x1)*(tr.y2-tr.y1) + d.w*d.h - inter;
            float iou = (u > 0.f) ? inter / u : 0.f;
            if (iou > best_iou) { best_iou = iou; best_depth = d.depth; }
        }
        return best_depth;
    }

    // ----- 멤버 -----
    static constexpr int MAX_LOST_BUF = 50;   // KF Lost 외삽 조회 버퍼 크기

    // ----- 추적 품질 메트릭 (0524 추가, 로깅 전용 / 동작 무관) -----
    // 추후 IoU 게이팅 결합 등 설계 결정을 데이터로 내리기 위한 누적 통계.
    unsigned long m_total_frames_   = 0;  // 콜백 호출(=프레임) 누적
    unsigned long m_hit_frames_     = 0;  // owner를 현재 detection에서 직접 찾은 프레임
    unsigned long m_extrap_frames_  = 0;  // KF lost 외삽으로 메운 프레임
    unsigned long m_lost_frames_    = 0;  // is_detected=false로 나간 프레임
    unsigned long m_id_switches_    = 0;  // 재매핑으로 target_id_가 바뀐 횟수
    unsigned long m_perma_lost_     = 0;  // 영구 lost(재초기화) 발생 횟수
    unsigned long m_speed_caps_     = 0;  // 속도 clip이 걸린 프레임
    int  metric_log_period_ = 150;        // 요약 로그 주기(프레임). 30fps 기준 5초

    void* tracker_           = nullptr;
    int   target_id_         = -1;
    bool  target_initialized_= false;
    int   lost_count_        = 0;
    int   grace_frames_      = 10;
    float reassign_base_dist_px_        = 50.f;  // [0525] 동적 재매핑 거리 base (좁게)
    float reassign_growth_px_per_frame_ = 5.f;   // [0525] lost 1프레임당 거리 증가 (천천히)
    float max_reassign_dist_px_ = 300.f;         // [deprecated] 미사용
    int   lost_reset_frames_ = 60;

    // Outlier rejection (속도 기반)
    float max_speed_xy_mps_ = 2.0f;   // 좌우 최대 속도 (걷는 사람 ~1.5)
    float max_speed_z_mps_  = 2.0f;   // 전후 최대 속도

    // EMA 평탄화 상태
    bool  ema_initialized_ = false;
    int   ema_track_id_    = -1;
    float ema_x_ = 0.f, ema_y_ = 0.f, ema_z_ = 0.f;
    float ema_alpha_xy_    = 0.35f;
    float ema_alpha_z_     = 0.15f;
    int   ema_lost_streak_       = 0;
    int   ema_lost_reset_frames_ = 30;   // 30프레임 (1초) 이상 lost면 EMA 리셋
    rclcpp::Time last_pose_time_;        // 속도 추정용

    // 주인 마지막 픽셀 위치 (재매핑 기준)
    float owner_last_cx_     = 0.f;
    float owner_last_cy_     = 0.f;

    int   image_width_       = 640;
    int   image_height_      = 480;

    bool  intrinsic_ready_   = false;
    float fx_ = 0.f, fy_ = 0.f, cx_ = 0.f, cy_ = 0.f;

    rclcpp::Publisher<ros2_tracking_node::msg::OwnerPose>::SharedPtr owner_pub_;
    rclcpp::Subscription<ros2_tracking_node::msg::DetectionArray>::SharedPtr det_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackingNode>());
    rclcpp::shutdown();
    return 0;
}
