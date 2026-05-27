#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <future>
#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <robotiq_2f_urcap_adapter/action/gripper_command.hpp>

class EEPoseCommander : public rclcpp::Node
{
public:
  using GripperCommand = robotiq_2f_urcap_adapter::action::GripperCommand;

  explicit EEPoseCommander(const rclcpp::NodeOptions& options)
  : Node("ee_pose_commander", options)
  {
    /*
      NEW BOARD CALIBRATION METHOD:

      The board is now calculated relative to chess_board_frame.

      Assumption:
      - chess_board_frame is located at the A1 corner of the board.
      - +X direction goes from A file to H file.
      - +Y direction goes from rank 1 to rank 8.
      - +Z direction points upward from the board.

      So:
      A1 = near x = square_size/2, y = square_size/2
      H8 = near x = board_size - square_size/2,
           y = board_size - square_size/2
    */
    declare_parameter("board_size", 0.50);
    declare_parameter("pick_z", 0.2);

    /*
      End-effector orientation.
      This orientation points the tool down toward the board.
    */
    declare_parameter("qx", -0.010);
    declare_parameter("qy",  1.000);
    declare_parameter("qz",  0.002);
    declare_parameter("qw", -0.015);

    /*
      Safe height above each square.
    */
    declare_parameter("lift_z", 0.2);

    /*
      MoveIt setup.
    */
    declare_parameter("planning_group", "ur5e_workcell_manipulator");
    declare_parameter("pose_reference_frame", "chess_board_frame");
    declare_parameter("end_effector_link", "tool0");
    declare_parameter("home_named_target", "up");

    declare_parameter("position_tolerance", 0.01);
    declare_parameter("orientation_tolerance", 1.57);
    declare_parameter("planning_time", 10.0);
    declare_parameter("num_planning_attempts", 10);
    declare_parameter("velocity_scale", 0.50);
    declare_parameter("acceleration_scale", 0.50);
    declare_parameter("allow_replanning", true);

    declare_parameter("startup_delay_sec", 2.0);
    declare_parameter("state_wait_sec", 10.0);

    /*
      Bezier / Cartesian path settings.
    */
    declare_parameter("bezier_points", 18);
    declare_parameter("eef_step", 0.008);
    declare_parameter("cartesian_fraction_min", 0.70);
    declare_parameter("vertical_handle_ratio", 0.30);
    declare_parameter("peak_extra_z", 0.010);

    /*
      true  -> robot returns to named target "up" after every move
      false -> robot stops above destination square
    */
    declare_parameter("return_home_after_move", true);

    /*
      Topic input from game engine.
      Game engine should publish strings like:
      e2e4
      g1f3
      h6h8
    */
    declare_parameter("chess_move_topic", "/chess_move");
    declare_parameter("robot_status_topic", "/robot_status");
    declare_parameter("wait_for_move_sec", 60.0);

    // CHANGE: FEN topic from the interface/game state.
    // The code uses this to check whether the destination square is occupied
    // before physically executing the robot engine move.
    declare_parameter("board_fen_topic", "/interface/board_fen");

    // CHANGE: startup calibration confirmation topic.
    // The robot moves to A1 first, then waits until this topic receives "done".
    declare_parameter("startup_calibration_enabled", true);
    declare_parameter("calibration_done_topic", "/calibration_done");

    // CHANGE: captured-piece drop pose.
    // The pose you gave is expressed in the robot base frame.
    declare_parameter("capture_drop_frame", "base_link");
    declare_parameter("capture_drop_x", -0.162);
    declare_parameter("capture_drop_y", -0.055);
    declare_parameter("capture_drop_z", 1.207);
    declare_parameter("capture_drop_qx", 0.999);
    declare_parameter("capture_drop_qy", -0.033);
    declare_parameter("capture_drop_qz", -0.023);
    declare_parameter("capture_drop_qw", 0.014);
    declare_parameter("capture_drop_lift_z", 0.060);

    /*
      Gripper settings.
    */
    declare_parameter("use_gripper", true);
    declare_parameter("gripper_action_name", "/robotiq_2f_urcap_adapter/gripper_command");

    declare_parameter("gripper_open_position", 0.050);
    declare_parameter("gripper_close_position", 0.037);
    declare_parameter("gripper_start_open_position", 0.050);

    declare_parameter("gripper_max_speed", 0.10);
    declare_parameter("gripper_max_effort", 50.0);

    declare_parameter("gripper_wait_sec", 0.5);
    declare_parameter("gripper_server_wait_sec", 5.0);

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this,
      get_parameter("gripper_action_name").as_string());

    const std::string chess_move_topic =
      get_parameter("chess_move_topic").as_string();

    move_sub_ = create_subscription<std_msgs::msg::String>(
      chess_move_topic,
      10,
      std::bind(&EEPoseCommander::moveCallback, this, std::placeholders::_1));

    // CHANGE: subscribe to board FEN updates.
    const std::string board_fen_topic =
      get_parameter("board_fen_topic").as_string();

    board_fen_sub_ = create_subscription<std_msgs::msg::String>(
      board_fen_topic,
      10,
      std::bind(&EEPoseCommander::boardFenCallback, this, std::placeholders::_1));

    // CHANGE: subscribe to calibration confirmation topic.
    const std::string calibration_done_topic =
      get_parameter("calibration_done_topic").as_string();

    calibration_done_sub_ = create_subscription<std_msgs::msg::String>(
      calibration_done_topic,
      10,
      std::bind(&EEPoseCommander::calibrationDoneCallback, this, std::placeholders::_1));

    const std::string robot_status_topic =
      get_parameter("robot_status_topic").as_string();

    robot_status_pub_ = create_publisher<std_msgs::msg::String>(
      robot_status_topic,
      10);

    RCLCPP_INFO(
      get_logger(),
      "Subscribed to chess move topic: %s",
      chess_move_topic.c_str());

    // CHANGE:
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to board FEN topic: %s",
      board_fen_topic.c_str());

    // CHANGE:
    RCLCPP_INFO(
      get_logger(),
      "Subscribed to calibration done topic: %s",
      calibration_done_topic.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Publishing robot status on topic: %s",
      robot_status_topic.c_str());
  }

private:
  struct ChessMove
  {
    std::string from;
    std::string to;
  };

  struct Vec3
  {
    double x{};
    double y{};
    double z{};
  };

  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_sub_;

  // CHANGE: latest FEN history from /interface/board_fen.
  // We keep a few recent FENs because the engine publishes two FENs quickly:
  //   1) after human move / before robot move  -> source square is still occupied
  //   2) after robot move in engine state       -> source square becomes empty
  // When a robot move arrives, we choose the newest FEN where move.from is occupied.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr board_fen_sub_;
  std::mutex fen_mutex_;
  std::vector<std::string> recent_fens_;

  // CHANGE: calibration confirmation topic state.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr calibration_done_sub_;
  std::mutex calibration_mutex_;
  bool calibration_done_received_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
  std::mutex move_mutex_;
  std::optional<ChessMove> latest_move_;
  bool move_received_{false};

  void publishRobotStatus(const std::string& status)
  {
    std_msgs::msg::String msg;
    msg.data = status;

    robot_status_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Published robot status: %s",
      status.c_str());
  }

  // CHANGE: store recent board states.
  // Do not decide capture here; decision is made only when a robot move arrives.
  void boardFenCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(fen_mutex_);

    recent_fens_.push_back(msg->data);

    if (recent_fens_.size() > 6)
      recent_fens_.erase(recent_fens_.begin());

    RCLCPP_INFO(
      get_logger(),
      "Received board FEN: %s",
      msg->data.c_str());
  }

  void moveCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    ChessMove parsed_move;

    try
    {
      parsed_move = parseMoveString(msg->data);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(
        get_logger(),
        "Invalid chess move received: '%s'. Error: %s",
        msg->data.c_str(),
        e.what());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      latest_move_ = parsed_move;
      move_received_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Received chess move from topic: %s -> %s",
      parsed_move.from.c_str(),
      parsed_move.to.c_str());
  }

  bool waitForChessMove(ChessMove& move_out, double timeout_sec)
  {
    const auto start = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Waiting for chess move on topic...");

    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(move_mutex_);

        if (move_received_ && latest_move_.has_value())
        {
          move_out = latest_move_.value();
          move_received_ = false;
          return true;
        }
      }

      const auto now = std::chrono::steady_clock::now();

      const double elapsed =
        std::chrono::duration_cast<std::chrono::duration<double>>(
          now - start).count();

      if (elapsed > timeout_sec)
        return false;

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
  }

  static double lerp(double a, double b, double t)
  {
    return (1.0 - t) * a + t * b;
  }

  static Vec3 lerpVec3(const Vec3& a, const Vec3& b, double t)
  {
    return {
      lerp(a.x, b.x, t),
      lerp(a.y, b.y, t),
      lerp(a.z, b.z, t)
    };
  }

  static Vec3 cubicBezierVec3(
    const Vec3& p0,
    const Vec3& p1,
    const Vec3& p2,
    const Vec3& p3,
    double t)
  {
    const Vec3 a = lerpVec3(p0, p1, t);
    const Vec3 b = lerpVec3(p1, p2, t);
    const Vec3 c = lerpVec3(p2, p3, t);

    const Vec3 d = lerpVec3(a, b, t);
    const Vec3 e = lerpVec3(b, c, t);

    return lerpVec3(d, e, t);
  }

  static Vec3 poseToVec3(const geometry_msgs::msg::Pose& p)
  {
    return {p.position.x, p.position.y, p.position.z};
  }

  static geometry_msgs::msg::Pose vec3ToPose(
    const Vec3& v,
    const geometry_msgs::msg::Pose& reference_pose)
  {
    auto p = reference_pose;

    p.position.x = v.x;
    p.position.y = v.y;
    p.position.z = v.z;

    return p;
  }

  static void appendSegment(
    std::vector<geometry_msgs::msg::Pose>& full_path,
    const std::vector<geometry_msgs::msg::Pose>& segment)
  {
    if (segment.empty())
      return;

    if (full_path.empty())
    {
      full_path.insert(full_path.end(), segment.begin(), segment.end());
    }
    else
    {
      full_path.insert(full_path.end(), segment.begin() + 1, segment.end());
    }
  }

  geometry_msgs::msg::Pose makePose(
    double x,
    double y,
    double z,
    double qx,
    double qy,
    double qz,
    double qw)
  {
    geometry_msgs::msg::Pose p;

    p.position.x = x;
    p.position.y = y;
    p.position.z = z;

    p.orientation.x = qx;
    p.orientation.y = qy;
    p.orientation.z = qz;
    p.orientation.w = qw;

    return p;
  }

  bool squareToRowCol(const std::string& square, int& row, int& col)
  {
    if (square.size() != 2)
      return false;

    const char file = static_cast<char>(
      std::toupper(static_cast<unsigned char>(square[0])));

    const char rank = square[1];

    if (file < 'A' || file > 'H')
      return false;

    if (rank < '1' || rank > '8')
      return false;

    col = file - 'A';
    row = rank - '1';

    return true;
  }


  // CHANGE: receives the confirmation that calibration is finished.
  // Publish this from terminal using:
  // ros2 topic pub --once /calibration_done std_msgs/msg/String "{data: 'done'}"
  void calibrationDoneCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != "done")
    {
      RCLCPP_WARN(
        get_logger(),
        "Calibration topic received '%s'. Send exactly 'done' to continue.",
        msg->data.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(calibration_mutex_);
      calibration_done_received_ = true;
    }

    RCLCPP_INFO(get_logger(), "Received calibration confirmation: done");
  }

  // CHANGE: waits until /calibration_done receives the word "done".
  bool waitForCalibrationDone()
  {
    RCLCPP_INFO(
      get_logger(),
      "Waiting for calibration confirmation. Publish 'done' on /calibration_done to continue.");

    while (rclcpp::ok())
    {
      {
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        if (calibration_done_received_)
        {
          calibration_done_received_ = false;
          return true;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
  }
  // CHANGE: returns true if a square is occupied in a given FEN string.
  bool isSquareOccupiedInFen(const std::string& square, const std::string& fen)
  {
    int row = 0;
    int col = 0;

    if (!squareToRowCol(square, row, col))
      throw std::runtime_error("Invalid chess square for FEN check: " + square);

    // FEN placement is written from rank 8 down to rank 1.
    // Our row is rank 1 -> 0, rank 8 -> 7.
    const int target_fen_rank = 7 - row;
    const int target_file = col;

    std::string placement = fen;
    const auto first_space = placement.find(' ');
    if (first_space != std::string::npos)
      placement = placement.substr(0, first_space);

    int fen_rank = 0;
    int file = 0;

    for (char c : placement)
    {
      if (c == '/')
      {
        fen_rank++;
        file = 0;
        continue;
      }

      if (std::isdigit(static_cast<unsigned char>(c)))
      {
        file += c - '0';
        continue;
      }

      if (std::isalpha(static_cast<unsigned char>(c)))
      {
        if (fen_rank == target_fen_rank && file == target_file)
          return true;

        file++;
        continue;
      }
    }

    return false;
  }

  // CHANGE: choose the correct FEN for the robot move.
  // The correct FEN is the newest one where move.from is still occupied.
  // That means it is before the robot physically moves, not after the engine
  // already applied the robot move in game state.
  std::optional<std::string> chooseFenBeforeRobotMove(const ChessMove& move)
  {
    std::lock_guard<std::mutex> lock(fen_mutex_);

    for (auto it = recent_fens_.rbegin(); it != recent_fens_.rend(); ++it)
    {
      try
      {
        if (isSquareOccupiedInFen(move.from, *it))
          return *it;
      }
      catch (const std::exception& e)
      {
        RCLCPP_WARN(
          get_logger(),
          "Skipping invalid FEN while choosing pre-robot FEN: %s",
          e.what());
      }
    }

    return std::nullopt;
  }

  ChessMove parseMoveString(const std::string& raw_input)
  {
    std::string s;
    s.reserve(raw_input.size());

    for (char c : raw_input)
    {
      if (!std::isspace(static_cast<unsigned char>(c)))
      {
        s.push_back(static_cast<char>(
          std::tolower(static_cast<unsigned char>(c))));
      }
    }

    /*
      Accepts:
      e2e4
      E2E4
      "e2e4"

      If promotion is received like e7e8q,
      the robot motion still uses e7 -> e8.
    */
    if (s.size() < 4)
      throw std::runtime_error("Invalid move format. Use format like e2e4.");

    ChessMove move;
    move.from = s.substr(0, 2);
    move.to   = s.substr(2, 2);

    int row = 0;
    int col = 0;

    if (!squareToRowCol(move.from, row, col))
      throw std::runtime_error("Invalid source square: " + move.from);

    if (!squareToRowCol(move.to, row, col))
      throw std::runtime_error("Invalid target square: " + move.to);

    return move;
  }

  geometry_msgs::msg::Pose squareToPose(
    const std::string& square,
    double board_size,
    double pick_z,
    double qx,
    double qy,
    double qz,
    double qw)
  {
    int row = 0;
    int col = 0;

    if (!squareToRowCol(square, row, col))
      throw std::runtime_error("Invalid chess square: " + square);

    /*
      Frame-based board calculation.

      The pose is calculated relative to chess_board_frame.

      A1:
        col = 0
        row = 0

      H8:
        col = 7
        row = 7

      square_size = board_size / 8

      Square center:
        x = square_size/2 + col * square_size
        y = square_size/2 + row * square_size
    */
    const double square_size = board_size / 8.0;

    const double x =
      square_size / 2.0 + static_cast<double>(col) * square_size;

    const double y =
      square_size / 2.0 + static_cast<double>(row) * square_size;

    return makePose(x, y, pick_z, qx, qy, qz, qw);
  }

  std::vector<geometry_msgs::msg::Pose> sampleCubicBezier(
    const geometry_msgs::msg::Pose& p0,
    const geometry_msgs::msg::Pose& p1,
    const geometry_msgs::msg::Pose& p2,
    const geometry_msgs::msg::Pose& p3,
    int num_points)
  {
    num_points = std::max(num_points, 2);

    std::vector<geometry_msgs::msg::Pose> output;
    output.reserve(num_points);

    const Vec3 v0 = poseToVec3(p0);
    const Vec3 v1 = poseToVec3(p1);
    const Vec3 v2 = poseToVec3(p2);
    const Vec3 v3 = poseToVec3(p3);

    for (int i = 0; i < num_points; ++i)
    {
      const double t =
        static_cast<double>(i) / static_cast<double>(num_points - 1);

      output.push_back(
        vec3ToPose(cubicBezierVec3(v0, v1, v2, v3, t), p3));
    }

    return output;
  }

  std::vector<geometry_msgs::msg::Pose> buildVerticalBezier(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    int num_points,
    double handle_ratio)
  {
    auto p0 = start_pose;
    auto p1 = start_pose;
    auto p2 = end_pose;
    auto p3 = end_pose;

    const double dz = end_pose.position.z - start_pose.position.z;
    const double handle = std::fabs(dz) * handle_ratio;

    p1.position.z = start_pose.position.z + (dz >= 0.0 ? handle : -handle);
    p2.position.z = end_pose.position.z - (dz >= 0.0 ? handle : -handle);

    p1.orientation = end_pose.orientation;
    p2.orientation = end_pose.orientation;
    p3.orientation = end_pose.orientation;

    return sampleCubicBezier(p0, p1, p2, p3, num_points);
  }

  std::vector<geometry_msgs::msg::Pose> buildAboveToAboveBezier(
    const geometry_msgs::msg::Pose& start_above,
    const geometry_msgs::msg::Pose& end_above,
    int num_points,
    double extra_peak_z)
  {
    auto p0 = start_above;
    auto p1 = start_above;
    auto p2 = end_above;
    auto p3 = end_above;

    const double dx = end_above.position.x - start_above.position.x;
    const double dy = end_above.position.y - start_above.position.y;

    const double peak_z =
      std::max(start_above.position.z, end_above.position.z) + extra_peak_z;

    p1.position.x = start_above.position.x + 0.25 * dx;
    p1.position.y = start_above.position.y + 0.25 * dy;
    p1.position.z = peak_z;

    p2.position.x = start_above.position.x + 0.75 * dx;
    p2.position.y = start_above.position.y + 0.75 * dy;
    p2.position.z = peak_z;

    p1.orientation = end_above.orientation;
    p2.orientation = end_above.orientation;
    p3.orientation = end_above.orientation;

    return sampleCubicBezier(p0, p1, p2, p3, num_points);
  }

  bool moveToNamedTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& target_name)
  {
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    if (!move_group.setNamedTarget(target_name))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Named target '%s' does not exist.",
        target_name.c_str());

      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to plan to named target '%s'.",
        target_name.c_str());

      return false;
    }

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to execute named target '%s'.",
        target_name.c_str());

      return false;
    }

    return true;
  }

  bool moveToPoseTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& end_effector_link)
  {
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.clearPathConstraints();

    move_group.setApproximateJointValueTarget(target_pose, end_effector_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group.plan(plan);

    move_group.clearPoseTargets();

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to plan to pose target: x=%.3f, y=%.3f, z=%.3f",
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);

      return false;
    }

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to execute pose target.");
      return false;
    }

    return true;
  }

  // CHANGE: move to a pose expressed in a specific reference frame, then restore the normal frame.
  bool moveToPoseTargetInFrame(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& frame_id,
    const std::string& normal_reference_frame,
    const std::string& end_effector_link)
  {
    move_group.setPoseReferenceFrame(frame_id);

    const bool ok = moveToPoseTarget(
      move_group,
      target_pose,
      end_effector_link);

    move_group.setPoseReferenceFrame(normal_reference_frame);

    return ok;
  }

  bool executeCartesianPath(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& path,
    double eef_step,
    double min_fraction,
    double velocity_scale,
    double acceleration_scale)
  {
    if (path.size() < 2)
    {
      RCLCPP_ERROR(get_logger(), "Cartesian path has less than 2 points.");
      return false;
    }

    move_group.setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory trajectory_msg;

    const double jump_threshold = 0.0;

    const double fraction = move_group.computeCartesianPath(
      path,
      eef_step,
      jump_threshold,
      trajectory_msg);

    if (fraction < min_fraction)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Cartesian path incomplete. Fraction = %.3f, required = %.3f",
        fraction,
        min_fraction);

      return false;
    }

    auto current_state = move_group.getCurrentState();

    if (!current_state)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Could not get current state for time parameterization.");

      return false;
    }

    robot_trajectory::RobotTrajectory robot_traj(
      move_group.getRobotModel(),
      move_group.getName());

    robot_traj.setRobotTrajectoryMsg(*current_state, trajectory_msg);

    trajectory_processing::IterativeParabolicTimeParameterization time_param;

    if (!time_param.computeTimeStamps(
          robot_traj,
          velocity_scale,
          acceleration_scale))
    {
      RCLCPP_ERROR(get_logger(), "Time parameterization failed.");
      return false;
    }

    robot_traj.getRobotTrajectoryMsg(trajectory_msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory_msg;

    if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Cartesian path execution failed.");
      return false;
    }

    return true;
  }

  // CHANGE: execute a Cartesian path whose waypoint poses are expressed in a specific frame.
  bool executeCartesianPathInFrame(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& path,
    const std::string& frame_id,
    const std::string& normal_reference_frame,
    double eef_step,
    double min_fraction,
    double velocity_scale,
    double acceleration_scale)
  {
    move_group.setPoseReferenceFrame(frame_id);

    const bool ok = executeCartesianPath(
      move_group,
      path,
      eef_step,
      min_fraction,
      velocity_scale,
      acceleration_scale);

    move_group.setPoseReferenceFrame(normal_reference_frame);

    return ok;
  }

  bool waitForGripperServer()
  {
    const bool use_gripper = get_parameter("use_gripper").as_bool();

    if (!use_gripper)
    {
      RCLCPP_INFO(get_logger(), "[GRIPPER DISABLED] Skipping gripper server wait.");
      return true;
    }

    const double wait_sec =
      get_parameter("gripper_server_wait_sec").as_double();

    RCLCPP_INFO(get_logger(), "Waiting for gripper action server...");

    if (!gripper_client_->wait_for_action_server(
          std::chrono::duration<double>(wait_sec)))
    {
      RCLCPP_ERROR(get_logger(), "Gripper action server not available.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Gripper action server is available.");
    return true;
  }

  bool sendGripperCommand(double position)
  {
    const bool use_gripper = get_parameter("use_gripper").as_bool();

    if (!use_gripper)
    {
      RCLCPP_INFO(
        get_logger(),
        "[GRIPPER DISABLED] Would send gripper position %.3f",
        position);
      return true;
    }

    GripperCommand::Goal goal_msg;

    goal_msg.command.position = position;

    goal_msg.command.max_speed =
      get_parameter("gripper_max_speed").as_double();

    goal_msg.command.max_effort =
      get_parameter("gripper_max_effort").as_double();

    RCLCPP_INFO(
      get_logger(),
      "Sending gripper command: position=%.3f",
      position);

    auto send_goal_future =
      gripper_client_->async_send_goal(goal_msg);

    if (send_goal_future.wait_for(std::chrono::seconds(5)) !=
        std::future_status::ready)
    {
      RCLCPP_WARN(
        get_logger(),
        "Timed out while sending gripper goal. Continuing robot motion anyway.");

      std::this_thread::sleep_for(
        std::chrono::duration<double>(
          get_parameter("gripper_wait_sec").as_double()));

      return true;
    }

    auto goal_handle = send_goal_future.get();

    if (!goal_handle)
    {
      RCLCPP_WARN(
        get_logger(),
        "Gripper goal was rejected. Continuing robot motion anyway.");

      std::this_thread::sleep_for(
        std::chrono::duration<double>(
          get_parameter("gripper_wait_sec").as_double()));

      return true;
    }

    /*
      IMPORTANT FIX:

      We do NOT call async_get_result() here anymore.

      Reason:
      The Robotiq gripper can physically open/close correctly, but the action
      server may still report ABORTED / not SUCCEEDED. If we wait for that
      result and return false, the chess motion stops before moving back up
      and before returning home.

      For this project, once the gripper goal is accepted, we wait briefly for
      the physical gripper movement, then continue the robot sequence.
    */
    RCLCPP_INFO(
      get_logger(),
      "Gripper goal accepted. Fire-and-continue mode: waiting briefly, then continuing robot motion.");

    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        get_parameter("gripper_wait_sec").as_double()));

    return true;
  }

  // CHANGE: remove a piece already occupying the destination square.
  // This is used before the normal engine move is executed.
  bool removeOccupiedPieceFromSquare(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::string& normal_reference_frame,
    const std::string& occupied_square,
    const std::string& end_effector_link,
    double board_size,
    double pick_z,
    double qx,
    double qy,
    double qz,
    double qw,
    double lift_z,
    int bezier_points,
    double eef_step,
    double min_fraction,
    double vertical_handle_ratio,
    double velocity_scale,
    double acceleration_scale)
  {
    RCLCPP_INFO(
      get_logger(),
      "Removing occupied piece from %s before normal robot move.",
      occupied_square.c_str());

    const auto occupied_pose = squareToPose(
      occupied_square,
      board_size,
      pick_z,
      qx,
      qy,
      qz,
      qw);

    auto occupied_above = occupied_pose;
    occupied_above.position.z += lift_z;

    if (!moveToPoseTarget(move_group, occupied_above, end_effector_link))
      return false;

    std::vector<geometry_msgs::msg::Pose> path;

    appendSegment(
      path,
      buildVerticalBezier(
        occupied_above,
        occupied_pose,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPath(
          move_group,
          path,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    if (!sendGripperCommand(get_parameter("gripper_close_position").as_double()))
      return false;

    path.clear();

    appendSegment(
      path,
      buildVerticalBezier(
        occupied_pose,
        occupied_above,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPath(
          move_group,
          path,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    const std::string capture_drop_frame =
      get_parameter("capture_drop_frame").as_string();

    const double capture_drop_x = get_parameter("capture_drop_x").as_double();
    const double capture_drop_y = get_parameter("capture_drop_y").as_double();
    const double capture_drop_z = get_parameter("capture_drop_z").as_double();
    const double capture_drop_lift_z = get_parameter("capture_drop_lift_z").as_double();

    double capture_drop_qx = get_parameter("capture_drop_qx").as_double();
    double capture_drop_qy = get_parameter("capture_drop_qy").as_double();
    double capture_drop_qz = get_parameter("capture_drop_qz").as_double();
    double capture_drop_qw = get_parameter("capture_drop_qw").as_double();

    tf2::Quaternion drop_q(
      capture_drop_qx,
      capture_drop_qy,
      capture_drop_qz,
      capture_drop_qw);
    drop_q.normalize();

    const auto drop_pose = makePose(
      capture_drop_x,
      capture_drop_y,
      capture_drop_z,
      drop_q.x(),
      drop_q.y(),
      drop_q.z(),
      drop_q.w());

    auto drop_above = drop_pose;
    drop_above.position.z += capture_drop_lift_z;

    RCLCPP_INFO(
      get_logger(),
      "Capture drop pose in %s: x=%.3f, y=%.3f, z=%.3f",
      capture_drop_frame.c_str(),
      drop_pose.position.x,
      drop_pose.position.y,
      drop_pose.position.z);

    if (!moveToPoseTargetInFrame(
          move_group,
          drop_above,
          capture_drop_frame,
          normal_reference_frame,
          end_effector_link))
    {
      return false;
    }

    path.clear();

    appendSegment(
      path,
      buildVerticalBezier(
        drop_above,
        drop_pose,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPathInFrame(
          move_group,
          path,
          capture_drop_frame,
          normal_reference_frame,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    if (!sendGripperCommand(get_parameter("gripper_open_position").as_double()))
      return false;

    path.clear();

    appendSegment(
      path,
      buildVerticalBezier(
        drop_pose,
        drop_above,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPathInFrame(
          move_group,
          path,
          capture_drop_frame,
          normal_reference_frame,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    move_group.setPoseReferenceFrame(normal_reference_frame);
    return true;
  }

  bool executeChessMove(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const ChessMove& move,
    const std::string& end_effector_link,
    double board_size,
    double pick_z,
    double qx,
    double qy,
    double qz,
    double qw,
    double lift_z,
    int bezier_points,
    double eef_step,
    double min_fraction,
    double vertical_handle_ratio,
    double peak_extra_z,
    double velocity_scale,
    double acceleration_scale)
  {
    const auto from_pose = squareToPose(
      move.from,
      board_size,
      pick_z,
      qx,
      qy,
      qz,
      qw);

    const auto to_pose = squareToPose(
      move.to,
      board_size,
      pick_z,
      qx,
      qy,
      qz,
      qw);

    auto from_above = from_pose;
    from_above.position.z += lift_z;

    auto to_above = to_pose;
    to_above.position.z += lift_z;

    RCLCPP_INFO(
      get_logger(),
      "Executing chess move: %s -> %s",
      move.from.c_str(),
      move.to.c_str());

    RCLCPP_INFO(
      get_logger(),
      "%s pick pose in chess_board_frame:  x=%.3f, y=%.3f, z=%.3f",
      move.from.c_str(),
      from_pose.position.x,
      from_pose.position.y,
      from_pose.position.z);

    RCLCPP_INFO(
      get_logger(),
      "%s place pose in chess_board_frame: x=%.3f, y=%.3f, z=%.3f",
      move.to.c_str(),
      to_pose.position.x,
      to_pose.position.y,
      to_pose.position.z);

    RCLCPP_INFO(
      get_logger(),
      "%s above pose in chess_board_frame: x=%.3f, y=%.3f, z=%.3f",
      move.from.c_str(),
      from_above.position.x,
      from_above.position.y,
      from_above.position.z);

    RCLCPP_INFO(
      get_logger(),
      "%s above pose in chess_board_frame: x=%.3f, y=%.3f, z=%.3f",
      move.to.c_str(),
      to_above.position.x,
      to_above.position.y,
      to_above.position.z);

    /*
      1) Move safely above source square.
    */
    if (!moveToPoseTarget(move_group, from_above, end_effector_link))
      return false;

    /*
      2) Smoothly move down to pick pose.
    */
    std::vector<geometry_msgs::msg::Pose> path;

    appendSegment(
      path,
      buildVerticalBezier(
        from_above,
        from_pose,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPath(
          move_group,
          path,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    /*
      3) Close gripper.
    */
    if (!sendGripperCommand(get_parameter("gripper_close_position").as_double()))
      return false;

    /*
      4) Smoothly move:
         pick pose -> above source -> above target -> target pose
    */
    path.clear();

    appendSegment(
      path,
      buildVerticalBezier(
        from_pose,
        from_above,
        bezier_points,
        vertical_handle_ratio));

    appendSegment(
      path,
      buildAboveToAboveBezier(
        from_above,
        to_above,
        bezier_points,
        peak_extra_z));

    appendSegment(
      path,
      buildVerticalBezier(
        to_above,
        to_pose,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPath(
          move_group,
          path,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    /*
      5) Open gripper.
    */
    if (!sendGripperCommand(get_parameter("gripper_open_position").as_double()))
      return false;

    /*
      6) Smoothly move back above destination.
    */
    path.clear();

    appendSegment(
      path,
      buildVerticalBezier(
        to_pose,
        to_above,
        bezier_points,
        vertical_handle_ratio));

    if (!executeCartesianPath(
          move_group,
          path,
          eef_step,
          min_fraction,
          velocity_scale,
          acceleration_scale))
    {
      return false;
    }

    return true;
  }

public:
  void run()
  {
    const double startup_delay_sec =
      get_parameter("startup_delay_sec").as_double();

    const double state_wait_sec =
      get_parameter("state_wait_sec").as_double();

    const double wait_for_move_sec =
      get_parameter("wait_for_move_sec").as_double();

    const std::string planning_group =
      get_parameter("planning_group").as_string();

    const std::string pose_reference_frame =
      get_parameter("pose_reference_frame").as_string();

    const std::string end_effector_link =
      get_parameter("end_effector_link").as_string();

    const std::string home_named_target =
      get_parameter("home_named_target").as_string();

    const double board_size =
      get_parameter("board_size").as_double();

    const double pick_z =
      get_parameter("pick_z").as_double();

    double qx = get_parameter("qx").as_double();
    double qy = get_parameter("qy").as_double();
    double qz = get_parameter("qz").as_double();
    double qw = get_parameter("qw").as_double();

    const double lift_z =
      get_parameter("lift_z").as_double();

    const double planning_time =
      get_parameter("planning_time").as_double();

    const int num_planning_attempts =
      get_parameter("num_planning_attempts").as_int();

    const double velocity_scale =
      get_parameter("velocity_scale").as_double();

    const double acceleration_scale =
      get_parameter("acceleration_scale").as_double();

    const double position_tolerance =
      get_parameter("position_tolerance").as_double();

    const double orientation_tolerance =
      get_parameter("orientation_tolerance").as_double();

    const bool allow_replanning =
      get_parameter("allow_replanning").as_bool();

    const int bezier_points =
      get_parameter("bezier_points").as_int();

    const double eef_step =
      get_parameter("eef_step").as_double();

    const double min_fraction =
      get_parameter("cartesian_fraction_min").as_double();

    const double vertical_handle_ratio =
      get_parameter("vertical_handle_ratio").as_double();

    const double peak_extra_z =
      get_parameter("peak_extra_z").as_double();

    const bool return_home_after_move =
      get_parameter("return_home_after_move").as_bool();

    // CHANGE: startup calibration switch.
    const bool startup_calibration_enabled =
      get_parameter("startup_calibration_enabled").as_bool();

    /*
      Normalize quaternion to avoid invalid orientation issues.
    */
    tf2::Quaternion q(qx, qy, qz, qw);
    q.normalize();

    qx = q.x();
    qy = q.y();
    qz = q.z();
    qw = q.w();

    RCLCPP_INFO(get_logger(), "Starting chess commander...");
    RCLCPP_INFO(get_logger(), "Waiting for moves from /chess_move topic.");

    RCLCPP_INFO(
      get_logger(),
      "Pose reference frame: %s",
      pose_reference_frame.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Board is calculated relative to chess_board_frame, not H8.");

    RCLCPP_INFO(
      get_logger(),
      "Board size: %.3f m",
      board_size);

    RCLCPP_INFO(
      get_logger(),
      "Square size: %.4f m",
      board_size / 8.0);

    RCLCPP_INFO(
      get_logger(),
      "Pick Z relative to chess_board_frame: %.3f",
      pick_z);

    RCLCPP_INFO(
      get_logger(),
      "Normalized quaternion: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
      qx,
      qy,
      qz,
      qw);

    std::this_thread::sleep_for(
      std::chrono::duration<double>(startup_delay_sec));

    moveit::planning_interface::MoveGroupInterface move_group(
      shared_from_this(),
      planning_group);

    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(num_planning_attempts);
    move_group.setMaxVelocityScalingFactor(velocity_scale);
    move_group.setMaxAccelerationScalingFactor(acceleration_scale);
    move_group.allowReplanning(allow_replanning);

    /*
      Important:
      All square poses are interpreted relative to chess_board_frame.
    */
    move_group.setPoseReferenceFrame(pose_reference_frame);
    move_group.setEndEffectorLink(end_effector_link);

    move_group.setGoalPositionTolerance(position_tolerance);
    move_group.setGoalOrientationTolerance(orientation_tolerance);

    move_group.startStateMonitor();

    if (!move_group.getCurrentState(state_wait_sec))
    {
      RCLCPP_ERROR(get_logger(), "Could not get a valid current robot state.");
      rclcpp::shutdown();
      return;
    }

    /*
      Go to named home target first.
    */
    if (!moveToNamedTarget(move_group, home_named_target))
    {
      RCLCPP_ERROR(get_logger(), "Could not move to home before listening for chess moves.");
      rclcpp::shutdown();
      return;
    }

    // CHANGE: optional startup calibration using the SAME motion logic as the old a1a1 move.
    // It goes above A1 first, then moves DOWN to the actual A1 pick pose, waits for
    // /calibration_done, moves back up, returns home, then continues normal operation.
    if (startup_calibration_enabled)
    {
      publishRobotStatus("calibrating");

      const auto a1_pose = squareToPose(
        "a1",
        board_size,
        pick_z,
        qx,
        qy,
        qz,
        qw);

      auto a1_above = a1_pose;
      a1_above.position.z += lift_z;

      RCLCPP_INFO(
        get_logger(),
        "Startup calibration enabled. Moving ABOVE A1 first: x=%.3f, y=%.3f, z=%.3f",
        a1_above.position.x,
        a1_above.position.y,
        a1_above.position.z);

      RCLCPP_INFO(
        get_logger(),
        "A1 actual calibration/pick pose: x=%.3f, y=%.3f, z=%.3f",
        a1_pose.position.x,
        a1_pose.position.y,
        a1_pose.position.z);

      /*
        1) Move safely above A1 first, exactly like normal chess movement.
      */
      if (!moveToPoseTarget(move_group, a1_above, end_effector_link))
      {
        RCLCPP_ERROR(get_logger(), "Failed to move above A1 calibration pose.");
        rclcpp::shutdown();
        return;
      }

      /*
        2) Move down to the actual A1 pose using the same Cartesian vertical path
           used during normal pick/place movement.
      */
      std::vector<geometry_msgs::msg::Pose> calibration_path;

      appendSegment(
        calibration_path,
        buildVerticalBezier(
          a1_above,
          a1_pose,
          bezier_points,
          vertical_handle_ratio));

      if (!executeCartesianPath(
            move_group,
            calibration_path,
            eef_step,
            min_fraction,
            velocity_scale,
            acceleration_scale))
      {
        RCLCPP_ERROR(get_logger(), "Failed to move down to actual A1 calibration pose.");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "Robot is now at the ACTUAL A1 calibration pose.");
      RCLCPP_INFO(get_logger(), "Adjust/check board alignment now.");
      RCLCPP_INFO(get_logger(), "When done, run:");
      RCLCPP_INFO(
        get_logger(),
        "ros2 topic pub --once /calibration_done std_msgs/msg/String \"{data: 'done'}\"");

      if (!waitForCalibrationDone())
      {
        RCLCPP_ERROR(get_logger(), "Calibration confirmation wait was interrupted.");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "Calibration confirmed. Moving back above A1.");

      /*
        3) Move back up from actual A1 to above A1 before returning home.
      */
      calibration_path.clear();

      appendSegment(
        calibration_path,
        buildVerticalBezier(
          a1_pose,
          a1_above,
          bezier_points,
          vertical_handle_ratio));

      if (!executeCartesianPath(
            move_group,
            calibration_path,
            eef_step,
            min_fraction,
            velocity_scale,
            acceleration_scale))
      {
        RCLCPP_ERROR(get_logger(), "Failed to move up from A1 calibration pose.");
        rclcpp::shutdown();
        return;
      }

      RCLCPP_INFO(get_logger(), "Returning home before normal operation.");

      if (!moveToNamedTarget(move_group, home_named_target))
      {
        RCLCPP_ERROR(get_logger(), "Failed to return home after calibration.");
        rclcpp::shutdown();
        return;
      }

      publishRobotStatus("idle");
      RCLCPP_INFO(get_logger(), "Startup calibration finished. Continuing normal chess operation.");
    }

    /*
      Gripper setup:
      Wait for gripper server and open gripper before starting.
    */
    if (!waitForGripperServer())
    {
      rclcpp::shutdown();
      return;
    }

    if (!sendGripperCommand(get_parameter("gripper_start_open_position").as_double()))
    {
      RCLCPP_ERROR(get_logger(), "Failed to open gripper before starting.");
      rclcpp::shutdown();
      return;
    }

    /*
      Robot is ready and not moving.
      Mapper can use this to sync the board state.
    */
    publishRobotStatus("idle");

    /*
      Main topic-driven loop.
      Every time /chess_move publishes a move like e2e4,
      robot executes the pick-place sequence.
    */
    while (rclcpp::ok())
    {
      ChessMove move;

      if (!waitForChessMove(move, wait_for_move_sec))
      {
        RCLCPP_WARN(
          get_logger(),
          "No chess move received within %.1f seconds. Still listening...",
          wait_for_move_sec);
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "Executing move from topic: %s -> %s",
        move.from.c_str(),
        move.to.c_str());

      // CHANGE: ignore no-op/dummy moves such as a1a1.
      // Calibration is now handled by a separate node, so the joint commander
      // should only execute real chess engine moves.
      if (move.from == move.to)
      {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring no-op move %s -> %s. This is not a real robot move.",
          move.from.c_str(),
          move.to.c_str());
        continue;
      }

      publishRobotStatus("moving");

      // CHANGE: choose the correct FEN before the robot move.
      // If the engine publishes two FENs quickly, we use the newest FEN where
      // move.from is still occupied. That is the state before the robot move.
      bool destination_occupied = false;
      const auto fen_before_robot_move = chooseFenBeforeRobotMove(move);

      if (!fen_before_robot_move.has_value())
      {
        RCLCPP_WARN(
          get_logger(),
          "Could not find a FEN where source %s is occupied. "
          "Skipping capture-removal and executing normal move only.",
          move.from.c_str());
      }
      else
      {
        try
        {
          destination_occupied = isSquareOccupiedInFen(move.to, fen_before_robot_move.value());

          RCLCPP_INFO(
            get_logger(),
            "Destination square %s from chosen pre-robot FEN: %s",
            move.to.c_str(),
            destination_occupied ? "occupied" : "empty");
        }
        catch (const std::exception& e)
        {
          publishRobotStatus("idle");
          RCLCPP_ERROR(
            get_logger(),
            "Failed to check destination occupancy from FEN. Error: %s",
            e.what());
          continue;
        }
      }

      if (destination_occupied)
      {
        RCLCPP_INFO(
          get_logger(),
          "Destination %s is occupied. Removing piece first.",
          move.to.c_str());

        if (!removeOccupiedPieceFromSquare(
              move_group,
              pose_reference_frame,
              move.to,
              end_effector_link,
              board_size,
              pick_z,
              qx,
              qy,
              qz,
              qw,
              lift_z,
              bezier_points,
              eef_step,
              min_fraction,
              vertical_handle_ratio,
              velocity_scale,
              acceleration_scale))
        {
          publishRobotStatus("idle");
          RCLCPP_ERROR(get_logger(), "Failed to remove occupied destination piece.");
          continue;
        }

        if (!moveToNamedTarget(move_group, home_named_target))
        {
          publishRobotStatus("idle");
          RCLCPP_ERROR(get_logger(), "Failed to return home after removing occupied piece.");
          continue;
        }
      }
      else
      {
        RCLCPP_INFO(
          get_logger(),
          "Destination %s is empty. Continuing with normal move.",
          move.to.c_str());
      }

      if (!executeChessMove(
            move_group,
            move,
            end_effector_link,
            board_size,
            pick_z,
            qx,
            qy,
            qz,
            qw,
            lift_z,
            bezier_points,
            eef_step,
            min_fraction,
            vertical_handle_ratio,
            peak_extra_z,
            velocity_scale,
            acceleration_scale))
      {
        publishRobotStatus("idle");
        RCLCPP_ERROR(get_logger(), "Chess move failed. Waiting for next move...");
        continue;
      }

      if (return_home_after_move)
      {
        moveToNamedTarget(move_group, home_named_target);
      }

      publishRobotStatus("idle");

      RCLCPP_INFO(get_logger(), "Move finished successfully. Waiting for next chess move...");
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EEPoseCommander>(
    rclcpp::NodeOptions());

  /*
    MultiThreadedExecutor is useful here because:
    - one thread runs node->run()
    - executor still receives /chess_move callbacks
  */
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread worker([node]() {
    node->run();
  });

  executor.spin();

  if (worker.joinable())
    worker.join();

  rclcpp::shutdown();
  return 0;
}
