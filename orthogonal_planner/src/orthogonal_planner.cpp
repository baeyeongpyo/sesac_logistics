#include "orthogonal_planner/orthogonal_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace orthogonal_planner
{

void OrthogonalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  // RCLCPP_INFO(node_->get_logger(), "OrthogonalPlanner configured");
}

void OrthogonalPlanner::cleanup()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalPlanner cleanup");
}

void OrthogonalPlanner::activate()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalPlanner activated");
}

void OrthogonalPlanner::deactivate()
{
  // RCLCPP_INFO(node_->get_logger(), "OrthogonalPlanner deactivated");
}


// A*에서 우선순위 큐에 들어갈 노드
struct QueueNode
{
  unsigned int x;
  unsigned int y;
  int dir;

  double g;
  double f;
};


// f가 작은 노드를 먼저 꺼내기 위한 비교 함수
struct CompareNode
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    if (a.f != b.f) {
      return a.f > b.f;
    }

    // f가 같으면 실제 누적 비용 g가 큰 것 우선
    // = goal 쪽으로 더 진행한 노드를 우선
    if (a.g != b.g) {
      return a.g < b.g;
    }

    // 그것도 같으면 방향 번호로 고정
    return a.dir > b.dir;
  }
};


nav_msgs::msg::Path OrthogonalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;

  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  path.header.stamp = node_->now();


  // ============================================================
  // 1. 실제 좌표(m) -> Costmap Grid 좌표
  // ============================================================

  unsigned int start_x, start_y;
  unsigned int goal_x, goal_y;

  if (!costmap_->worldToMap(
      start.pose.position.x,
      start.pose.position.y,
      start_x,
      start_y))
  {
    // RCLCPP_ERROR(node_->get_logger(), "Start is outside costmap");
    return path;
  }

  if (!costmap_->worldToMap(
      goal.pose.position.x,
      goal.pose.position.y,
      goal_x,
      goal_y))
  {
    // RCLCPP_ERROR(node_->get_logger(), "Goal is outside costmap");
    return path;
  }

  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "Start grid: (%u, %u), Goal grid: (%u, %u)",
  //   start_x, start_y,
  //   goal_x, goal_y);


  // ============================================================
  // 2. Costmap 크기
  // ============================================================

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();

  constexpr int DIR_COUNT = 4;

  /*
       dir = 0 : 오른쪽
       dir = 1 : 아래
       dir = 2 : 왼쪽
       dir = 3 : 위

       대각선 없음.
  */

  const std::array<int, DIR_COUNT> dx = {1, 0, -1, 0};
  const std::array<int, DIR_COUNT> dy = {0, 1, 0, -1};


  // ============================================================
  // 3. A* 비용 설정
  // ============================================================

  // 한 칸 이동 비용
  constexpr double MOVE_COST = 1.0;

  // 방향이 바뀌었을 때 추가 비용
  constexpr double TURN_PENALTY = 5.0;

  // Inflation cost를 얼마나 경로 비용에 반영할지
  constexpr double COSTMAP_WEIGHT = 18.0;


  // ============================================================
  // 4. 상태 번호
  //
  // 상태 = (x, y, 방향)
  //
  // 같은 칸이라도
  //
  // (10, 20, 오른쪽)
  // (10, 20, 위쪽)
  //
  // 은 서로 다른 상태다.
  // ============================================================

  const size_t state_count =
    static_cast<size_t>(size_x) *
    static_cast<size_t>(size_y) *
    DIR_COUNT;

  auto stateIndex =
    [size_x](unsigned int x, unsigned int y, int dir) -> size_t
    {
      return
        (static_cast<size_t>(y) * size_x + x) *
        DIR_COUNT + dir;
    };


  const double INF = std::numeric_limits<double>::infinity();

  std::vector<double> g_cost(state_count, INF);

  // 각 상태가 어디에서 왔는지 저장
  std::vector<long long> parent(state_count, -1);


  // ============================================================
  // 5. Manhattan heuristic
  // ============================================================

  auto heuristic =
    [goal_x, goal_y](unsigned int x, unsigned int y) -> double
    {
      return
        std::abs(static_cast<int>(goal_x) - static_cast<int>(x)) +
        std::abs(static_cast<int>(goal_y) - static_cast<int>(y));
    };


  // ============================================================
  // 6. OPEN LIST
  // ============================================================

  std::priority_queue<
    QueueNode,
    std::vector<QueueNode>,
    CompareNode>
  open;


  /*
   * 시작점에는 아직 "이전 진행 방향"이 없다.
   *
   * 그래서 4개의 방향 상태를 모두 g=0으로 시작시킨다.
   *
   * 이렇게 하면 첫 번째 이동에는
   * TURN_PENALTY가 붙지 않는다.
   */

  for (int dir = 0; dir < DIR_COUNT; ++dir)
  {
    size_t index = stateIndex(start_x, start_y, dir);

    g_cost[index] = 0.0;

    open.push({
      start_x,
      start_y,
      dir,
      0.0,
      heuristic(start_x, start_y)
    });
  }


  // 최종 goal 상태
  long long goal_state = -1;


  // ============================================================
  // 7. A* 탐색
  // ============================================================

  while (!open.empty())
  {
    QueueNode current = open.top();
    open.pop();

    size_t current_index =
      stateIndex(current.x, current.y, current.dir);


    // 이미 더 좋은 경로가 발견된 옛날 데이터면 무시
    if (current.g > g_cost[current_index])
    {
      continue;
    }


    // Goal 도착
    if (current.x == goal_x && current.y == goal_y)
    {
      goal_state = static_cast<long long>(current_index);
      break;
    }


    // ----------------------------------------------------------
    // 상하좌우 4방향 탐색
    // ----------------------------------------------------------

    for (int next_dir = 0; next_dir < DIR_COUNT; ++next_dir)
    {
      int nx =
        static_cast<int>(current.x) + dx[next_dir];

      int ny =
        static_cast<int>(current.y) + dy[next_dir];


      // Costmap 밖이면 제외
      if (
        nx < 0 ||
        ny < 0 ||
        nx >= static_cast<int>(size_x) ||
        ny >= static_cast<int>(size_y))
      {
        continue;
      }


      unsigned int next_x =
        static_cast<unsigned int>(nx);

      unsigned int next_y =
        static_cast<unsigned int>(ny);


      // --------------------------------------------------------
      // 장애물 검사
      // --------------------------------------------------------

      unsigned char cell_cost =
        costmap_->getCost(next_x, next_y);


      // 장애물 또는 unknown 영역은 통과하지 않는다.
      if (
        cell_cost >=
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }


      // --------------------------------------------------------
      // 이동 비용
      // --------------------------------------------------------

      double new_g =
        current.g + MOVE_COST;


      // 방향이 바뀌었다면 회전 패널티
      if (current.dir != next_dir)
      {
        new_g += TURN_PENALTY;
      }


      // --------------------------------------------------------
      // Costmap inflation 비용
      //
      // FREE_SPACE = 거의 0
      // 장애물 근처 = 값 증가
      //
      // 따라서 같은 길이라면 장애물에서 먼 길을 선호
      // --------------------------------------------------------

      double normalized_cost =
        static_cast<double>(cell_cost) / 252.0;

      new_g +=
        COSTMAP_WEIGHT * normalized_cost;


      size_t next_index =
        stateIndex(next_x, next_y, next_dir);


      // 기존 경로보다 싸다면 갱신
      if (new_g < g_cost[next_index])
      {
        g_cost[next_index] = new_g;

        parent[next_index] =
          static_cast<long long>(current_index);


        double h =
          heuristic(next_x, next_y);

        double f =
          new_g + h;


        open.push({
          next_x,
          next_y,
          next_dir,
          new_g,
          f
        });
      }
    }
  }


  // ============================================================
  // 8. Goal까지 경로를 못 찾음
  // ============================================================

  if (goal_state == -1)
  {
    // RCLCPP_WARN(
    //   node_->get_logger(),
    //   "A* failed to find path");

    return path;
  }


  // ============================================================
  // 9. Parent를 따라가며 경로 복원
  // ============================================================

  std::vector<size_t> states;

  long long current_state = goal_state;

  while (current_state != -1)
  {
    states.push_back(
      static_cast<size_t>(current_state));

    current_state =
      parent[static_cast<size_t>(current_state)];
  }


  // 지금은 Goal -> Start 순서이므로 뒤집는다.
  std::reverse(states.begin(), states.end());


  // ============================================================
  // 10. Grid 좌표 -> 실제 map 좌표(m)
  // ============================================================

  for (size_t state : states)
  {
    size_t cell_index =
      state / DIR_COUNT;

    unsigned int y =
      static_cast<unsigned int>(
        cell_index / size_x);

    unsigned int x =
      static_cast<unsigned int>(
        cell_index % size_x);


    double wx;
    double wy;

    costmap_->mapToWorld(
      x,
      y,
      wx,
      wy);


    geometry_msgs::msg::PoseStamped pose;

    pose.header = path.header;

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;

    // 일단 orientation은 기본값
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }


  // 마지막 pose는 사용자가 지정한 정확한 Goal 위치/방향 사용
  if (!path.poses.empty())
  {
    path.poses.back().pose.position.x =
      goal.pose.position.x;

    path.poses.back().pose.position.y =
      goal.pose.position.y;

    path.poses.back().pose.position.z =
      goal.pose.position.z;

    path.poses.back().pose.orientation =
      goal.pose.orientation;
  }


  // RCLCPP_INFO(
  //   node_->get_logger(),
  //   "A* success: %zu poses, total cost %.2f",
  //   path.poses.size(),
  //   g_cost[static_cast<size_t>(goal_state)]);


  return path;
}

}  // namespace orthogonal_planner


PLUGINLIB_EXPORT_CLASS(
  orthogonal_planner::OrthogonalPlanner,
  nav2_core::GlobalPlanner)