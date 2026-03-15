#!/bin/bash

# ─────────────────────────────────────────────────────────────────────────────
# CR5 ROS2 Jazzy -- GitHub Issue Creator (New Projects API)
# Run this from inside your cr5_ws directory
# Usage: bash create_issues.sh
# ─────────────────────────────────────────────────────────────────────────────

REPO="bonnybabukachappilly/cr5-ros2-jazzy"
OWNER="bonnybabukachappilly"
PROJECT_NUMBER="11"

# Helper function -- creates issue then adds to project
create_issue() {
  local title="$1"
  local body="$2"
  local label="$3"
  local milestone="$4"

  echo "Creating: $title"

  # Create issue and capture its URL
  ISSUE_URL=$(gh issue create \
    --repo "$REPO" \
    --title "$title" \
    --body "$body" \
    --label "$label" \
    --milestone "$milestone" 2>&1 | tail -1)

  # Add issue to project using URL
  gh project item-add "$PROJECT_NUMBER" \
    --owner "$OWNER" \
    --url "$ISSUE_URL" 2>/dev/null

  echo "  Done: $ISSUE_URL"
  sleep 1
}

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Creating Phase 3 Issues ==="
# ─────────────────────────────────────────────────────────────────────────────

create_issue \
  "Load CR5 URDF into MoveIt2 Setup Assistant" \
  "## Acceptance Criteria\n- [ ] cr5_moveit_config package generated cleanly\n- [ ] Package builds with colcon build" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Verify SRDF + kinematics config" \
  "## Acceptance Criteria\n- [ ] IK solves for test poses in RViz2\n- [ ] No self-collision errors on home position" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Add FollowJointTrajectory action server to cr5_driver" \
  "## Acceptance Criteria\n- [ ] Action server visible via ros2 action list\n- [ ] Accepts JointTrajectory goals" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Connect MoveIt2 trajectories to ServoJ commands" \
  "## Acceptance Criteria\n- [ ] RViz2 Plan + Execute moves the real arm\n- [ ] ServoJ commands sent to port 30003 at 33Hz" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Implement Cartesian path planning" \
  "## Acceptance Criteria\n- [ ] compute_cartesian_path() executes a straight-line move\n- [ ] No joint limit violations during execution" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Add collision objects to planning scene" \
  "## Acceptance Criteria\n- [ ] Virtual table added to planning scene\n- [ ] MoveIt2 plans around it without collision" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "Write MoveIt2 Python API waypoint script" \
  "## Acceptance Criteria\n- [ ] Script executes 3-point waypoint sequence\n- [ ] Each waypoint reached within 5mm tolerance" \
  "phase-3" \
  "M3: MoveIt2 Complete"

create_issue \
  "C++ Bridge: MoveIt2 single pose goal node" \
  "## Acceptance Criteria\n- [ ] C++ node plans and executes one pose goal\n- [ ] Builds cleanly with ament_cmake" \
  "phase-3" \
  "M3: MoveIt2 Complete"

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Creating Phase 4 Issues ==="
# ─────────────────────────────────────────────────────────────────────────────

create_issue \
  "Create cr5_world.sdf scene file" \
  "## Acceptance Criteria\n- [ ] Gazebo launches with table and CR5 visible\n- [ ] No physics errors on startup" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Configure ros2_control for simulated arm" \
  "## Acceptance Criteria\n- [ ] JointTrajectoryController active in Gazebo\n- [ ] /cr5_sim/joint_states topic publishing" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Add GripperActionController to sim" \
  "## Acceptance Criteria\n- [ ] Gripper open/close works in simulation\n- [ ] Action server visible via ros2 action list" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Write cr5_twin_bridge node" \
  "## Acceptance Criteria\n- [ ] Sim joint states update when real arm moves\n- [ ] Bridge runs without dropping messages" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Verify mirroring latency under 50ms" \
  "## Acceptance Criteria\n- [ ] rqt_plot shows real vs sim within 50ms\n- [ ] Latency measured and documented in NOTES.md" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "MoveIt2 on sim -- full plan + execute" \
  "## Acceptance Criteria\n- [ ] Sim executes plans with no real arm connected\n- [ ] All 6 joints move correctly in Gazebo" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "MoveIt2 on real via same launch file" \
  "## Acceptance Criteria\n- [ ] hardware_interface arg swaps sim/real\n- [ ] No code changes needed to switch between sim and real" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Collision avoidance test in Gazebo" \
  "## Acceptance Criteria\n- [ ] Collision box added to Gazebo scene\n- [ ] MoveIt2 plans around it without collision" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "rosbag record and replay test" \
  "## Acceptance Criteria\n- [ ] /cr5/joint_states recorded cleanly\n- [ ] Replayed bag drives Gazebo sim correctly" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "Record 2-minute demo video" \
  "## Acceptance Criteria\n- [ ] Demo video shows real arm + Gazebo mirror simultaneously\n- [ ] Video linked in README" \
  "phase-4" \
  "M4: Digital Twin Complete"

create_issue \
  "C++ Bridge: Rewrite twin_bridge in C++" \
  "## Acceptance Criteria\n- [ ] C++ twin bridge achieves <30ms latency\n- [ ] Performance benchmark documented in NOTES.md" \
  "phase-4" \
  "M4: Digital Twin Complete"

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Creating Phase 5 Issues ==="
# ─────────────────────────────────────────────────────────────────────────────

create_issue \
  "Install RealSense ROS2 driver" \
  "## Acceptance Criteria\n- [ ] Depth and colour topics visible\n- [ ] ros2 topic list shows /camera/depth/image_rect_raw" \
  "phase-5" \
  "M5: RealSense Complete"

create_issue \
  "camera_tf_broadcaster node" \
  "## Acceptance Criteria\n- [ ] Camera frame in TF tree\n- [ ] Visible in RViz2 alongside robot model" \
  "phase-5" \
  "M5: RealSense Complete"

create_issue \
  "Add camera model to Gazebo scene" \
  "## Acceptance Criteria\n- [ ] Camera model visible in Digital Twin\n- [ ] Camera publishes simulated depth topic in Gazebo" \
  "phase-5" \
  "M5: RealSense Complete"

create_issue \
  "Basic object detection node" \
  "## Acceptance Criteria\n- [ ] /cr5/detected_object topic publishes XYZ position\n- [ ] Detects a simple object (e.g. red cube) in the workspace" \
  "phase-5" \
  "M5: RealSense Complete"

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "=== Creating CI Issues ==="
# ─────────────────────────────────────────────────────────────────────────────

create_issue \
  "Add CI workflow file (lint + build)" \
  "## Acceptance Criteria\n- [ ] .github/workflows/ros2_ci.yml exists\n- [ ] Green CI badge visible on README" \
  "ci" \
  "M0: Environment Ready"

create_issue \
  "Add unit test job to CI" \
  "## Acceptance Criteria\n- [ ] Test job runs on every PR to dev and production\n- [ ] All parser tests pass in CI container" \
  "ci" \
  "M1: TCP Driver Complete"

create_issue \
  "Add integration test job to CI" \
  "## Acceptance Criteria\n- [ ] launch_testing runs in CI\n- [ ] Node startup and topic existence verified" \
  "ci" \
  "M2: ROS2 Concepts Complete"

create_issue \
  "Add dependency check job to CI" \
  "## Acceptance Criteria\n- [ ] rosdep check passes on clean container\n- [ ] All package.xml dependencies verified" \
  "ci" \
  "M0: Environment Ready"

create_issue \
  "Add C++ lint to CI" \
  "## Acceptance Criteria\n- [ ] ament_cpplint runs on all C++ files\n- [ ] No lint errors on C++ bridge code" \
  "ci" \
  "M1: TCP Driver Complete"

# ─────────────────────────────────────────────────────────────────────────────
echo ""
echo "All issues created and added to project!"
echo "Visit:  https://github.com/$REPO/issues"
echo "Board:  https://github.com/users/$OWNER/projects/$PROJECT_NUMBER"
