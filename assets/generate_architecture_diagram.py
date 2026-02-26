#!/usr/bin/env python3
"""Generate a publication-quality ROS 2 architecture diagram for Smart Kitchen."""

import subprocess, os

DOT = r"""
digraph SmartKitchenROS2 {

    /* ── Global ────────────────────────────────────────────────── */
    graph [
        fontname  = "Helvetica Neue, Helvetica, Arial, sans-serif"
        fontsize  = 18
        label     = <<TABLE BORDER="0" CELLBORDER="0" CELLSPACING="2">
                      <TR><TD><B>Smart Kitchen — ROS 2 Node &amp; Topic Architecture</B></TD></TR>
                      <TR><TD><FONT POINT-SIZE="12" COLOR="#666666">Packages: </FONT><FONT POINT-SIZE="12" COLOR="#1565C0"><B>aruco_tracker</B></FONT><FONT POINT-SIZE="12" COLOR="#666666"> · </FONT><FONT POINT-SIZE="12" COLOR="#2E7D32"><B>smart_kitchen_logic</B></FONT><FONT POINT-SIZE="12" COLOR="#666666">  |  Real-robot configuration</FONT></TD></TR>
                      <TR><TD><FONT POINT-SIZE="10" COLOR="#2E7D32">──── solid = topic (pub/sub)</FONT><FONT POINT-SIZE="10" COLOR="#999999">    </FONT><FONT POINT-SIZE="10" COLOR="#E65100">╌╌╌ dashed orange = action (client→server)</FONT><FONT POINT-SIZE="10" COLOR="#999999">    </FONT><FONT POINT-SIZE="10" COLOR="#90A4AE">╌╌╌ dashed gray = feedback / result</FONT></TD></TR>
                     </TABLE>>
        labelloc  = t
        pad       = "0.6,0.4"
        nodesep   = 0.7
        ranksep   = 1.0
        bgcolor   = "white"
        compound  = true
        splines   = true
        rankdir   = TB
        dpi       = 180
    ];

    node [
        fontname = "Helvetica Neue, Helvetica, Arial, sans-serif"
        fontsize = 10
        shape    = box
        style    = "filled,rounded"
        penwidth = 1.3
        margin   = "0.14,0.07"
    ];

    edge [
        fontname  = "Helvetica Neue, Helvetica, Arial, sans-serif"
        fontsize  = 8
        color     = "#555555"
        penwidth  = 1.1
        arrowsize = 0.7
    ];

    /* ═══════════════════════════════════════════════════════════
       LAYER 1 — SENSORS / EXTERNAL PROVIDERS
       ═══════════════════════════════════════════════════════════ */
    subgraph cluster_sensors {
        label     = <<B><FONT POINT-SIZE="11" COLOR="#616161">Sensors &amp; State Providers</FONT></B>>
        style     = "dashed,rounded"
        color     = "#BDBDBD"
        fillcolor = "#FAFAFA"
        fontsize  = 11
        margin    = 16

        rgb_cam    [label="RGB Overhead\nCamera"         shape=box3d fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
        rsp        [label="Robot State Publisher\n+ TF2"  fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
        realsense  [label="RealSense\nCamera"            shape=box3d fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
    }

    /* ═══════════════════════════════════════════════════════════
       LAYER 2 — PERCEPTION
       ═══════════════════════════════════════════════════════════ */
    subgraph cluster_percep {
        label     = <<B><FONT POINT-SIZE="11" COLOR="#1565C0">Perception</FONT></B>>
        style     = "filled,rounded"
        color     = "#1565C0"
        fillcolor = "#E3F2FD"
        margin    = 14

        human_det [label="human_detection_node\n(YOLO person detector)" fillcolor="#90CAF9" fontcolor="#0D47A1"]
        aruco     [label="aruco_all_distance\n(ArUco marker tracker)"   fillcolor="#90CAF9" fontcolor="#0D47A1"]
    }

    /* ═══════════════════════════════════════════════════════════
       LAYER 3 — PLANNING & ORCHESTRATION
       ═══════════════════════════════════════════════════════════ */
    subgraph cluster_orch {
        label     = <<B><FONT POINT-SIZE="11" COLOR="#2E7D32">Planning &amp; Orchestration</FONT></B>>
        style     = "filled,rounded"
        color     = "#2E7D32"
        fillcolor = "#E8F5E9"
        margin    = 14

        cmd_seq   [label="command_sequence_client\n(decision-point sequencer)"  fillcolor="#A5D6A7" fontcolor="#1B5E20"]
        planning  [label="planning_node\n(IK · aruco tracking · motion orchestration)" fillcolor="#81C784" fontcolor="#1B5E20"]
    }

    /* ═══════════════════════════════════════════════════════════
       LAYER 4 — EXECUTION
       ═══════════════════════════════════════════════════════════ */
    subgraph cluster_exec {
        label     = <<B><FONT POINT-SIZE="11" COLOR="#4E342E">Execution Layer</FONT></B>>
        style     = "filled,rounded"
        color     = "#795548"
        fillcolor = "#EFEBE9"
        margin    = 14

        cmd_exec   [label="command_executor_node\n(waypoint sequence executor)"  fillcolor="#BCAAA4" fontcolor="#3E2723"]
        moving     [label="moving_node\n(joint / carriage / lift forwarder)"     fillcolor="#D7CCC8" fontcolor="#3E2723"]
        robot_ctrl [label="robot_controller_node\n(MoveToJoints action server)"  fillcolor="#D7CCC8" fontcolor="#3E2723"]
    }

    /* ═══════════════════════════════════════════════════════════
       LAYER 5 — HARDWARE CONTROLLERS
       ═══════════════════════════════════════════════════════════ */
    subgraph cluster_hw {
        label     = <<B><FONT POINT-SIZE="11" COLOR="#616161">Hardware Controllers &amp; Actuators</FONT></B>>
        style     = "dashed,rounded"
        color     = "#BDBDBD"
        fillcolor = "#FAFAFA"
        margin    = 16

        gripper    [label="robotiq_gripper\n_controller"     fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
        elmo       [label="ELMO Drives\n(carriage + lift)"   shape=box3d fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
        jtc        [label="joint_trajectory\n_controller"     fillcolor="#CFD8DC" fontcolor="#37474F" penwidth=0.8]
    }

    /* ═══════════════════════════════════════════════════════════
       VERTICAL LAYER ORDERING
       ═══════════════════════════════════════════════════════════ */
    { rank = same; rgb_cam; rsp; realsense }
    { rank = same; human_det; aruco }
    { rank = same; cmd_seq; planning }
    { rank = same; cmd_exec; moving; robot_ctrl }
    { rank = same; gripper; elmo; jtc }

    /* ═══════════════════════════════════════════════════════════
       SENSOR → PERCEPTION  (blue arrows)
       ═══════════════════════════════════════════════════════════ */
    realsense -> aruco [
        label=<<FONT COLOR="#1565C0">/camera/color/image_raw<BR/>/camera/color/camera_info</FONT>>
        color="#1976D2" penwidth=1.3
    ];

    rgb_cam -> human_det [
        label=<<FONT COLOR="#1565C0">/rgb/image_raw<BR/>/rgb/camera_info</FONT>>
        color="#1976D2" penwidth=1.3
    ];

    rsp -> planning [
        label=<<FONT COLOR="#546E7A">/robot_description<BR/>/joint_states · TF2</FONT>>
        color="#78909C" penwidth=1.0
    ];

    /* ═══════════════════════════════════════════════════════════
       PERCEPTION → ORCHESTRATION  (green arrows, main data flow)
       ═══════════════════════════════════════════════════════════ */
    aruco -> planning [
        label=<<B><FONT COLOR="#1565C0">/aruco_distances</FONT></B><BR/><FONT COLOR="#666666" POINT-SIZE="7">Float32MultiArray  [id, dist, x, y, z, …]</FONT>>
        color="#1565C0" penwidth=1.6
    ];

    human_det -> cmd_seq [
        label=<<B><FONT COLOR="#2E7D32">/human_detection/left</FONT></B><BR/><B><FONT COLOR="#2E7D32">/human_detection/right</FONT></B><BR/><FONT COLOR="#666666" POINT-SIZE="7">Bool</FONT>>
        color="#388E3C" penwidth=1.6
    ];

    /* ═══════════════════════════════════════════════════════════
       ACTION CONNECTIONS  (dashed orange)
       ═══════════════════════════════════════════════════════════ */

    planning -> cmd_exec [
        label=<<B><FONT COLOR="#E65100">/execute_command</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">ExecuteCommand action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    cmd_seq -> cmd_exec [
        label=<<B><FONT COLOR="#E65100">/execute_command</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">ExecuteCommand action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    cmd_exec -> robot_ctrl [
        label=<<B><FONT COLOR="#E65100">/move_to_joints</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">MoveToJoints action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    robot_ctrl -> jtc [
        label=<<B><FONT COLOR="#E65100">/…/follow_joint_trajectory</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">FollowJointTrajectory action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    cmd_exec -> gripper [
        label=<<B><FONT COLOR="#E65100">/…/gripper_cmd</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">GripperCommand action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    /* ═══════════════════════════════════════════════════════════
       PLANNING ↔ MOVING NODE  (topic-based command/result loop)
       ═══════════════════════════════════════════════════════════ */
    planning -> moving [
        label=<<FONT COLOR="#2E7D32"><B>/move_robot_goal</B>  (Float64MultiArray)<BR/><B>/move_carriage_goal</B>  (Float32)</FONT>>
        color="#2E7D32" penwidth=1.3
    ];

    moving -> planning [
        label=<<FONT COLOR="#78909C">/move_robot_result<BR/>/move_carriage_result  (Bool)</FONT>>
        color="#90A4AE" style=dashed penwidth=0.9
        constraint=false
    ];

    /* ═══════════════════════════════════════════════════════════
       EXECUTION → HARDWARE
       ═══════════════════════════════════════════════════════════ */
    moving -> jtc [
        label=<<B><FONT COLOR="#E65100">/…/follow_joint_trajectory</FONT></B><BR/><FONT COLOR="#999999" POINT-SIZE="7">FollowJointTrajectory action</FONT>>
        style=dashed color="#E65100" penwidth=1.4
    ];

    moving -> elmo [
        label=<<FONT COLOR="#546E7A">/elmo/id1/{carriage,lift}/position/set</FONT>>
        color="#78909C" penwidth=1.0
    ];

    cmd_exec -> elmo [
        label=<<FONT COLOR="#546E7A">/elmo/id1/{carriage,lift}/position/set</FONT>>
        color="#78909C" penwidth=1.0
    ];

    /* ═══════════════════════════════════════════════════════════
       ELMO FEEDBACK  (dashed gray, going up)
       ═══════════════════════════════════════════════════════════ */
    elmo -> planning [
        label=<<FONT COLOR="#90A4AE">/elmo/id1/carriage/position/get</FONT>>
        color="#B0BEC5" style=dashed penwidth=0.8
        constraint=false
    ];

    elmo -> cmd_exec [
        label=<<FONT COLOR="#90A4AE">/elmo/id1/{carriage,lift}/position/get</FONT>>
        color="#B0BEC5" style=dashed penwidth=0.8
        constraint=false
    ];
}
"""

OUT_DIR = "/home/ros2-jazzy/workspace/smart_kitchen_ws"
DOT_FILE = os.path.join(OUT_DIR, "ros2_architecture.dot")
PNG_FILE = os.path.join(OUT_DIR, "ros2_architecture.png")
SVG_FILE = os.path.join(OUT_DIR, "ros2_architecture.svg")

with open(DOT_FILE, "w") as f:
    f.write(DOT)

for fmt, path in [("png", PNG_FILE), ("svg", SVG_FILE)]:
    result = subprocess.run(
        ["dot", f"-T{fmt}", "-o", path, DOT_FILE],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        print(f"ERROR ({fmt}): {result.stderr}")
    else:
        print(f"Generated: {path}")

print("Done.")
