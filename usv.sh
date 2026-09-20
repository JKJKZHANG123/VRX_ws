#!/usr/bin/env bash
# ============================================================
# USV 全链路一键启动
# 用法:  ./usv.sh          # 启动全部 (sim+lio+nav+bridge+view)
#        ./usv.sh stop     # 停止全部
#        ./usv.sh status   # 查看各进程状态
#        ./usv.sh restart  # 重启（重新加载配置）
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$SCRIPT_DIR/.usv_logs"
PID_FILE="$SCRIPT_DIR/.usv_pids"
# This managed environment may not allow writes to ~/.ros/log. Keep launch
# logs inside the workspace unless the caller explicitly chose another path.
export ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
mkdir -p "$ROS_LOG_DIR"

# ── 颜色 ───────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

log()  { echo -e "${CYAN}[USV]${NC} $*"; }
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
warn() { echo -e "${YELLOW}[!!]${NC}  $*"; }

# ── stop ───────────────────────────────────────────────────
do_stop() {
    log "停止全部 USV 进程..."
    local tracked_pids=()
    if [[ -f "$PID_FILE" ]]; then
        while read -r name pid; do
            [[ "$pid" =~ ^[0-9]+$ ]] || continue
            tracked_pids+=("$pid")
            if kill -0 "$pid" 2>/dev/null; then
                # Each service is started in its own session/process group.
                # Killing the group avoids leaving ros2 launch children alive.
                kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
                echo "  停止 $name (pid $pid)"
            fi
        done < "$PID_FILE"
        sleep 2
        for pid in "${tracked_pids[@]}"; do
            kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
        done
        rm -f "$PID_FILE"
    fi

    # Extra cleanup for processes that may have survived an old version of
    # this script.  Keep patterns specific; do not use a broad `ros2` kill.
    local patterns=(
        'laserMapping'
        'ekf_node'
        'navsat_transform'
        'covariance_injector'
        'bt_navigator'
        'controller_server'
        'planner_server'
        'smoother_server'
        'behavior_server'
        'waypoint_follower'
        'velocity_smoother'
        'lifecycle_manager'
        'feature_nav_node'
        'click_to_goal'
        'costmap_3d_markers'
        'pointlio_mapping'
        'control_bridge'
        'clock_bridge_watchdog'
        'clock_recovery_bridge'
        'dynamic_obstacle_tracker'
        'dynamic_obstacle_scenario'
        'nav_data_logger'
        'nav_path_recorder'
        'startup_cloud_gate'
        'm5_multigoal_command'
        'obstacle_filter'
        # Children of the cloud-filter and Gazebo-click launch parents do not
        # necessarily retain the `ros2 launch ...` command line.  Match their
        # executable/bridge names too, otherwise an old run can survive and
        # create duplicate SafetyCloud or goal topics after restart.
        'cloud_filter'
        'gz_click_to_goal_node'
        'gz_click_bridge'
        # ros2 launch parents from an older run are not necessarily in the
        # current PID file.  Match only this workspace's launch entrypoints
        # so stale launch parents cannot create duplicate TF/control writers.
        'ros2 launch point_lio mapping_vrx.launch.py'
        'ros2 launch vrx_gz competition.launch.py'
        'ros2 launch usv_cloud_filter cloud_filter.launch.py'
        'ros2 launch usv_navigation navigation.launch.py'
        'ros2 launch usv_control_bridge control_bridge.launch.py'
        'ros2 launch usv_click_gazebo gz_click_bridge.launch.py'
        'parameter_bridge /model/wamv'
        'parameter_bridge /clock@'
        'parameter_bridge /gazebo/click/point@'
        'gz sim'
        'gzserver'
        'rviz2'
    )
    # pgrep -f also searches the command line of the shell that invoked this
    # script.  If that command line contains a cleanup pattern (for example
    # the post-stop `ps ... cloud_filter` check), killing the immediate parent
    # would terminate the caller and make cleanup appear to fail.  Exclude the
    # complete ancestor chain, not only $$ and $PPID.
    local excluded_pids=" $$ "
    local ancestor=$PPID ancestor_ppid
    while [[ "$ancestor" =~ ^[0-9]+$ ]]; do
        excluded_pids+="$ancestor "
        [[ "$ancestor" == 1 ]] && break
        ancestor_ppid=$(ps -o ppid= -p "$ancestor" 2>/dev/null | tr -d ' ')
        [[ "$ancestor_ppid" =~ ^[0-9]+$ ]] || break
        ancestor="$ancestor_ppid"
    done
    local pattern pid
    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ "$pid" =~ ^[0-9]+$ ]] || continue
            [[ " $excluded_pids " == *" $pid "* ]] && continue
            kill -TERM "$pid" 2>/dev/null || true
        done < <(pgrep -f "$pattern" 2>/dev/null || true)
    done
    sleep 1
    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ "$pid" =~ ^[0-9]+$ ]] || continue
            [[ " $excluded_pids " == *" $pid "* ]] && continue
            kill -KILL "$pid" 2>/dev/null || true
        done < <(pgrep -f "$pattern" 2>/dev/null || true)
    done
    ok "已停止"
}

# ── status ─────────────────────────────────────────────────
# A ros2 launch parent can remain alive after one of its child nodes exits.
# Therefore status must inspect the actual Nav2 lifecycle nodes/action servers,
# not only the PID file. This is intentionally a non-blocking snapshot.
nav2_is_ready() {
    local nodes actions node state
    nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
    for node in /planner_server /controller_server /bt_navigator \
                /behavior_server /waypoint_follower /velocity_smoother; do
        grep -qx "$node" <<<"$nodes" || return 1
        state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
        grep -q 'active \[3\]' <<<"$state" || return 1
    done
    actions="$(timeout 5 ros2 action list 2>/dev/null || true)"
    grep -qx '/navigate_to_pose' <<<"$actions" || return 1
    grep -qx '/navigate_through_poses' <<<"$actions" || return 1
    return 0
}

goal_pose_is_ready() {
    timeout 5 ros2 topic info /goal_pose 2>/dev/null \
        | grep -qE 'Subscription count: [1-9][0-9]*'
}

do_status() {
    echo ""
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━ USV 链路状态 ━━━━━━━━━━━━━━━━━━━━━━${NC}"
    if [[ ! -f "$PID_FILE" ]]; then
        warn "未找到进程记录 (未启动或已停止)"
        if nav2_is_ready; then
            echo -e "  ${GREEN}●${NC} Nav2 实际就绪（PID 文件缺失，存在旧/外部启动链路）"
        else
            echo -e "  ${RED}○${NC} Nav2 未就绪"
        fi
        return
    fi
    while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then
            echo -e "  ${GREEN}●${NC} $name  (pid $pid)"
        else
            echo -e "  ${RED}○${NC} $name  (pid $pid — 已停止)"
        fi
    done < "$PID_FILE"
    if nav2_is_ready; then
        echo -e "  ${GREEN}✓ Nav2 实际状态: READY（6 节点 active [3] + 2 action server）${NC}"
    else
        echo -e "  ${RED}✗ Nav2 实际状态: NOT READY（即使 nav2 launch PID 仍存活也禁止发目标）${NC}"
    fi
    if goal_pose_is_ready; then
        echo -e "  ${GREEN}✓ /goal_pose: 有订阅者${NC}"
    else
        echo -e "  ${RED}✗ /goal_pose: 无订阅者${NC}"
    fi
    echo ""
}

# ── 启动单个服务（后台，记录 PID 和日志） ──────────────────
start_service() {
    local name="$1"; shift
    local logfile="$LOG_DIR/${name}.log"
    mkdir -p "$LOG_DIR"
    log "启动 $name ..."
    setsid bash -c "source /opt/ros/jazzy/setup.bash; source '$SCRIPT_DIR/install/setup.bash'; $*" \
        > "$logfile" 2>&1 &
    local pid=$!
    echo "$name $pid" >> "$PID_FILE"
    echo "  → pid $pid  日志: $logfile"
}

# ── 等待某个服务会话中的实际进程 ─────────────────────────
# start_service records the session leader. ros2 launch is only a parent, so
# checking that PID is insufficient for GUI nodes: the parent can stay alive
# while rviz2 has already crashed. Match by session ID and executable name.
wait_for_session_process() {
    local session_pid="$1"; local process_name="$2"; local max_wait="${3:-30}"
    [[ "$session_pid" =~ ^[0-9]+$ ]] || return 1
    local waited=0 actual_pid
    while (( waited < max_wait )); do
        actual_pid="$(ps -eo pid=,sid=,comm= | awk -v sid="$session_pid" -v name="$process_name" '$2 == sid && $3 == name {print $1; exit}')"
        if [[ "$actual_pid" =~ ^[0-9]+$ ]] && kill -0 "$actual_pid" 2>/dev/null; then
            echo "$actual_pid"
            return 0
        fi
        sleep 1
        waited=$((waited + 1))
    done
    return 1
}

# ── 等待某个 ROS 话题收到首条消息（最多 N 秒） ─────────────
# 不使用 `ros2 topic hz`：它要先累计多条消息，在低频/仿真时钟刚启动时
# 容易误判“没有数据”。`echo --once` 只确认话题真的已经能发出消息。
wait_for_topic() {
    local topic="$1"; local max_wait="${2:-30}"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
    local waited=0
    # PointCloud2 safety streams use SensorDataQoS (best effort).  Make the
    # preflight subscriber explicit instead of relying on the CLI default,
    # otherwise a reliable reader can report a false "no first frame" and
    # abort a valid startup.  Best-effort readers remain compatible with the
    # reliable odometry topic used earlier in this script.
    while ! timeout 4 ros2 topic echo --qos-profile sensor_data --once "$topic" \
            >/dev/null 2>&1; do
        sleep 2; waited=$((waited + 2))
        [[ $waited -ge $max_wait ]] && { warn "$topic 等待超时 (${max_wait}s)"; return 1; }
        echo -n "."
    done
    echo " ✓"
}

# ── 等待 TF 链路可用（最多 N 秒） ──────────────────────────
# Nav2 的 costmap 在 lifecycle 激活时会立即检查 global_frame -> base_link。
# 如果在 Point-LIO 首次发布 camera_init -> aft_mapped 之前启动 Nav2，
# RViz 会报 SafetyCloud/Transform，local_costmap 也会不断打印 Invalid frame。
# 这里在启动 Nav2 前确认完整链路已经存在，避免“先启动再撞错误”。
wait_for_tf() {
    local parent="$1"; local child="$2"; local max_wait="${3:-30}"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
    local waited=0
    while ! timeout 4 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>/dev/null \
            | grep -qE '^At time|^Translation:'; do
        sleep 2; waited=$((waited + 2))
        [[ $waited -ge $max_wait ]] && {
            warn "TF ${parent} -> ${child} 等待超时 (${max_wait}s)"
            return 1
        }
        echo -n "."
    done
    echo " ✓"
}

# ── 等待 Nav2 全部生命周期节点激活 ────────────────────────
# 只 sleep 固定时间会把“Nav2 进程已启动但未激活”误判为就绪，
# 导致控制桥/目标发布器先运行而目标被丢弃。这里同时检查节点、
# lifecycle 状态和两个导航 action server，任何一项未满足都不放行。
wait_for_node() {
    local node_name="$1"; local max_wait="${2:-30}"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
    local waited=0
    while ! timeout 5 ros2 node list 2>/dev/null | grep -qx "$node_name"; do
        sleep 2
        waited=$((waited + 2))
        [[ $waited -ge $max_wait ]] && {
            warn "节点 ${node_name} 等待超时 (${max_wait}s)"
            return 1
        }
        echo -n "."
    done
    echo " ✓"
}

wait_for_topic_subscriber() {
    local topic="$1"; local max_wait="${2:-30}"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
    local waited=0 info
    while true; do
        info="$(timeout 5 ros2 topic info "$topic" 2>/dev/null || true)"
        if grep -qE 'Subscription count: [1-9][0-9]*' <<<"$info"; then
            echo " ✓"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
        [[ $waited -ge $max_wait ]] && {
            warn "话题 ${topic} 没有订阅者 (${max_wait}s)"
            return 1
        }
        echo -n "."
    done
}

wait_for_nav2_ready() {
    local max_wait="${1:-90}"
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null
    local waited=0
    local required=(/planner_server /controller_server /bt_navigator \
                    /behavior_server /waypoint_follower /velocity_smoother)
    while true; do
        local nodes actions all_nodes active_nodes
        nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
        all_nodes=true
        for node in "${required[@]}"; do
            grep -qx "$node" <<<"$nodes" || { all_nodes=false; break; }
        done
        actions="$(timeout 5 ros2 action list 2>/dev/null || true)"
        local actions_ready=false
        if grep -qx '/navigate_to_pose' <<<"$actions" && \
           grep -qx '/navigate_through_poses' <<<"$actions"; then
            actions_ready=true
        fi
        active_nodes=true
        if [[ "$all_nodes" == true ]]; then
            for node in "${required[@]}"; do
                local state
                state="$(timeout 5 ros2 lifecycle get "$node" 2>/dev/null || true)"
                grep -q 'active \[3\]' <<<"$state" || { active_nodes=false; break; }
            done
        else
            active_nodes=false
        fi
        if [[ "$all_nodes" == true && "$active_nodes" == true && \
              "$actions_ready" == true ]]; then
            echo " ✓ (nodes active, action servers ready)"
            return 0
        fi
        if (( waited >= max_wait )); then
            warn "Nav2 就绪检查超时 (${max_wait}s)"
            echo "  nodes: $(tr '\n' ' ' <<<"$nodes")"
            echo "  actions: $(tr '\n' ' ' <<<"$actions")"
            return 1
        fi
        sleep 3
        waited=$((waited + 3))
        echo -n "."
    done
}

# ── start ──────────────────────────────────────────────────
do_start() {
    # Always clean tracked and untracked leftovers before a restart. A missing
    # PID file does not prove that an older launch parent/child has exited.
    log "启动前强制清理旧 ROS/Gazebo/Nav2 进程..."
    do_stop
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true
    ros2 daemon stop >/dev/null 2>&1 || true
    sleep 2
    rm -f "$PID_FILE"
    mkdir -p "$LOG_DIR"

    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║      USV 自主导航全链路启动                 ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    # ① 仿真 (必须显式 export PRIME, 否则 gz 跑核显会卡)
    log "① 启动 VRX 仿真 (Gazebo)..."
    start_service "sim" \
        "export __NV_PRIME_RENDER_OFFLOAD=1; \
         export __GLX_VENDOR_LIBRARY_NAME=nvidia; \
         export INSTALL_DIR='$SCRIPT_DIR/install'; \
         for pkg_share in \"\$INSTALL_DIR\"/*/share; do \
            [ -d \"\$pkg_share\" ] && export GZ_SIM_RESOURCE_PATH=\"\$pkg_share:\${GZ_SIM_RESOURCE_PATH:-}\"; \
         done; \
         export GZ_GUI_PLUGIN_PATH=\"\$INSTALL_DIR/usv_click_gazebo/lib/usv_click_gazebo:\${GZ_GUI_PLUGIN_PATH:-}\"; \
         ros2 launch vrx_gz competition.launch.py world:=sydney_regatta"
    log "等待仿真初始化 (25s)..."; sleep 25

    # Gazebo 的联合 parameter_bridge 长时间运行后可能出现“进程仍在、
    # /clock 发布端消失”。独立看门狗只在 /clock 连续静默时补起恢复桥，
    # 自身始终使用系统时间，避免 Nav2/LIO 因仿真时钟冻结而失效。
    log "启动仿真时钟看门狗..."
    start_service "clock_watchdog" \
        "ros2 run usv_navigation clock_bridge_watchdog --ros-args -p use_sim_time:=false"
    sleep 2

    # ② Point-LIO + its own RViz. Keep the LIO view enabled: it is the
    # authoritative view for /cloud_registered and LIO odometry. The previous
    # workaround disabled it to avoid two windows, which silently removed the
    # LIO visualization. The two RViz nodes now have distinct names and their
    # saved geometries are tiled on the 2560x1600 desktop.
    log "② 启动 Point-LIO（同时启动 LIO RViz 视图）..."
    local gui_display="${DISPLAY:-:0}"
    local gui_xauthority="${XAUTHORITY:-$HOME/.Xauthority}"
    start_service "lio" \
        "export DISPLAY='$gui_display'; \
         export XAUTHORITY='$gui_xauthority'; \
         export QT_QPA_PLATFORM=\${QT_QPA_PLATFORM:-xcb}; \
         export __NV_PRIME_RENDER_OFFLOAD=1; \
         export __GLX_VENDOR_LIBRARY_NAME=nvidia; \
         ros2 launch point_lio mapping_vrx.launch.py rviz:=true"
    local lio_pid lio_rviz_pid
    lio_pid="$(awk '$1 == "lio" {p=$2} END {print p}' "$PID_FILE")"
    lio_rviz_pid="$(wait_for_session_process "$lio_pid" rviz2 12 || true)"
    if [[ -z "$lio_rviz_pid" ]]; then
        warn "LIO RViz 未能启动；请检查 $LOG_DIR/lio.log 和 $ROS_LOG_DIR/"
        tail -100 "$LOG_DIR/lio.log" 2>/dev/null || true
        do_stop
        return 1
    fi
    ok "LIO RViz 已启动 (pid $lio_rviz_pid, DISPLAY=$gui_display)"
    log "等待 LIO 首条里程计 (10s)..."; sleep 10
    log "等待 Point-LIO 首条里程计，确认 camera_init 已建立..."
    if ! wait_for_topic "/aft_mapped_to_init" 45; then
        warn "Point-LIO 未发布 /aft_mapped_to_init；为避免 Nav2 Transform 错误，本次不启动导航栈。"
        warn "请先检查 .usv_logs/lio.log（通常是输入点云/IMU或ROS DDS权限问题）。"
        do_stop
        return 1
    fi

    # ③ GPS 锚定 (DISABLED: VRX GPS causes coordinate jumps → "飞天" bug)
    # log "③ 启动 GPS 锚定 (防 LIO 漂移)..."
    # start_service "gps" \
    #     "ros2 launch usv_localization navsat_only.launch.py"
    # sleep 3

    # ③ 先确认 TF，再启动水面点云过滤。
    # obstacle_filter has its own TF listener.  A fixed sleep is not enough:
    # the first raw LiDAR packet can arrive before that listener has received
    # the camera_init -> aft_mapped -> base_link chain, which produces a
    # misleading SafetyCloud Transform warning.
    log "确认点云过滤所需 TF camera_init -> base_link..."
    if ! wait_for_tf "camera_init" "wamv/wamv/base_link" 20; then
        warn "点云过滤所需 TF 不完整；为避免 SafetyCloud Transform 错误，本次不启动后续链路。"
        do_stop
        return 1
    fi
    log "启动水面点云过滤..."
    start_service "cloud_filter" \
        "ros2 launch usv_cloud_filter cloud_filter.launch.py"
    # Do not start Nav2 until the safety stream has actually produced a
    # message.  This is stronger than sleeping because it also proves that
    # the filter's own TF listener can transform the raw LiDAR frame.
    log "等待 SafetyCloud 首帧 (确认坐标系与 TF 已就绪)..."
    if ! wait_for_topic "/usv/safety_cloud" 20; then
        warn "SafetyCloud 未就绪；为避免 Nav2 Transform/安全链路错误，本次不启动导航栈。"
        warn "请检查 .usv_logs/cloud_filter.log。"
        do_stop
        return 1
    fi

    # Require several consecutive scans with no returns intersecting the
    # expanded vessel envelope. This prevents occasional startup spray/self/TF
    # artifacts from seeding Nav2 with obstacles around the boat. Preserve the
    # per-frame evidence for every startup instead of relying on screenshots.
    local startup_id="$(date +%Y%m%d_%H%M%S)"
    local startup_data="$SCRIPT_DIR/report3/data/startup_cloud_gate_${startup_id}.csv"
    local startup_config="$SCRIPT_DIR/report3/configs/startup_cloud_gate_${startup_id}"
    mkdir -p "$startup_config"
    cp "$SCRIPT_DIR/src/usv_cloud_filter/config/cloud_filter_params.yaml" "$startup_config/"
    log "检查启动点云洁净度（连续 5 帧）..."
    if ! python3 "$SCRIPT_DIR/experiments/check_startup_cloud.py" \
            --out-csv "$startup_data" --timeout 20 --clean-frames 5 \
            > "$LOG_DIR/startup_cloud_gate_${startup_id}.log" 2>&1; then
        warn "启动点云在船体附近存在异常点；拒绝启动 Nav2。"
        warn "数据: $startup_data"
        warn "日志: $LOG_DIR/startup_cloud_gate_${startup_id}.log"
        printf 'result=failed\nwall_time=%s\n' "$(date --iso-8601=seconds)" \
            > "$startup_config/result.txt"
        do_stop
        return 1
    fi
    printf 'result=passed\nwall_time=%s\n' "$(date --iso-8601=seconds)" \
        > "$startup_config/result.txt"
    ok "启动点云洁净度通过；数据已保存到 $startup_data"

    # ④ Nav2 导航栈 (global_frame=camera_init, 直接用 Point-LIO 里程计,
    #    不再需要 EKF; aft_mapped→base_link 静态 TF 在 mapping_vrx.launch.py)
    # 注: local_costmap 覆盖 30m (近距避障)，resolution 0.3m
    #     global_costmap 覆盖 100m 半径 (远程规划)，resolution 0.5m
    #     obstacle_max_range: 25m (匹配点云过滤的 max_range)
    log "④ 启动 Nav2 导航栈..."
    local dynamic_tracker="${USV_DYNAMIC_TRACKER:-false}"
    log "动态障碍物跟踪器: $dynamic_tracker"
    log "Nav2 只使用带衰减的实时点云层（不加载持久结构地图）"
    start_service "nav2" \
        "ros2 launch usv_navigation navigation.launch.py enable_dynamic_tracker:=$dynamic_tracker"
    log "等待 Nav2 全部节点激活并确认 action server..."
    if ! wait_for_nav2_ready 90; then
        warn "Nav2 未完全就绪；不启动控制桥、目标桥或 RViz，避免在半启动状态下执行测试。"
        do_stop
        return 1
    fi

    # ⑤ 控制桥
    log "⑤ 启动控制桥..."
    start_service "bridge" \
        "ros2 launch usv_control_bridge control_bridge.launch.py"
    log "确认控制桥节点已运行..."
    if ! wait_for_node "/control_bridge" 30; then
        warn "控制桥未就绪；不启动目标桥和 RViz。"
        do_stop
        return 1
    fi

    # ⑥ Gazebo 点击 -> Nav2 目标桥 (bridge + 换算节点)
    log "⑥ 启动 Gazebo 点击目标桥..."
    start_service "gz_click" \
        "ros2 launch usv_click_gazebo gz_click_bridge.launch.py"
    log "确认目标桥节点和 /goal_pose 订阅端已运行..."
    if ! wait_for_node "/gz_click_bridge" 30 || \
       ! wait_for_node "/gz_click_to_goal_node" 30 || \
       ! wait_for_topic_subscriber "/goal_pose" 30; then
        warn "目标桥未完全就绪；不启动 RViz，也不允许后续发送目标。"
        do_stop
        return 1
    fi

    # ⑦ 导航视图 RViz (3D 融合视图: 点云+路径+3D障碍方块)
    # Use an explicit GUI environment and exec rviz2 so the PID in
    # .usv_pids is the actual RViz process, not a short-lived shell.  This
    # avoids the previous silent-failure case where view.log stayed empty and
    # the launch script still printed "启动完成".
    log "⑦ 启动 3D 导航视图 (RViz)..."
    RVIZ_CONFIG="$SCRIPT_DIR/install/usv_navigation/share/usv_navigation/config/nav3d_view.rviz"
    if [[ ! -f "$RVIZ_CONFIG" ]]; then
        warn "Nav2 RViz 配置不存在: $RVIZ_CONFIG"
        do_stop
        return 1
    fi
    start_service "view" \
        "export DISPLAY='$gui_display'; \
         export XAUTHORITY='$gui_xauthority'; \
         export QT_QPA_PLATFORM=\${QT_QPA_PLATFORM:-xcb}; \
         export __NV_PRIME_RENDER_OFFLOAD=1; \
         export __GLX_VENDOR_LIBRARY_NAME=nvidia; \
         exec rviz2 -d '$RVIZ_CONFIG' --ros-args -r __node:=nav_rviz -p use_sim_time:=true"
    local view_session_pid nav_rviz_pid
    view_session_pid="$(awk '$1 == "view" {p=$2} END {print p}' "$PID_FILE")"
    nav_rviz_pid="$(wait_for_session_process "$view_session_pid" rviz2 12 || true)"
    if [[ -z "$nav_rviz_pid" ]]; then
        warn "Nav2 RViz 未能保持运行；请检查 $LOG_DIR/view.log 和 $ROS_LOG_DIR/"
        tail -80 "$LOG_DIR/view.log" 2>/dev/null || true
        do_stop
        return 1
    fi
    ok "Nav2 RViz 已启动 (pid $nav_rviz_pid, DISPLAY=$gui_display)"

    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  全链路启动完成！                            ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  RViz 窗口:                                  ║${NC}"
    echo -e "${GREEN}║  - LIO RViz: 点云/里程计（右侧）              ║${NC}"
    echo -e "${GREEN}║  - Nav2 3D RViz: 路径/障碍方块（左侧）        ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  发送导航目标 (建议用命令行):                ║${NC}"
    echo -e "${GREEN}║    ./goto.sh 20 0    # 前进 20 米            ║${NC}"
    echo -e "${GREEN}║    ./goto.sh 15 15   # 斜向右前方            ║${NC}"
    echo -e "${GREEN}║    ./goto.sh cancel  # 取消当前目标          ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  已启用:                                     ║${NC}"
    echo -e "${GREEN}║  • 距离过滤 (只保留 30m 内点云, 去除地平线)     ║${NC}"
    echo -e "${GREEN}║  • 高度过滤 (z>0.3m, 只保留岸线/浮标结构)       ║${NC}"
    echo -e "${GREEN}║  • Point-LIO 直接导航 (无 EKF, 无 GPS)          ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  日志目录: .usv_logs/                        ║${NC}"
    echo -e "${GREEN}║  停止全部: ./usv.sh stop                     ║${NC}"
    echo -e "${GREEN}║  重新加载配置: ./usv.sh restart              ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
    echo ""
}

# ── 入口 ───────────────────────────────────────────────────
case "${1:-start}" in
    start)  do_start  ;;
    stop)   do_stop   ;;
    status) do_status ;;
    restart) do_stop; sleep 2; do_start ;;
    *)
        echo "用法: $0 [start|stop|status|restart]"
        exit 1
        ;;
esac
