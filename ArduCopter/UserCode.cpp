#include "Copter.h"
#define USERHOOK_INIT
#define USERHOOK_SUPERSLOWLOOP
#include <cstdio>
#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL& hal;
#include <AP_Math/AP_Math.h>
void send_status_text(const char* text);

void send_status_text(const char* text) {
    if (GCS::get_singleton() != nullptr) {
        GCS::get_singleton()->send_text(MAV_SEVERITY_INFO, "%s", text);
    }
}


#include "mode.h"



// ====================== 新增：任务控制核心变量 ======================
static bool mission_trigger = false;       // 任务触发标志（外部置true则启动任务）
static bool mission_running = false;       // 任务运行中标志（防止重复触发）
// ====================== 原有变量保留 ======================
static uint8_t mission_step = 0;
static uint32_t step_time = 0;
static Vector3p target_pos_ned_m;
static bool mission_initialized = false;
static bool step0_armed = false;
static bool takeoff_triggered = false;
static bool ignore_prompted = false;
static bool last_rc7_high = false;  // 记录上一次RC7是否为高
static bool sensors_ready_prompted = false;
static uint32_t last_mission_reset_msg_ms = 0; // 重置消息限频（ms），0表示未打印过
static bool home_wait_prompted = false;  // Home未就绪提示只发一次
static uint32_t last_prearm_wait_msg_ms = 0;
static uint32_t last_arm_attempt_ms = 0;
static uint32_t last_arm_fail_msg_ms = 0;

// 读取当前相对原点的 NED 位置（米），不控制电机。返回 true 表示获取成功并填充 out_pos。
static bool get_current_relative_pos_NED(Vector3p &out_pos)
{
    // 调用 AHRS 提供的接口，从 origin 获取相对位置（NED，单位米）
    return AP::ahrs().get_relative_position_NED_origin(out_pos);
}

static bool get_current_optflow_debug(Vector2f &flow_rate, uint8_t &quality)
{
#if AP_OPTICALFLOW_ENABLED
    AP_OpticalFlow *of = AP::opticalflow();
    if (of == nullptr || !of->healthy()) {
        return false;
    }

    flow_rate = of->flowRate();
    quality = of->quality();
    return true;
#else
    UNUSED(flow_rate);
    UNUSED(quality);
    return false;
#endif
}
// 【新增】任务重置函数（执行完/异常结束时调用，恢复初始状态）
void Copter::reset_indoor_mission()
{
    mission_step = 0;
    step_time = 0;
    target_pos_ned_m = Vector3p();
    mission_initialized = false;
    step0_armed = false;
    takeoff_triggered = false;
    mission_running = false;    // 标记任务停止
    mission_trigger = false;  
    ignore_prompted = false;  // 清空触发标志
    
    sensors_ready_prompted = false; // 新增：清空传感器就绪提示标记
    home_wait_prompted = false;
    last_prearm_wait_msg_ms = 0;
    last_arm_attempt_ms = 0;
    last_arm_fail_msg_ms = 0;
    
    if (last_mission_reset_msg_ms == 0 || AP_HAL::millis() - last_mission_reset_msg_ms >= 5000) {
        gcs().send_text(MAV_SEVERITY_INFO, "Mission reset! Ready for next trigger");
        last_mission_reset_msg_ms = AP_HAL::millis();
    }
}

void Copter::trigger_indoor_mission()
{
    // 新增：静态变量标记「是否已经提示过“任务运行中”」，避免重复刷屏
  

    // 1. 任务已在运行 → 只提示一次，后续静默
    if (mission_running) {
        if (!ignore_prompted) {
            gcs().send_text(MAV_SEVERITY_WARNING, "Mission is running! Ignore trigger");
            ignore_prompted = true; // 标记已提示，后续不再打印
        }
        return;
    }

    // 2. 任务未运行 → 正常触发，重置提示标记
    if (!mission_trigger) { // 仅当未触发时才打印，避免重复触发重复打印
        mission_trigger = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Mission triggered! Starting...");
    }
    ignore_prompted = false; // 重置提示标记，为下次触发做准备
    last_mission_reset_msg_ms = 0; // 允许下一次重置时立即打印（节流重置）
}

// 实现Copter类的indoor_mission成员函数（带启停控制+无GPS适配）
void Copter::indoor_mission()
{
      const float POS_REACHED_THRESHOLD = 0.1f;
    const uint32_t TAKEOFF_TIMEOUT_MS = 15000;
    const uint32_t MOVE_TIMEOUT_MS = 8000; 
    const float TAKEOFF_TARGET_ALT_M = 1.0f;
        const uint32_t STATUS_THROTTLE_MS = 1000;

        static uint32_t last_ekf_wait_msg_ms = 0;
        static uint32_t last_relpos_wait_msg_ms = 0;

     uint16_t rc7_pwm = RC_Channels::get_radio_in(6);
  //trigger_indoor_mission();
      bool curr_rc7_high = (rc7_pwm > 1800);
    if (curr_rc7_high && !last_rc7_high) {
        // 上升沿：从低变高 → 触发任务
        trigger_indoor_mission();
    } else if (rc7_pwm < 1200) {
        // 低电平：重置任务
      
        reset_indoor_mission();
    }
    last_rc7_high = curr_rc7_high; // 更新上一次状态


     if (mission_trigger && !mission_running) {
        mission_running = true;
    }

    // 只有“未触发且未运行”，才退出（任务没开始才不执行）
    if (!mission_trigger && !mission_running) {
        return;
    }
    
    // 核心配置
  

    // ====================== 第一步：任务触发判断 ======================
    // 1. 未触发/运行中 → 直接返回
   // 只有仿真环境才会生效
    // ====================== SITL RC触发核心逻辑 ======================
    // 读取RC通道7的值（索引从0开始，通道7对应索引6）
    // 注：SITL里RC值范围是1000~2000（1500是中位）
      // 通道 7 → 索引 6（1~16通道 → 0~15索引）
   
    //
    // 2. 触发且未运行 → 标记为运行中
   

    // ====================== 第二步：无GPS健康检查（替换原position_ok()） ======================
        static uint32_t ekf_wait_start = 0;
    if (ekf_wait_start == 0) ekf_wait_start = AP_HAL::millis();
    

    if (!AP::ahrs().home_is_set()) {
        Location home_loc;
        // 1. 接收 get_location 的返回值，检查是否获取位置成功
        bool get_loc_ok = AP::ahrs().get_location(home_loc);
        if (get_loc_ok) {
            // 2. 接收 set_home 的返回值，检查是否设置 Home 点成功
            bool set_home_ok = AP::ahrs().set_home(home_loc);
            if (set_home_ok) {
                gcs().send_text(MAV_SEVERITY_INFO, "Home set via AP::ahrs() in no-GPS mode.");
                home_wait_prompted = false;
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Failed to set home from current location.");
            }
        } else {
            // 兜底逻辑：获取当前位置失败，尝试用 AHRS 原点设置
            Location origin_loc;
            bool get_origin_ok = AP::ahrs().get_origin(origin_loc);
            if (get_origin_ok) {
                bool set_home_from_origin_ok = AP::ahrs().set_home(origin_loc);
                if (set_home_from_origin_ok) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Home set from AHRS origin.");
                    home_wait_prompted = false;
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Failed to set home from AHRS origin.");
                }
            } else if (!home_wait_prompted) {
                // During EKF/flow startup this can be expected; keep this one-shot per mission.
                gcs().send_text(MAV_SEVERITY_INFO, "Home not set yet: waiting for EKF origin/position.");
                home_wait_prompted = true;
            }
        }
    }

     

    // 1. 检查AHRS（EKF）健康
    if (!ahrs.healthy()) {
        if (AP_HAL::millis() - ekf_wait_start < 10000) { // 10秒内持续提示
            if (AP_HAL::millis() - last_ekf_wait_msg_ms >= STATUS_THROTTLE_MS) {
                gcs().send_text(MAV_SEVERITY_INFO, "Waiting for EKF healthy (flow/rangefinder)...");
                last_ekf_wait_msg_ms = AP_HAL::millis();
            }
        } else { // 10秒超时后，仿真环境强制跳过（避免卡死）
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF wait timeout, skip check (sim only)!");
        }
        mission_running = false;
        // 10秒超时后不return，继续执行（仿真专属容错）
        if (AP_HAL::millis() - ekf_wait_start < 10000) return;
    }

    // 2. 检查EKF相对位置（仿真适配）
    // 优先使用AHRS相对位置；若其暂不可用，则允许使用pos_control估计作为fallback继续任务。
    bool rel_pos_ok = ekf_has_relative_position();
    bool rel_pos_fallback_ok = false;
    if (!rel_pos_ok) {
        nav_filter_status nav_status{};
        const bool nav_status_ok = AP::ahrs().get_filter_status(nav_status);
        Location origin_loc;
        const bool origin_ok = AP::ahrs().get_origin(origin_loc);

#if AP_OPTICALFLOW_ENABLED
        AP_OpticalFlow *of = AP::opticalflow();
        const bool flow_ok = (of != nullptr) && of->enabled() && of->healthy() && (of->quality() > 0);
#else
        const bool flow_ok = false;
#endif

        const Vector3p &pos_est_ned_m = pos_control->get_pos_estimate_NED_m();
        const bool pos_est_moved = !is_zero((float)pos_est_ned_m.z);

        rel_pos_fallback_ok = nav_status_ok && nav_status.flags.initalized && nav_status.flags.horiz_vel &&
                              AP::ahrs().home_is_set() && origin_ok && flow_ok && pos_est_moved;

        if (!rel_pos_fallback_ok) {
            if (AP_HAL::millis() - ekf_wait_start < 10000) {
                if (AP_HAL::millis() - last_relpos_wait_msg_ms >= STATUS_THROTTLE_MS) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Waiting for relative position (flow)...");
                    last_relpos_wait_msg_ms = AP_HAL::millis();
                }
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "Relative position wait timeout, skip check (sim only)!");
            }
            mission_running = false;
            if (AP_HAL::millis() - ekf_wait_start < 10000) return;
        } else if (AP_HAL::millis() - last_relpos_wait_msg_ms >= STATUS_THROTTLE_MS) {
            gcs().send_text(MAV_SEVERITY_INFO, "Relative pos fallback active: using pos_control estimate");
            last_relpos_wait_msg_ms = AP_HAL::millis();
        }
    }

    // 3. 检查测距仪数据（核心修复：改用仿真默认朝向 ROTATION_NONE）
    // 可选值：ROTATION_NONE / ROTATION_PITCH_90（仿真都兼容）
  /*  if (!rangefinder.has_data_orient(ROTATION_NONE)) {
        if (AP_HAL::millis() - ekf_wait_start < 10000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Waiting for rangefinder data...");
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Rangefinder wait timeout, skip check (sim only)!");
        }
        mission_running = false;
        if (AP_HAL::millis() - ekf_wait_start < 10000) return;
    }*/

// 健康检查通过，重置超时计时，提示就绪
    if (!sensors_ready_prompted) {
        gcs().send_text(MAV_SEVERITY_INFO, "All sensors ready! Start mission...");
        sensors_ready_prompted = true;
        
        // 【新增】传感器就绪后立即尝试设置EKF原点（无GPS环境下这很关键）
        set_ekf_origin_from_current();
    }

    ekf_wait_start = 0;
    mission_initialized = true;
    // ====================== 第三步：原有任务逻辑（仅新增任务结束重置） ======================
    switch (mission_step)
    {
    //--------------------------------
    // Step0 解锁 + 切换Guided + 起飞1米
    //--------------------------------
    case 0:
    {
        if (!step0_armed) {
            // 1. 先切换到 Guided 模式（必须在解锁前）
            if (flightmode->mode_number() != Mode::Number::GUIDED) {
                bool mode_ok = set_mode(Mode::Number::GUIDED, ModeReason::SCRIPTING);
                if (!mode_ok) {
                    gcs().send_text(MAV_SEVERITY_ERROR, "Switch GUIDED failed!");
                    reset_indoor_mission(); // 切换失败，重置任务
                 
                    return;
                }
                gcs().send_text(MAV_SEVERITY_INFO, "Switched to GUIDED");
                step_time = AP_HAL::millis();
                return;
            }

            // 2. 等待模式切换稳定
            if (AP_HAL::millis() - step_time < 500) {
                return;
            }

            // 3. 解锁电机
            if (!motors->armed()) {
                const uint32_t now_ms = AP_HAL::millis();

                // Pre-arm未通过时不尝试解锁，避免Gyro等检查失败刷屏
                if (!AP::arming().pre_arm_checks(false)) {
                    if (now_ms - last_prearm_wait_msg_ms >= STATUS_THROTTLE_MS) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Pre-arm checks not ready, waiting...");
                        last_prearm_wait_msg_ms = now_ms;
                    }
                    return;
                }

                // 通过检查后，解锁尝试也做限频，避免瞬时重复调用
                if (now_ms - last_arm_attempt_ms < STATUS_THROTTLE_MS) {
                    return;
                }
                last_arm_attempt_ms = now_ms;

                bool arm_ok = AP::arming().arm(AP_Arming::Method::SCRIPTING, true);
                if (!arm_ok) {
                    if (now_ms - last_arm_fail_msg_ms >= STATUS_THROTTLE_MS) {
                        gcs().send_text(MAV_SEVERITY_WARNING, "Arm failed, retrying...");
                        last_arm_fail_msg_ms = now_ms;
                    }
                   // reset_indoor_mission(); // 解锁失败，重置任务
                    return;
                }
                gcs().send_text(MAV_SEVERITY_INFO, "Armed success");
            }

            // 4. 开启auto_armed，防止自动上锁
            set_auto_armed(true);
            gcs().send_text(MAV_SEVERITY_INFO, "Auto-armed enabled");

            // 5. 标记Step0完成
            step0_armed = true;
            step_time = AP_HAL::millis();
        }

        // 6. 等待解锁后稳定
        if (AP_HAL::millis() - step_time < 1000) {
            return;
        }

        // 7. 初始化起飞（只执行一次）
        if (!takeoff_triggered) {
            bool takeoff_ok = mode_guided.do_user_takeoff_start_m(TAKEOFF_TARGET_ALT_M);
            if (takeoff_ok) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff start (1m)");
                step_time = AP_HAL::millis();
                takeoff_triggered = true;
            } else {
                gcs().send_text(MAV_SEVERITY_ERROR, "Takeoff init failed!");
                reset_indoor_mission(); // 起飞失败，重置任务
              
                return;
            }
        }

        // 8. 判断起飞完成（改用测距仪高度，适配无GPS）
     float posD;
     ahrs.get_relative_position_D_home(posD);
     float curr_alt_m = -posD;
        gcs().send_text(MAV_SEVERITY_DEBUG, "Current alt: %.2f, is_taking_off: %d", curr_alt_m, mode_guided.is_taking_off());

        bool takeoff_finished = !mode_guided.is_taking_off() && 
                                (curr_alt_m >= TAKEOFF_TARGET_ALT_M * 0.9f) && 
                                (AP_HAL::millis() - step_time > 2000);

        if (takeoff_finished) {
            gcs().send_text(MAV_SEVERITY_INFO, "Takeoff complete (alt:%.2fm)", curr_alt_m);
            mission_step = 1;
            step_time = AP_HAL::millis(); 
            takeoff_triggered = false;
        }

        // 9. 起飞超时保护（超时则重置任务）
        if (AP_HAL::millis() - step_time > TAKEOFF_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Takeoff timeout (alt:%.2fm)", curr_alt_m);
            reset_indoor_mission(); // 超时，重置任务
           
            mission_step = 6;
        }

        break;
    }

    //--------------------------------
    // Step1 前进1m（核心修复）
    //--------------------------------
    case 1:
    {
        // 进入Step1后先等待0.5秒稳定
        if (AP_HAL::millis() - step_time < 500) {
            break;
        }

        // 【关键修复】目标位置仅计算+下发一次，固定不变
        static bool pos_set = false;
        if (!pos_set) {
            // 仅第一次进入时，基于当前位置计算目标
            const Vector3p& init_pos_ned_m = pos_control->get_pos_estimate_NED_m();
            target_pos_ned_m = Vector3p(
                init_pos_ned_m.x + 1.0f, // X轴+1米（前进），固定目标
                init_pos_ned_m.y,
                init_pos_ned_m.z
            );

            // 下发目标位置
            bool set_pos_ok = mode_guided.set_pos_NED_m(
                target_pos_ned_m, false, 0.0f, false, 0.0f, false, false
            );
            if (!set_pos_ok) {
                gcs().send_text(MAV_SEVERITY_ERROR, "Set forward pos failed!");
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Forward 1m (target X:%.2f)", target_pos_ned_m.x);
            pos_set = true;
        }

        // 【关键修复】每次循环都获取实时当前位置，和固定目标计算距离
        const Vector3p& curr_pos_ned_m = pos_control->get_pos_estimate_NED_m();
        float dist_to_target = get_horizontal_distance(curr_pos_ned_m.xy(), target_pos_ned_m.xy());
        
        // 调试日志，实时输出距离
        gcs().send_text(MAV_SEVERITY_DEBUG, "Forward dist: %.2fm", dist_to_target);

        // 判断是否到达目标
        if (dist_to_target < POS_REACHED_THRESHOLD) {
            gcs().send_text(MAV_SEVERITY_INFO, "Reached forward target (dist:%.2fm)", dist_to_target);
            mission_step = 2;
            step_time = AP_HAL::millis(); 
            pos_set = false; 
        }

        // 移动超时保护（超时则重置任务）
        if (AP_HAL::millis() - step_time > MOVE_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Forward move timeout! Final dist:%.2fm", dist_to_target);
            reset_indoor_mission(); // 超时，重置任务
           
            mission_step = 6;
        }

        break;
    }

    //--------------------------------
    // Step2 悬停2秒
    //--------------------------------
    case 2:
    {
        mode_guided.hold_position();

        if (AP_HAL::millis() - step_time > 2000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Hold 2s complete");
            mission_step = 3;
            step_time = AP_HAL::millis(); 
        }
        break;
    }

    //--------------------------------
    // Step3 左移1m（同步修复，和Step1逻辑一致）
    //--------------------------------
    case 3:
    {
        // 进入Step3后先等待0.5秒稳定
        if (AP_HAL::millis() - step_time < 500) {
            break;
        }

        // 目标位置仅计算+下发一次，固定不变
        static bool pos_set = false;
        if (!pos_set) {
            // 仅第一次进入时，基于当前位置计算目标
            const Vector3p& init_pos_ned_m = pos_control->get_pos_estimate_NED_m();
            target_pos_ned_m = Vector3p(
                init_pos_ned_m.x,
                init_pos_ned_m.y - 1.0f, // Y轴-1米（左移），固定目标
                init_pos_ned_m.z
            );

            // 下发目标位置
            bool set_pos_ok = mode_guided.set_pos_NED_m(
                target_pos_ned_m, false, 0.0f, false, 0.0f, false, false
            );
            if (!set_pos_ok) {
                gcs().send_text(MAV_SEVERITY_ERROR, "Set left pos failed!");
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Left 1m (target Y:%.2f)", target_pos_ned_m.y);
            pos_set = true;
        }

        // 每次循环获取实时位置，计算到固定目标的距离
        const Vector3p& curr_pos_ned_m = pos_control->get_pos_estimate_NED_m();
        float dist_to_target = get_horizontal_distance(curr_pos_ned_m.xy(), target_pos_ned_m.xy());
        
        // 调试日志
        gcs().send_text(MAV_SEVERITY_DEBUG, "Left dist: %.2fm", dist_to_target);

        // 判断是否到达目标
        if (dist_to_target < POS_REACHED_THRESHOLD) {
            gcs().send_text(MAV_SEVERITY_INFO, "Reached left target (dist:%.2fm)", dist_to_target);
            mission_step = 4;
            step_time = AP_HAL::millis();
            pos_set = false;
        }

        // 移动超时保护（超时则重置任务）
        if (AP_HAL::millis() - step_time > MOVE_TIMEOUT_MS) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Left move timeout! Final dist:%.2fm", dist_to_target);
            reset_indoor_mission(); // 超时，重置任务
        
            mission_step = 6;
        }

        break;
    }

    //--------------------------------
    // Step4 悬停2秒
    //--------------------------------
    case 4:
    {
        mode_guided.hold_position();

        if (AP_HAL::millis() - step_time > 2000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Hold 2s complete");
            mission_step = 5;
            step_time = AP_HAL::millis();
        }
        break;
    }

    //--------------------------------
    // Step5 降落
    //--------------------------------
    case 5:
    {
        if (flightmode->mode_number() != Mode::Number::LAND) {
            bool land_ok = set_mode(Mode::Number::LAND, ModeReason::SCRIPTING);
            if (!land_ok) {
                gcs().send_text(MAV_SEVERITY_ERROR, "Switch LAND failed!");
                break;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Landing start");
        }

        if (ap.land_complete) {
            gcs().send_text(MAV_SEVERITY_INFO, "Landing complete!");
            AP::arming().disarm(AP_Arming::Method::SCRIPTING);
            set_auto_armed(false); 
            mission_step = 6;
        }

        break;
    }

    //--------------------------------
    // Step6 任务完成 → 自动重置，等待下一次触发
    //--------------------------------
    case 6:
    {
        if (step_time == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Mission complete! Ready for re-trigger");
            step_time = AP_HAL::millis();
        }
        // 任务完成后延迟1秒重置（确保状态稳定）
        if (AP_HAL::millis() - step_time > 1000) {
            reset_indoor_mission(); // 自动重置，可再次触发
           
        }
        break;
    }
    }
}



// 【新增】手动设置EKF原点 - 从当前AHRS位置或指定经纬度/高度
void Copter::set_ekf_origin_from_current()
{
    Location loc;
    
    // 优先尝试从AHRS获取当前位置
    if (AP::ahrs().get_location(loc)) {
        if (AP::ahrs().set_origin(loc)) {
            gcs().send_text(MAV_SEVERITY_INFO, "✓ EKF Origin SET from AHRS location: Lat=%.7f, Lon=%.7f, Alt=%.2fm",
                            (double)loc.lat * 1e-7, (double)loc.lng * 1e-7, (double)loc.alt * 0.01);

            // In no-GPS testing, origin alone is not enough for some relative position APIs.
            if (!AP::ahrs().home_is_set()) {
                if (AP::ahrs().set_home(loc)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "✓ Home SET together with EKF origin");
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "✗ EKF origin set but Home set failed");
                }
            }
            return;
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "✗ Failed to set EKF origin from AHRS location");
        }
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "✗ AHRS location not available, cannot set origin");
    }
}

// 【新增】将 EKF 原点设置为固定测试坐标（用于本地化/zg函数测试）
void Copter::set_ekf_origin_to_test()
{
    Location loc;
    // 示例测试坐标：经纬度使用 1e7 单位，海拔使用厘米
    // 请根据你的测试场景修改下面的坐标（目前为: Lat=30.0000000 Lon=120.0000000 Alt=1.00m）
    loc.lat = (int32_t)(30.0 * 1e7);
    loc.lng = (int32_t)(120.0 * 1e7);
    loc.alt = (int32_t)(1.0 * 100.0); // 1.00 m -> 100 cm

    if (AP::ahrs().set_origin(loc)) {
        gcs().send_text(MAV_SEVERITY_INFO, "✓ EKF Test Origin SET: Lat=%.7f Lon=%.7f Alt=%.2fm",
                        (double)loc.lat * 1e-7, (double)loc.lng * 1e-7, (double)loc.alt * 0.01);

        if (!AP::ahrs().home_is_set()) {
            if (AP::ahrs().set_home(loc)) {
                gcs().send_text(MAV_SEVERITY_INFO, "✓ Home SET from test origin");
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING, "✗ Test origin set but Home set failed");
            }
        }
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "✗ Failed to set EKF Test Origin");
    }
}

void Copter::handle_custom_mavlink_command(const mavlink_command_long_t& cmd)
{
    if (cmd.command == MAV_CMD_USER_1) { // 自定义指令1作为触发信号
        trigger_indoor_mission();
    } else if (cmd.command == MAV_CMD_USER_2) { // 自定义指令2作为强制重置
        reset_indoor_mission();
      
        gcs().send_text(MAV_SEVERITY_INFO, "Mission forced reset!");
    }
}

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // 在初始化时尝试通过当前 AHRS/GPS 位置设置 EKF 原点/Home，便于无GPS或仿真环境下定位
    set_ekf_origin_from_current();
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // 每秒分步骤验证：惯导 -> 光流 -> 相对位置
    static uint32_t last_report_ms = 0;
    const uint32_t REPORT_INTERVAL_MS = 1000;

    if (AP_HAL::millis() - last_report_ms < REPORT_INTERVAL_MS) {
        return;
    }
    last_report_ms = AP_HAL::millis();

    // RC8 上升沿触发：设置为测试原点（只触发一次上升沿）
    static bool last_rc8_high = false;
    uint16_t rc8_pwm = RC_Channels::get_radio_in(7);
    bool rc8_high = (rc8_pwm > 1800);
    if (rc8_high && !last_rc8_high) {
        // 上升沿：执行一次测试原点设置
        set_ekf_origin_to_test();
    }
    last_rc8_high = rc8_high;

    nav_filter_status nav_status{};
    const bool nav_status_ok = AP::ahrs().get_filter_status(nav_status);

    if (!AP::ahrs().have_inertial_nav()) {
        send_status_text("Step1: inertial nav not ready");
        return;
    }

    char status_buf[160];

    Location ahrs_loc;
    const bool current_loc_ok = AP::ahrs().get_location(ahrs_loc);
    Location origin_loc;
    const bool origin_ok = AP::ahrs().get_origin(origin_loc);
    bool home_ok = AP::ahrs().home_is_set();

    // Keep trying to set Home from origin in no-GPS test cases.
    if (!home_ok && origin_ok) {
        if (AP::ahrs().set_home(origin_loc)) {
            home_ok = true;
            send_status_text("Step1a+: home set from origin in SuperSlowLoop");
        }
    }

    snprintf(status_buf, sizeof(status_buf), "Step1a: home:%d origin:%d loc:%d", (int)home_ok, (int)origin_ok, (int)current_loc_ok);
    send_status_text(status_buf);

    if (nav_status_ok) {
        snprintf(status_buf, sizeof(status_buf), "Step1b: rel:%d pred:%d abs:%d const:%d", (int)nav_status.flags.horiz_pos_rel, (int)nav_status.flags.pred_horiz_pos_rel, (int)nav_status.flags.horiz_pos_abs, (int)nav_status.flags.const_pos_mode);
        send_status_text(status_buf);

        snprintf(status_buf, sizeof(status_buf), "Step1c: takeoff:%d gps:%d init:%d dead:%d", (int)nav_status.flags.takeoff_detected, (int)nav_status.flags.using_gps, (int)nav_status.flags.initalized, (int)nav_status.flags.dead_reckoning);
        send_status_text(status_buf);
    } else {
        send_status_text("Step1b: nav status unavailable");
    }

    Vector2f flow_rate;
    uint8_t quality = 0;
    if (get_current_optflow_debug(flow_rate, quality)) {
        snprintf(status_buf, sizeof(status_buf), "Step2: optflow q:%u fx:%.3f fy:%.3f", quality, (double)flow_rate.x, (double)flow_rate.y);
        send_status_text(status_buf);
    } else {
        send_status_text("Step2: optflow not ready");
        return;
    }

    Vector3p rel_pos;
    if (get_current_relative_pos_NED(rel_pos)) {
        snprintf(status_buf, sizeof(status_buf), "Step3: Rel N:%.2f E:%.2f D:%.2f", rel_pos.x, rel_pos.y, rel_pos.z);
        send_status_text(status_buf);
    } else {
        send_status_text("Step3: ✗ rel pos unavailable");

        // 【增强调试输出】查看EKF标志详细信息
        if (nav_status_ok) {
            snprintf(status_buf, sizeof(status_buf), "Step3b: rel:%d pred:%d abs:%d const:%d", (int)nav_status.flags.horiz_pos_rel, (int)nav_status.flags.pred_horiz_pos_rel, (int)nav_status.flags.horiz_pos_abs, (int)nav_status.flags.const_pos_mode);
            send_status_text(status_buf);

            snprintf(status_buf, sizeof(status_buf), "Step3c: init:%d dead:%d takeoff:%d gps:%d vel:%d", (int)nav_status.flags.initalized, (int)nav_status.flags.dead_reckoning, (int)nav_status.flags.takeoff_detected, (int)nav_status.flags.using_gps, (int)nav_status.flags.horiz_vel);
            send_status_text(status_buf);
        }

        // 【增强调试输出】查看光流是否启用和融合状态
#if AP_OPTICALFLOW_ENABLED
        AP_OpticalFlow *of = AP::opticalflow();
        if (of != nullptr) {
            snprintf(status_buf, sizeof(status_buf), "Step3d: optflow_enabled:%d optflow_healthy:%d", (int)of->enabled(), (int)of->healthy());
            send_status_text(status_buf);
        }
#endif

        // 【增强调试输出】显示position_control的估计值
        const Vector3p &pos_est_ned_m = pos_control->get_pos_estimate_NED_m();
        snprintf(status_buf, sizeof(status_buf),
                 "Step3e: pos_control_est N:%.2f E:%.2f D:%.2f",
                 pos_est_ned_m.x,
                 pos_est_ned_m.y,
                 pos_est_ned_m.z);
        send_status_text(status_buf);

        // If AHRS relative position is unavailable, surface position-controller estimate as fallback.
        snprintf(status_buf, sizeof(status_buf),
             "Step3e+: fallback_rel N:%.2f E:%.2f D:%.2f",
             pos_est_ned_m.x,
             pos_est_ned_m.y,
             pos_est_ned_m.z);
        send_status_text(status_buf);

        // 【增强调试输出】尝试显示ekf_alt_ok状态
        snprintf(status_buf, sizeof(status_buf), "Step3f: ekf_alt_ok:%d ahrs_healthy:%d motors_armed:%d", (int)ekf_alt_ok(), (int)ahrs.healthy(), (int)motors->armed());
        send_status_text(status_buf);
    }
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
