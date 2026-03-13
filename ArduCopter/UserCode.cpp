#include "Copter.h"
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
    
    gcs().send_text(MAV_SEVERITY_INFO, "Mission reset! Ready for next trigger");
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
}

// 实现Copter类的indoor_mission成员函数（带启停控制+无GPS适配）
void Copter::indoor_mission()
{
      const float POS_REACHED_THRESHOLD = 0.1f;
    const uint32_t TAKEOFF_TIMEOUT_MS = 15000;
    const uint32_t MOVE_TIMEOUT_MS = 8000; 
    const float TAKEOFF_TARGET_ALT_M = 1.0f;

     uint16_t rc7_pwm = RC_Channels::get_radio_in(6);

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
                gcs().send_text(MAV_SEVERITY_INFO, "✅ Home set via AP::ahrs()! No GPS mode.");
            } else {
                gcs().send_text(MAV_SEVERITY_ERROR, "❌ Failed to set home from current location!");
            }
        } else {
            // 兜底逻辑：获取当前位置失败，尝试用 AHRS 原点设置
            Location origin_loc;
            bool get_origin_ok = AP::ahrs().get_origin(origin_loc);
            if (get_origin_ok) {
                bool set_home_from_origin_ok = AP::ahrs().set_home(origin_loc);
                if (set_home_from_origin_ok) {
                    gcs().send_text(MAV_SEVERITY_INFO, "✅ Home set from AHRS origin!");
                } else {
                    gcs().send_text(MAV_SEVERITY_ERROR, "❌ Failed to set home from origin!");
                }
            } else {
                gcs().send_text(MAV_SEVERITY_ERROR, "❌ Failed to get current location and origin! Home not set!");
            }
        }
    }

     

    // 1. 检查AHRS（EKF）健康
    if (!ahrs.healthy()) {
        if (AP_HAL::millis() - ekf_wait_start < 10000) { // 10秒内持续提示
            gcs().send_text(MAV_SEVERITY_INFO, "Waiting for EKF healthy (flow/rangefinder)...");
        } else { // 10秒超时后，仿真环境强制跳过（避免卡死）
            gcs().send_text(MAV_SEVERITY_WARNING, "EKF wait timeout, skip check (sim only)!");
        }
        mission_running = false;
        // 10秒超时后不return，继续执行（仿真专属容错）
        if (AP_HAL::millis() - ekf_wait_start < 10000) return;
    }

    // 2. 检查EKF相对位置（仿真适配）
    if (!ekf_has_relative_position()) {
        if (AP_HAL::millis() - ekf_wait_start < 10000) {
            gcs().send_text(MAV_SEVERITY_INFO, "Waiting for relative position (flow)...");
        } else {
            gcs().send_text(MAV_SEVERITY_WARNING, "Relative position wait timeout, skip check (sim only)!");
        }
        mission_running = false;
        if (AP_HAL::millis() - ekf_wait_start < 10000) return;
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
                bool arm_ok = AP::arming().arm(AP_Arming::Method::SCRIPTING, true);
                if (!arm_ok) {
                    gcs().send_text(MAV_SEVERITY_ERROR, "Arm failed!");
                   // reset_indoor_mission(); // 解锁失败，重置任务
                    mission_running = false; 
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

// ====================== 可选：添加地面站触发方式（测试用） ======================
// 在MAVProxy控制台输入 "trigger_mission" 即可触发任务
// 需在Copter类中注册该指令（参考ArduPilot自定义指令文档）
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
    // put your initialisation code here
    // this will be called once at start-up
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
   /*  hal.console->println("hello 1");  
    Vector3f euler = attitude_control->get_att_target_euler_rad();
    float roll_deg  = euler.x * RAD_TO_DEG;
    float pitch_deg = euler.y * RAD_TO_DEG;
    float yaw_deg   = euler.z * RAD_TO_DEG;

    hal.console->printf("roll: %.2f, pitch: %.2f, yaw: %.2f\n", roll_deg, pitch_deg, yaw_deg);

   
 // 发送 MAVLink STATUSTEXT
    char msg[50];
    snprintf(msg, sizeof(msg), "roll: %.2f, pitch: %.2f, yaw: %.2f", roll_deg, pitch_deg, yaw_deg);
    send_status_text(msg);
  
*/
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
