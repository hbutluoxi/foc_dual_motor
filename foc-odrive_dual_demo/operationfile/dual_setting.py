from odrive.utils import *
import odrive


# Deng FOC参数
board_parameter = {"brake_res": 0,  # 耗散电阻值(根据实际接入的功率电阻值输入，不接为0)
                   "dc_max_pos_cur": 25,  # 电源的过流保护的电流值
                   "dc_max_neg_cur": -4.0,  # 反向电流的过流保护阈值
                   "under_volt_level": 8.0,  # 欠压保护阈值
                   "over_volt_level": 30.0,  # 过压保护阈值
                   "max_regen_current": 5  # 制动回流电流值
                   }
# 电机相关参数
motor0_parameter = {"pole_pairs": 7,  # 电机的极对数
                   "kv": 980,  # 电机kv值
                   "motor_type": MOTOR_TYPE_HIGH_CURRENT,  # 电机类型
                   "cur_lim": 25,  # 电机电流限制（A）
                   "cal_cur": 10,  # 电机校准电流限制（A）
                   "cal_vol": 2,  # 电机校准电压限制（v)
                   "requested_cur_range": 80,  # 电流采样范围（A）
                   }
motor1_parameter = {"pole_pairs": 7,  # 电机的极对数
                   "kv": 980,  # 电机kv值
                   "motor_type": MOTOR_TYPE_HIGH_CURRENT,  # 电机类型
                   "cur_lim": 25,  # 电机电流限制（A）
                   "cal_cur": 10,  # 电机校准电流限制（A）
                   "cal_vol": 2,  # 电机校准电压限制（v)
                   "requested_cur_range": 80,  # 电流采样范围（A）
                   }


# 编码器参数 for AS5047P
encoder0_parameter = {"mode": ["5047_ABI", "5047_SPI"],
                     "encoder_mode": [ENCODER_MODE_INCREMENTAL, ENCODER_MODE_SPI_ABS_AMS],  # 编码器模式
                     "cpr": [4000, 2 ** 14],  # 编码器cpr
                     "bandwidth": 3000,  # 带宽
                     "encoder_cs_pin":5,  # SPI CS引脚
                     "cali_cur": 10,  # 配置编码器时校准电机的电流
                     "cali_ramp_dis": 3.1415927410125732,  # 配置编码器时电机转动距离.
                     "cali_ramp_time": 0.4,  # 配置编码器时电流提高的时间
                     "cali_accel": 20,  # 配置编码器时电机的加速度
                     "cali_vel": 40,  # 配置编码器时电机的速度
                     "calib_range": 0.2,
                     }

encoder1_parameter = {"mode": ["5047_ABI", "5047_SPI"],
                     "encoder_mode": [ENCODER_MODE_INCREMENTAL, ENCODER_MODE_SPI_ABS_AMS],  # 编码器模式
                     "cpr": [4000, 2 ** 14],  # 编码器cpr
                     "bandwidth": 3000,  # 带宽
                     "encoder_cs_pin": 6,  # SPI CS引脚
                     "cali_cur": 10,  # 配置编码器时校准电机的电流
                     "cali_ramp_dis": 3.1415927410125732,  # 配置编码器时电机转动距离.
                     "cali_ramp_time": 0.4,  # 配置编码器时电流提高的时间
                     "cali_accel": 20,  # 配置编码器时电机的加速度
                     "cali_vel": 40,  # 配置编码器时电机的速度
                     "calib_range": 0.2,
                     }
# 控制器基本参数
controller_base_parameter = {"vel_lim": 20,  # 速度限制（[turn/s）
                             "vel_gain": 0.02,  # 速度环增益
                             "pos_gain": 30,  # 位置环增益
                             "vel_integrator": 0.2,  # 速度环积分
                             }
# 控制模式
contorl_mode = ["vel_passthrough", "vel_ramp", "pos_passthrough", "pos_trap_traj", "torque_passthrough", "torque_ramp"]
# 速度模式
velocity_mode_parameter = {"control_mode": CONTROL_MODE_VELOCITY_CONTROL,  # 设置速度控制
                           "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_VEL_RAMP],  # 设置输入模式
                           "vel_ramp_rate": 5,  # 设置速度爬升速率
                           "inertia": 0.1,  # 设置惯性
                           }
# 位置模式
position_mode_parameter = {"control_mode": CONTROL_MODE_POSITION_CONTROL,  # 设置位置控制
                           "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_TRAP_TRAJ],  # 设置输入模式
                           "trap_traj_vel_lim": 30,  # 梯形轨迹速度限制
                           "trap_traj_accel_limit": 5,  # 梯形轨迹加速度限制
                           "trap_traj_decel_limit": 5,  # 梯形轨迹减速度限制
                           }
# 力矩模式
torque_mode_parameter = {"control_mode": CONTROL_MODE_VELOCITY_CONTROL,  # 设置力矩控制
                         "input_mode": [INPUT_MODE_PASSTHROUGH, INPUT_MODE_TORQUE_RAMP],  # 设置输入模式
                         "tor_ramp_rate": 0,  # 设置力矩爬升模式
                         }


def USBerror():
    print("Device disconnected")
    
def set_board_param(odrv, board_param):
    odrv.config.brake_resistance = board_param["brake_res"]
    odrv.config.dc_bus_undervoltage_trip_level = board_param["under_volt_level"]
    odrv.config.dc_bus_overvoltage_trip_level = board_param["over_volt_level"]
    odrv.config.dc_max_positive_current = board_param["dc_max_pos_cur"]
    odrv.config.dc_max_negative_current = board_param["dc_max_neg_cur"]
    odrv.config.max_regen_current = board_param["max_regen_current"]



def set_motor_param(axis, motor_param):
    print("正在设置电机参数")
    axis.motor.config.pole_pairs = motor_param["pole_pairs"]
    axis.motor.config.motor_type = motor_param["motor_type"]
    axis.motor.config.current_lim = motor_param["cur_lim"]
    axis.motor.config.calibration_current = motor_param["cal_cur"]
    axis.motor.config.resistance_calib_max_voltage = motor_param["cal_vol"]
    axis.motor.config.requested_current_range = motor_param["requested_cur_range"]
    axis.motor.config.torque_constant = 8.27 / motor_param["kv"]

def set_encoder_param(axis, encoder_param):
    encoder_mode = int(input("选择编码器模式：\n"
                         "1.AS5047P-ABI(每次上电必须来回转以校准编码器)\n"
                         "2.AS5047P-SPI\n"))
    if encoder_mode == 1:
        print("正在设置编码器为ABI模式")
        axis.encoder.config.mode = encoder_param["encoder_mode"][0]
        axis.encoder.config.cpr = encoder_param["cpr"][0]
    elif encoder_mode == 2:
        print("正在设置编码器为SPI模式")
        axis.encoder.config.mode = encoder_param["encoder_mode"][1]
        axis.encoder.config.cpr = encoder_param["cpr"][1]
        axis.encoder.config.abs_spi_cs_gpio_pin = encoder_param["encoder_cs_pin"]
        axis.encoder.config.calib_range = encoder_param["calib_range"]


    axis.encoder.config.bandwidth = encoder_param["bandwidth"]
    axis.config.calibration_lockin.current = encoder_param["cali_cur"]
    axis.config.calibration_lockin.ramp_distance = encoder_param["cali_ramp_dis"]
    axis.config.calibration_lockin.ramp_time = encoder_param["cali_ramp_time"]
    axis.config.calibration_lockin.accel = encoder_param["cali_accel"]
    axis.config.calibration_lockin.vel = encoder_param["cali_vel"]

def set_controller_param(axis, controller_base_param, ):
    # 设置控制器的基本参数
    axis.controller.config.vel_limit = controller_base_param["vel_lim"]
    axis.controller.config.vel_gain = controller_base_param["vel_gain"]
    axis.controller.config.pos_gain = controller_base_param["pos_gain"]
    axis.controller.config.vel_integrator_gain = controller_base_param["vel_integrator"]

def set_velocity_mode(axis, velocity_mode_param, mode):
    print("设置电机为速度模式")

    axis.controller.config.control_mode = velocity_mode_param["control_mode"]
    if mode == "passthrough":
        axis.controller.config.input_mode = velocity_mode_param["input_mode"][0]
    elif mode == "velocity_ramp":
        axis.controller.config.input_mode = velocity_mode_param["input_mode"][1]
        axis.controller.config.vel_ramp_rate = velocity_mode_param["vel_ramp_rate"]
        axis.controller.config.inertia = velocity_mode_param["inertia"]


def set_torque_mode(axis, torque_mode_param, mode):
    print("设置电机为扭矩模式")

    axis.controller.config.control_mode = torque_mode_param["control_mode"]
    if mode == "passthrough":
        axis.controller.config.input_mode = torque_mode_param["input_mode"][0]
    elif mode == "torque_ramp":
        axis.controller.config.input_mode = torque_mode_param["input_mode"][1]
        axis.controller.config.torque_ramp_rate = torque_mode_param["tor_ramp_rate"]


def set_position_mode(axis, position_mode_param, mode):
    print("设置电机为位置模式")

    axis.controller.config.control_mode = position_mode_param["control_mode"]
    if mode == "passthrough":
        axis.controller.config.input_mode = position_mode_param["input_mode"][0]
    elif mode == "position_traj_ramp":
        axis.controller.config.input_mode = position_mode_param["input_mode"][1]
        axis.trap_traj.config.vel_limit = position_mode_param["trap_traj_vel_lim"]
        axis.trap_traj.config.accel_limit = position_mode_param["trap_traj_accel_limit"]
        axis.trap_traj.config.decel_limit = position_mode_param["trap_traj_decel_limit"]

def set_control_mode(axis):
    set_controller_param(axis, controller_base_parameter)  # 设置控制器基本参数
    mode = int(input("需要设置该电机为：\n"
                     "0.速度直接模式\n"
                     "1.速度爬升模式\n"
                     "2.位置直接模式\n"
                     "3.位置梯形轨迹模式\n"
                     "4.力矩直接模式\n"
                     "5.力矩爬升模式\n"))
    if mode == 0:
        set_velocity_mode(axis, velocity_mode_parameter, "passthrough")  # 速度直接模式
    elif mode == 1:
        set_velocity_mode(axis, velocity_mode_parameter, "velocity_ramp")  # 速度爬升模式
    elif mode == 2:
        set_position_mode(axis, position_mode_parameter, "passthrough")  # 位置直接模式
    elif mode == 3:
        set_position_mode(axis, position_mode_parameter, "position_traj_ramp")  # 位置梯形轨迹模式
    elif mode == 4:
        set_torque_mode(axis, torque_mode_parameter, "passthrough")  # 力矩直接模式
    elif mode == 5:
        set_torque_mode(axis, torque_mode_parameter, "torque_ramp")  # 力矩爬升模式

def calibrate_motor(axis):
    print('正在校准电机……')
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if axis.error != AXIS_ERROR_NONE:
        dump_errors(device)
        exit()
    print("校准电机成功")

    if axis.encoder.config.mode == 0:  # ABI mode
        axis.motor.config.pre_calibrated = True  # ABI 预校准电机
    elif axis.encoder.config.mode == 257:  # SPI mode
        axis.config.startup_motor_calibration = True  # SPI 开机校准电机 TODO SPI下也可以预校准电机？

    time.sleep(1)




def calibrate_encoder(axis):
    print('正在校准编码器……')
    axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if axis.error != AXIS_ERROR_NONE:
        dump_errors(device)
        exit()
    if not axis.encoder.is_ready:
        print("Failed to calibrate encoder! Quitting")
        exit()
    print("校准编码器成功")

    if axis.encoder.config.mode == 0:  # ABI mode
        axis.config.startup_encoder_offset_calibration = True  # ABI 必须开机校准编码器
    elif axis.encoder.config.mode == 257:  # SPI mode
        axis.encoder.config.pre_calibrated = True  # SPI

    time.sleep(1)



def close_loop(axis):
    print('正在测试闭环……')
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    if input("轻扭电机，检查是否进入闭环模式？ (Y/N)").upper() == "Y":
        axis.config.startup_closed_loop_control = True

def set_and_calibrate(device):
    command = input("选择要执行的操作——s:设置参数；c:校准电机;x:通道位0/1\n"
                    "如: s0——设置电机0的参数；c1——校准电机1；sc0——设置电机0参数并校准\n")
    if command[-1] == "0":
        if "s" in command:
            print("设置电机0参数")
            set_motor_param(device.axis0, motor0_parameter)
            set_encoder_param(device.axis0, encoder0_parameter)
            set_control_mode(device.axis0)
            if input("是否取消超速报错？ (Y/N)").upper() == "Y":
                device.axis0.controller.config.enable_overspeed_error=False
        if ("c" or "C") in command:
            if input("准备校准电机,输入 Y 确认 (Y/N)").upper() == "Y":
                calibrate_motor(device.axis0)
            if input("准备校准编码器,输入 Y 确认 (Y/N)").upper() == "Y":
                calibrate_encoder(device.axis0)
            if input("是否需要上电进入闭环模式？ (Y/N)").upper() == "Y":
                close_loop(device.axis0)
    elif command[-1] == "1":
        if ("s" or "S") in command:
            print("设置电机1参数")
            set_motor_param(device.axis1, motor1_parameter)
            set_encoder_param(device.axis1, encoder1_parameter)
            set_control_mode(device.axis1)
            if input("是否取消超速报错？ (Y/N)").upper() == "Y":
                device.axis1.controller.config.enable_overspeed_error=False
        if "c" in command:
            if input("准备校准电机,输入 Y 确认 (Y/N)").upper() == "Y":
                calibrate_motor(device.axis1)
            if input("准备校准编码器,输入 Y 确认 (Y/N)").upper() == "Y":
                calibrate_encoder(device.axis1)
            if input("是否需要上电进入闭环模式？ (Y/N)").upper() == "Y":
                close_loop(device.axis1)
    else:
        print("无效输入")

    if input("是否要继续设置或校准？ (Y/N)").upper() == "Y":
        set_and_calibrate(device)

if __name__ == '__main__':
    device = odrive.find_any()
    print("成功连接驱动器")
    device.__channel__._channel_broken.subscribe(USBerror)

    if input("是否需要设置板子基本参数？ (Y/N)").upper() == "Y":
        set_board_param(device, board_parameter)  # 设置Deng FOC基本参数
    
    
    set_and_calibrate(device)


    if input("是否保存设置？ (Y/N)").upper() == "Y":
        device.save_configuration()

    if input("是否需要重启？ (Y/N)").upper() == "Y":
        device.reboot()






