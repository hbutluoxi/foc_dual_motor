
import time
from odrive.enums import *
from odrive.utils import *
import odrive

axis_num = 0 #选择设置电机0还是电机1
pole_pairs = 7 # 电机的极对数
kv_value = 980 # 电机的KV值
current_lim = 30 #电机电流限制
vel_lim = 50 # 速度限制
vel_gain = 0.01 #速度P
vel_integrator_gain = 0.05 #速度I





def USBerror():
    print("Device disconnected")


if __name__ == '__main__':
    # 连接Deng FOC 注意退出odrivetool，否则无法连接
    odrv0 = odrive.find_any()
    # odrv0.__channel__._channel_broken.subscribe(USBerror)
    # print_configs(odrv0)
    if axis_num == 0: 
        device = odrv0.axis0
    elif axis_num == 1:
        device = odrv0.axis1
    else:
        print("设置的参数有问题，请检查")
        exit()
    print("当前准备设置M",axis_num,"为无感速度模式")
    print("当前设置的电机极对数为",pole_pairs,"KV值为：", kv_value)
    print("电流限制为：",current_lim,"速度限制为",vel_lim)
    print("电流限制为：", vel_gain, "速度限制为", vel_gain)
    print("电流限制为：", vel_integrator_gain, "速度限制为", vel_integrator_gain)
    print("如果要修改上面的数据，请停止运行本脚本进行修改")
    print("如果想设置其他参数，请参考群内文档")
    set_flag = input("是否进行相关参数设置(Y/N)")

    if set_flag.upper() == "Y":
        device.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (pole_pairs * kv_value)
        device.motor.config.current_lim = current_lim
        device.motor.config.direction = 1 
        device.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL  
        device.controller.config.vel_limit = vel_lim
        device.controller.config.vel_gain = vel_gain
        device.controller.config.vel_integrator_gain = vel_integrator_gain
        print("参数设置完毕")
        if input("校准电机？(Y/N)").upper() == "Y":
            device.requested_state = AXIS_STATE_MOTOR_CALIBRATION      
            while device.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if device.error != AXIS_ERROR_NONE:
                dump_errors(odrv0)
                exit()
            print("校准电机成功")
            device.motor.config.pre_calibrated = True
            if input("设置成无感？电机会转动一下(Y/N)").upper() == "Y":
                device.requested_state = AXIS_STATE_SENSORLESS_CONTROL
                while device.current_state != 5:
                    time.sleep(0.1)
                if device.error != AXIS_ERROR_NONE:
                    dump_errors(odrv0)
                    exit()
                if input("设置无感成功，是否设为上电默认无感速度模式？(Y/N)").upper() == "Y":
                    device.config.startup_sensorless_control = True
                print("正在保存设置，保存后需要重启才能控制电机")
                odrv0.save_configuration()
                print("电机控制指令：odrv0.axis0.controller.input_vel = (速度设定值，单位为turns/s), 控制M1就将axis0改成axis1")
    print("Done!")
    if input("是否重启(Y/N)").upper() == "Y":
        odrv0.reboot()