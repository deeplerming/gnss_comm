# SatelliteDataMsg.msg
# 单颗卫星单历元测量数据消息

GnssTimeMsg time              # 测量时间
uint32 sat                    # 卫星ID (使用`gnss_utility.hpp`中的`sat_no`函数定义)

float64 psr                 # 伪距测量 [m]
float64 psr_std             # 伪距标准差 [m]
float64 cp                  # 载波相位测量 [周期]
float64 cp_std              # 载波相位标准差 [周期]
float64 dopp                # 多普勒测量 [Hz]
float64 dopp_std            # 多普勒标准差 [Hz]

float64 wavelength            # 波长

float64[] sat_pos             # 卫星位置 [x, y, z] (m)
float64[] sat_vel             # 卫星速度 [vx, vy, vz] (m/s)
float64 sat_clk               # 卫星钟差
float64 sat_ddt               # 卫星钟漂
float64 elevation             # 高度角
float64 pr_uura               # 伪距用户测距精度
float64 cp_uura               # 载波相位用户测距精度  
float64 dopp_uura             # 多普勒用户测距精度