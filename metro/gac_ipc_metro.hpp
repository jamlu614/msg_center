/**********************************************************
 * 地铁消息接口
 * creat by cxc, 20250819
 **********************************************************/

#pragma once

#include <stdint.h>

struct GacIpcMetro
{
    int32_t algorithm_type = 0;
};

struct GacIMRobotStatus
{
    int32_t device_status = 0;
    uint8_t charge_level = 0;
    bool charging_status = false;
    bool estop = false;
};

int gac_ipc_metro_init(void);
void gac_ipc_metro_exit(void);
void gac_ipc_metro_write(const GacIpcMetro &data);
bool gac_ipc_metro_read(GacIpcMetro &data);

void gac_ipc_metro_robot_status_write(const GacIMRobotStatus &data);
bool gac_ipc_metro_robot_status_read(GacIMRobotStatus &data);