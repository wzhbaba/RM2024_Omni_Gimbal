/**
 *******************************************************************************
 * @file      : infantry.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "infantry.h"

#include "board_comm.h"
#include "bsp_dwt.h"
#include "chassis.h"
#include "gimbal.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
#include "vision.h"
/* Private macro -------------------------------------------------------------*/
static float dt;
static uint32_t dwt_cnt;
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief Initializes the system.
 */
void InfantryGimbalInit()
{
    remote.Init(&huart3);
    referee.Init(&huart6);
    gimbal.MotorInit();
    shoot.MotorInit();
    board_comm.Init(&hcan1, 0x111);
}

/**
 * @brief Initializes the gimbal.
 */
void GimbalInit()
{
    gimbal.PidInit();
    shoot.PidInit();
    chassis.Init();
    vision.Init();
}

/**
 * @brief This function handles the Gimbal task.
 */
void GimbalControllerTask()
{
    gimbal.Control();
    shoot.Control();
    chassis.Control();
}

void MotorTask()
{
    if (remote.GetS2() == 1 || remote.GetS2() == 3) {
        DjiMotorSend(&hcan1, 0x1FF, (int16_t)shoot.trig_output_, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x1FE, 0, gimbal.output_speed_[1], 0, 0);
        DjiMotorSend(&hcan2, 0x1FE, -gimbal.output_speed_[0], 0, 0, 0);
        DjiMotorSend(&hcan2, 0x200, (int16_t)shoot.fric_output_[0], (int16_t)shoot.fric_output_[1], (int16_t)shoot.trig_output_, 0);
    } else {
        DjiMotorSend(&hcan1, 0x1FF, 0, 0, 0, 0);
        DjiMotorSend(&hcan1, 0x1FE, 0, 0, 0, 0);
        DjiMotorSend(&hcan2, 0x1FE, 0, 0, 0, 0);
        DjiMotorSend(&hcan2, 0x200, 0, 0, 0, 0);
    }
}

void VisionCallback()
{
    vision.Ctrl();
}

void ChassisTask()
{
    board_comm.Send();
}