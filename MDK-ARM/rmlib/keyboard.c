/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file keyboard.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief keyboard message handle
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */
 
#include "keyboard.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "info_get_task.h"
#include "modeswitch_task.h"
#include "ramp.h"
#include "remote_ctrl.h"
#include "cmsis_os.h"
#include "sys_config.h"

/* mouse button long press time */
#define LONG_PRESS_TIME  1000  //ms
/* key acceleration time */
#define KEY_ACC_TIME     1500  //ms

kb_ctrl_t km;

ramp_t fb_ramp = RAMP_GEN_DAFAULT;
ramp_t lr_ramp = RAMP_GEN_DAFAULT;

void key_fsm(kb_state_e *sta, uint8_t *last_release, uint8_t key)
{
  switch (*sta)
  {
    case KEY_RELEASE:
    { 
      if (key)
			{
        *sta = KEY_PRESS_DOWN;
			}
      else if (!key)
			{
				*last_release = 1;
				*sta = KEY_RELEASE;
			}
			
    }break;
    
		case KEY_PRESS_DOWN: 
    {		
			if((*last_release == 1) & key )
			{
				*sta = KEY_PRESS_ONCE;
			}
			else
				*sta = KEY_RELEASE;
    }break;		
    
    case KEY_PRESS_ONCE:
    { 
      *sta = KEY_PRESS_DOWN;
			*last_release = 0;
    }break;
    
    default:
    break;
      
  }
}

static void move_speed_ctrl(uint8_t fast, uint8_t slow)
{
  if (fast)
  {
    km.move = FAST_MODE;
    km.x_spd_limit = CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = CHASSIS_KB_MAX_SPEED_Y;
  }
  else if (slow)
  {
    km.move = SLOW_MODE;
    km.x_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.7f * CHASSIS_KB_MAX_SPEED_Y;
  }
  else
  {
    km.move = NORMAL_MODE;
    km.x_spd_limit = 0.85f * CHASSIS_KB_MAX_SPEED_X;
    km.y_spd_limit = 0.85f * CHASSIS_KB_MAX_SPEED_Y;
  }
}

static void move_direction_ctrl(uint8_t forward, uint8_t back,
                                uint8_t left,    uint8_t right)
{
  //add ramp
  if (forward)
  {
    //km.vx = km.x_spd_limit * ramp_calc(&fb_ramp);
    km.vx = km.x_spd_limit;
  }
  else if (back)
  {
    //km.vx = -km.x_spd_limit * ramp_calc(&fb_ramp);
    km.vx = -km.x_spd_limit;
  }
  else
  {
    km.vx = 0;
    ramp_init(&fb_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }

  if (left)
  {
    //km.vy = km.y_spd_limit * ramp_calc(&lr_ramp);
    km.vy = km.y_spd_limit;
  }
  else if (right)
  {
    //km.vy = -km.y_spd_limit * ramp_calc(&lr_ramp);
    km.vy = -km.y_spd_limit;
  }
  else
  {
    km.vy = 0;
    ramp_init(&lr_ramp, KEY_ACC_TIME/INFO_GET_PERIOD);
  }
  
  if (forward || back || left || right)
    km.twist_ctrl = 0;
}

static void chassis_operation_func(uint8_t twist_chassis)
{
  if (twist_chassis)
    km.twist_ctrl = 1;
}


static void kb_fric_ctrl(uint8_t open_fric,  uint8_t close_fric)
{
  if (open_fric)
    shot.fric_wheel_run = 1;
  
  if (close_fric)
    shot.fric_wheel_run = 0;
}


static void kb_shoot_cmd(uint8_t shoot, uint8_t shoot_switch)
{ // add more mode
  shoot_mode_e mode  = shot.shoot_mode;
	if (shoot_switch ){   //switch shoot mode
		mode++;
		if (mode>3){
			mode = SEMI_ONE;
		}
		switch_shoot_mode(mode);
	}
  //shot.shoot_cmd = rc.mouse.l; //ctrl code is moved to shoot_task
}
static void gimbal_operation_func(int16_t pit_ref_spd, int16_t yaw_ref_spd,
                                  uint8_t shoot_buff,  uint8_t track_armor)
{
  km.pit_v = -pit_ref_spd * 0.1f; //0.01f; changed by H.F. 20180405
  km.yaw_v = -yaw_ref_spd * 0.5f; //0.01f;
	km.vw = yaw_ref_spd * 5; //0.01f; add by H.F.
 
  
  
  if (shoot_buff)
    km.buff_ctrl = 1;
  
  if (track_armor)
    km.track_ctrl = 1;
  else
    km.track_ctrl = 0;

}

static void exit_buff_hook(uint8_t forward, uint8_t back,
                           uint8_t left,    uint8_t right)
{
  if (forward || back || left || right)
    km.buff_ctrl = 0;
}

void keyboard_global_hook(void)
{
  if (km.kb_enable)
  {
    key_fsm(&km.lm.kb_sta,&km.lm.last_release, rc.mouse.l);
    key_fsm(&km.rm.kb_sta,&km.rm.last_release, rc.mouse.r);
		key_fsm(&km.rk.kb_sta,&km.rk.last_release, rc.kb.bit.R); //20180501 hf

  }
}


void keyboard_chassis_hook(void)
{
  if (km.kb_enable)
  {
    move_speed_ctrl(FAST_SPD, SLOW_SPD);
    
    move_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
    
    chassis_operation_func(TWIST_CTRL);
  }
  else
  {
    km.vx = 0;
    km.vy = 0;
    km.twist_ctrl = 0;
  }
}

void keyboard_gimbal_hook(void)
{
  if (km.kb_enable)
  {
    //gimbal_operation_func(rc.mouse.y, rc.mouse.x, BUFF_CTRL, TRACK_CTRL);
    gimbal_operation_func(rc.mouse.y, rc.mouse.x, BUFF_CTRL, TRACK_CTRL);
    
    exit_buff_hook(FORWARD, BACK, LEFT, RIGHT);
  }
  else
  {
    km.pit_v = 0;
    km.yaw_v = 0;
    km.buff_ctrl = 0;
    km.track_ctrl = 0;
  }
}

void keyboard_shoot_hook(void)
{
  //friction wheel control
  kb_fric_ctrl(KB_OPEN_FRIC_WHEEL, KB_CLOSE_FIRC_WHEEL);
  //single or continuous trigger bullet control
  kb_shoot_cmd( KB_SHOOT, KB_SHOOT_SWITCH); //hf 20180428

}
