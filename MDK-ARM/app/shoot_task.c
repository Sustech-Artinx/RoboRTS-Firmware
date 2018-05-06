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
/** @file shoot_task.c
 *  @version 1.1
 *  @date June 2017
 *
 *  @brief shoot bullet task
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "shoot_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_ctrl.h"
#include "bsp_io.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_io.h"
#include "keyboard.h"
#include "pid.h"
#include "sys_config.h"
#include "cmsis_os.h"
#include "string.h"

/* setting */
#define RELOAD_TIMEOUT 2500
#define FRIC_WHEEL_OFF 1000
#define ABS(num) ((num)>0?(num):-(num))

/* stack usage monitor */
UBaseType_t shoot_stack_surplus;

/* shot task global parameter */
shoot_t   shot;
gun_t   gun_17;
gun_t   gun_42;
bullet_supply_t g17_bupply;
bullet_supply_t g42_bupply;

void shot_task(void const *argu)
{
  osEvent event;
  
  while (1)
  {
    event = osSignalWait(SHOT_TASK_EXE_SIGNAL, osWaitForever);
    
    if (event.status == osEventSignal)
    {
      if (event.value.signals & SHOT_TASK_EXE_SIGNAL)
      {	
        fric_wheel_ctrl();
				if(gun_17.turn_on){
					gun_17_handle();
				}
				if(gun_42.turn_on){
					gun_42_handle();
				}
      }
    }
    
    shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
  }
}

static void gun_17_handle(void){
	static uint8_t last_cmd = 0;
	if(gun_17.state == WAITING_CMD){
		if(gun_17.cmd == 1 && last_cmd == 0){
			/* ignore shoot command when friction wheel is disable */
			if (shot.fric_wheel_run){
				g17_bupply.spd_ref = g17_bupply.feed_bullet_spd;
				gun_17.shooted_count = 0;
				gun_17.timestamp = HAL_GetTick();
				gun_17.state = SHOOTING;
			}
		} else if(gun_17.cmd == 2 && last_cmd == 0){
			g17_bupply.spd_ref = g17_bupply.feed_bullet_spd;
			gun_17.timestamp = HAL_GetTick();
			gun_17.state = RELOADING;
		}
	} else if(gun_17.state == SHOOTING){
		if(g17_shoot_bullet_handle()){
			;
		} else {
			//no need to update timestamp 
			//because shoot_bullet_handle() has made it
			g17_bupply.spd_ref = g17_bupply.feed_bullet_spd;
			gun_17.state = RELOADING;
		}
	} else if(gun_17.state == RELOADING){
		if(g17_bupply.bbkey_state == BBKEY_ON){
			//reload complete
			g17_bupply.spd_ref = 0;
			gun_17.state = WAITING_CMD;
		} else if(HAL_GetTick()-gun_17.timestamp > RELOAD_TIMEOUT){
			//timeout
			g17_bupply.spd_ref = 0;
			gun_17.state = WAITING_CMD;
		}
	} else if(gun_17.state == STUCK_HANDLING){
		if(g17_stuck_handle()){
			;
		} else {
			g17_bupply.spd_ref = g17_bupply.feed_bullet_spd;
			gun_17.timestamp = HAL_GetTick();
			gun_17.state = RELOADING;
		}
	}
	if(g17_stuck_detect()){
		g17_bupply.spd_ref = -g17_bupply.spd_ref;
		gun_17.timestamp = HAL_GetTick();
		gun_17.state = STUCK_HANDLING;
	}
	pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, g17_bupply.spd_ref);
	//g17_bupply.bbkey_state_last = g17_bupply.bbkey_state;
	g17_bupply.bbkey_state_last = BBKEY_ON;
	last_cmd = gun_17.cmd;
}

void gun_42_handle(void){
	static uint8_t last_cmd = 0;
	if(gun_42.state == WAITING_CMD){
		if(gun_42.cmd == 1 && last_cmd == 0){
			/* ignore shoot command when friction wheel is disable */
			if (shot.fric_wheel_run){
				g42_bupply_pusher_forward();
				gun_42.shooted_count = 0;
				gun_42.timestamp = HAL_GetTick();
				gun_42.state = SHOOTING;
			}
		} else if(gun_42.cmd == 2 && last_cmd == 0){
			;
		}
	} else if(gun_42.state == SHOOTING){
		if(g42_shoot_bullet_handle()){
			;
		} else {
			gun_42.state = WAITING_CMD;
		}
	} else if(gun_42.state == RELOADING){
		gun_42.state = WAITING_CMD;
	} else if(gun_42.state == STUCK_HANDLING){
		gun_42.state = WAITING_CMD;
	}
	last_cmd = gun_42.cmd;
}

static void g17_set_mode(gun_mode_e mode){
	gun_17.mode = mode;
  switch(gun_17.mode){
    case SEMI_ONE:
      gun_17.shoot_spd = 15;
      gun_17.shoot_num = 1;
      gun_17.fric_wheel_spd = 2000;
      break;
    case SEMI_THREE:
      gun_17.shoot_spd = 15;
      gun_17.shoot_num = 3;
      gun_17.fric_wheel_spd = 1800;
      break;
    case AUTO:
      gun_17.shoot_spd = 15;
      gun_17.shoot_num = 0;
      gun_17.fric_wheel_spd = 1500;
      break;
  }
}

static void g42_set_mode(gun_mode_e mode){
	gun_42.mode = mode;
  switch(gun_42.mode){
    case SEMI_ONE:
      gun_42.shoot_spd = 15;
      gun_42.shoot_num = 1;
      gun_42.fric_wheel_spd = 2000;
      break;
    case SEMI_THREE:
      gun_42.shoot_spd = 15;
      gun_42.shoot_num = 3;
      gun_42.fric_wheel_spd = 1800;
      break;
    case AUTO:
      gun_42.shoot_spd = 15;
      gun_42.shoot_num = 0;
      gun_42.fric_wheel_spd = 1500;
      break;
  }
}

void switch_shoot_mode(shoot_mode_e mode){
  shot.shoot_mode = mode;
  switch(shot.shoot_mode){
    case G17_SEMI_ONE:
			gun_17.turn_on = 1;
      g17_set_mode(SEMI_ONE);
			gun_42.turn_on = 0;
      break;
    case G17_SEMI_THREE:
			gun_17.turn_on = 1;
      g17_set_mode(SEMI_THREE);
			gun_42.turn_on = 0;
      break;
    case G17_AUTO:
			gun_17.turn_on = 1;
      g17_set_mode(AUTO);
			gun_42.turn_on = 0;
      break;
		case G42_SEMI_ONE:
			gun_17.turn_on = 0;
			gun_42.turn_on = 1;
			g42_set_mode(SEMI_ONE);
			break;
  }
}

static uint8_t g17_stuck_detect(void){
  static uint32_t last_uptime;
  if(ABS(moto_trigger.speed_rpm) < ABS(g17_bupply.spd_ref)/3){
    if(HAL_GetTick()-last_uptime > 500){
      //stuck confirm
      return 1;
    }
  } else {
    last_uptime = HAL_GetTick();
  }
  return 0;
}

static uint8_t g17_stuck_handle(void){
  if(HAL_GetTick()-gun_17.timestamp > 200000/ABS(g17_bupply.spd_ref)){
    return 0;
  } else {
    return 1;
  }
}

static void fric_wheel_ctrl(void){
  if (shot.fric_wheel_run)
  {
		if(gun_17.turn_on){
			turn_on_friction_wheel_g17(gun_17.fric_wheel_spd);
		} else {
			turn_off_friction_wheel_g17();
		}
		if(gun_42.turn_on){
			turn_on_friction_wheel_g42(gun_17.fric_wheel_spd);
		} else {
			turn_off_friction_wheel_g42();
		}
    turn_on_laser();
  }
  else
  {
    turn_off_friction_wheel_g17();
		turn_off_friction_wheel_g42();
    turn_off_laser();
  }
}

static uint8_t g17_shoot_bullet_handle(void){
  static uint8_t holding = 0;
  uint32_t time_now = HAL_GetTick();
  if(time_now-gun_17.timestamp > RELOAD_TIMEOUT){
    holding = 0;  //reset state
    return 0;
  } else if(gun_17.shooted_count == gun_17.shoot_num){
		if(gun_17.state != AUTO || gun_17.cmd == 0){
			holding = 0;  //reset state
			return 0;
		}
  } else {
    if(holding){
      if(time_now-gun_17.timestamp >= 1000/gun_17.shoot_spd){
        holding = 0;  //shoot
        g17_bupply.spd_ref = g17_bupply.feed_bullet_spd;
      }
    } else {
      if(g17_bupply.bbkey_state_last == BBKEY_ON && g17_bupply.bbkey_state == BBKEY_OFF){
        gun_17.shooted_count++;
        gun_17.timestamp = HAL_GetTick();
      } else if(g17_bupply.bbkey_state_last == BBKEY_OFF && g17_bupply.bbkey_state == BBKEY_ON){
        holding = 1;  //waiting next shoot
        g17_bupply.spd_ref = 0;
      }
    }
    return 1;
  }
}

static uint8_t g42_shoot_bullet_handle(void){
	static uint8_t holding = 0;
  uint32_t time_now = HAL_GetTick();
  if(time_now-gun_42.timestamp > RELOAD_TIMEOUT){
    holding = 0;  //reset state
    return 0;
  } else if(gun_42.shooted_count == gun_42.shoot_num){
		if(gun_42.state != AUTO || gun_42.cmd == 0){
			holding = 0;  //reset state
			return 0;
		}
  } else {
    if(holding){
      if(time_now-gun_42.timestamp >= 1000/gun_42.shoot_spd){
        holding = 0;  //shoot
        g42_bupply_pusher_forward();
				gun_42.timestamp = HAL_GetTick();
      }
    } else {
			if(time_now-gun_42.timestamp >= 25){
				g42_bupply_pusher_back();
				gun_42.shooted_count++;
				gun_42.timestamp = HAL_GetTick();
				holding = 1;
			}
    }
    return 1;
  }
}

void shot_param_init(void){
  memset(&shot, 0, sizeof(shoot_t));
  
  shot.ctrl_mode      = SHOT_DISABLE;
  gun_17.state    = WAITING_CMD;
	gun_42.state    = WAITING_CMD;
  switch_shoot_mode(G17_SEMI_ONE);
  //shot.remain_bullets = 0;
  
  memset(&g17_bupply, 0, sizeof(bullet_supply_t));
  
  g17_bupply.feed_bullet_spd = TRI_MOTO_POSITIVE_DIR*TRIGGER_MOTOR_SPEED; //2000; //changed by H.F.
}

