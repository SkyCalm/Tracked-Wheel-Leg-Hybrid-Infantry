#include "control.h"
#include "tim.h"
#include "judgement.h"
#include "HTmotor.h"
#include "RC.h"
#include "xuc_can.h"

void CONTROL::Init(std::vector<Motor*> motor)
{
	int num1{}, num2{}, num3{}, num4{};
	for (int i = 0; i < motor.size(); i++)
	{
		switch (motor[i]->function)
		{
		case(function_type::chassis):
			chassis_motor[num1++] = motor[i];
			break;
		case(function_type::pantile):
			pantile_motor[num2++] = motor[i];
			break;
		case(function_type::shooter):
			shooter_motor[num3++] = motor[i];
			break;
		case(function_type::supply):
			//supply_motor[num4]->spinning = false;
			supply_motor[num4]->need_curcircle = false;
			supply_motor[num4++] = motor[i];
		default:
			break;
		}
	}
	//pantile_motor[PANTILE::TYPE::PITCH]->setangle = para.initial_pitch;
	pantile.mark_yaw = para.initial_yaw;
}


void CONTROL::Control_Pantile(float ch_yaw, float ch_pitch)
{
	ch_pitch *= (-1.f);
    ch_yaw *= (1.f);// 修改方向

	DMmotor[2].setSpeed = 1.5;

	if (can1_motor[7].mode == POS) {  
		ctrl.pantile.mark_yaw -= pantile.sensitivity_yaw * ch_yaw;
		if (ctrl.pantile.mark_yaw > 8192.0)ctrl.pantile.mark_yaw -= 8192.0;
		if (ctrl.pantile.mark_yaw < 0.0)ctrl.pantile.mark_yaw += 8192.0;
		can1_motor[7].setangle = ctrl.pantile.mark_yaw;

		DMmotor[2].setPos += ch_pitch * (PI / 360.f) * 0.1f;
	}
	
	if (can1_motor[7].mode == POS_IMU && imu_pantile.IsDataValid()) {    
		if (ctrl.mode == CONTROL::AUTOAIM && xuc.GetTargetYaw() != 0)    
		{
			can1_motor[7].setangle = xuc.GetTargetYaw();
			DMmotor[2].setPos -= xuc.GetTargetPitch() * 0.01f;
		}
		else     // ctrl.mode == CONTROL::RC || PC
		{
			can1_motor[7].setangle -= pantile.sensitivity_yaw * ch_yaw;
			if (can1_motor[7].setangle > 180.f) {
				can1_motor[7].setangle -= 360.f;
			}
			else if (can1_motor[7].setangle < -180.f) {
				can1_motor[7].setangle += 360.f;
			}

			DMmotor[2].setPos += ch_pitch * (PI / 360.f) * 0.1f;
		}
	}

	if (DMmotor[2].setPos >= 0.40f) DMmotor[2].setPos = 0.40f;
	if (DMmotor[2].setPos <= -0.55f) DMmotor[2].setPos = -0.55f;
}

void CONTROL::PANTILE::Keep_Pantile(float angleKeep, PANTILE::TYPE type, IMU frameOfReference)
{
	float delta = 0;
	if (type == YAW)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAngleYaw()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;
		if (abs(delta) >= 10.f)
			mark_yaw += pantile_PID[PANTILE::YAW].Delta(delta);
	}
	else if (type == PITCH)
	{
		delta = degreeToMechanical(ctrl.GetDelta(angleKeep - frameOfReference.GetAnglePitch()));
		if (delta <= -4096.f)
			delta += 8192.f;
		else if (delta >= 4096.f)
			delta -= 8192.f;

		if (abs(delta) >= 10.f)
		{
			mark_pitch += pantile_PID[PANTILE::PITCH].Delta(delta);
		}
	}
}

void CONTROL::CHASSIS::Keep_Direction()
{
	double s_x = speedx, s_y = speedy;
	double theat = ctrl.GetDelta(mechanicalToDegree(can1_motor[7].angle[now])
		- mechanicalToDegree(para.initial_yaw)) / 180.f;
	double st = sin(theat*PI);
	double ct = cos(theat*PI);
	speedx = s_x * ct + s_y * st;
	speedy = -s_x * st + s_y * ct;
}

void CONTROL::CHASSIS::Update()
{
	double s_x = speedx;
	double s_y = speedy;

	// 车体相对初始朝向的偏角（弧度）
	double theta = ctrl.GetDelta(
		mechanicalToDegree(can1_motor[7].angle[now]) - mechanicalToDegree(para.initial_yaw)
	) / 180.0 * PI;

	double st = sin(theta);
	double ct = cos(theta);

	// 全局速度 -> 车体速度
	double vx = s_x * ct + s_y * st;
	double vy = -s_x * st + s_y * ct;

	// 按你的底盘映射顺序：0左前 1右前 2右后 3左后
	ctrl.chassis_motor[0]->setspeed = +vx + vy - speedz; // 左前
	ctrl.chassis_motor[1]->setspeed = -vx + vy - speedz; // 右前
	ctrl.chassis_motor[2]->setspeed = -vx - vy - speedz; // 右后
	ctrl.chassis_motor[3]->setspeed = +vx - vy - speedz; // 左后

   // ================= 限位保护 =================
	if (DMmotor[1].setPos > 0.0f)
	{
		DMmotor[1].setPos = 0.0f;
	}
	if (DMmotor[0].setPos < 0.0f)
	{
		DMmotor[0].setPos = 0.0f;
	}
	if (DMmotor[1].setPos < -0.95f)
	{
		DMmotor[1].setPos = -0.95f;
	}
	if (DMmotor[0].setPos > 0.95f)
	{
		DMmotor[0].setPos = 0.95f;
	}
}

void CONTROL::PANTILE::Update()
{

	if (mark_yaw > 8192.0)mark_yaw -= 8192.0;
	if (mark_yaw < 0.0)mark_yaw += 8192.0;

	mark_pitch = std::max(std::min(mark_pitch, para.pitch_max), para.pitch_min);

}

void CONTROL::SHOOTER::Update()
{
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;
	if (ctrl.mode == RC) {
		if (openRub)
		{
			ctrl.shooter_motor[0]->setspeed = 7000;
			ctrl.shooter_motor[1]->setspeed = -7000;
		}
		else
		{
			ctrl.shooter_motor[0]->setspeed = 0;
			ctrl.shooter_motor[1]->setspeed = 0;
		}

		if (supply_bullet && openRub)
		{
			if (auto_shoot)
			{
				//ctrl.supply_motor[0]->setspeed = 2160;
				//ctrl.supply_motor[0]->spinning = true;
			}
			else
			{
				//ctrl.supply_motor[0]->setspeed = 2160;
				//ctrl.supply_motor[0]->spinning = true;
			}
		}
		else
		{
			//ctrl.supply_motor[0]->spinning = false;
			//ctrl.supply_motor[1]->spinning = false;
		}
	}
}

float CONTROL::CHASSIS::Ramp(float setval, float curval, uint32_t RampSlope)
{

	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}

	return curval;
}

float CONTROL::GetDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}

int16_t CONTROL::Setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}

extern uint8_t Power_stsRx[];
