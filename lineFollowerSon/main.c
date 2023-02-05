#include "stm32f10x.h"                  // Device header
#include "Clock_Config.h"
#include "Delay_ms.h"
#include "stdio.h"

void pwm (void);
void pwm2 (void);
void PID_control(void);
void sensor_left (int Cl3,int Cl4);
void sensor_right (int Cr3,int Cr4);
void sharp_turn(void);
int data[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
static int SensorData = 0x00000000;
int position;
  float Kp = 0.015; //0.015
	float Ki = 0.003; //0.003
	float Kd = 10;//10
	int P, I, D;
int errors[10] = {0,0,0,0,0,0,0,0,0,0};
int error_sum = 0;
  const uint8_t maxspeedr = 100;
	const uint8_t maxspeedl = 100;
	const uint8_t basespeedr = 50;
	const uint8_t basespeedl = 50;
	const int ARR_const = 10;
  int lastError = 0;
  int i;
  int k;
  int error;
  int motorspeed;
  int motorspeedl;
  int motorspeedr;
	int varan = 0;
  int actives =0;
int last_end=0;
int last_idle=0;

void initGPIO (void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Enable port A clock
	GPIOA->CRL &= ~(0xffffffff); 
	
  RCC->APB2ENR |=(1<<3); // GPIO B clock enabled
	
	GPIOB->CRL |= ((1<<0)|(1<<1)|(1<<3));// 50 MHz altarnate PP PB0
  GPIOB->CRL &= ~((1<<2));
	
  GPIOB->CRL |=((1<<4)|(1<<5)|(1<<7)); //50 MHz, alternate PP PB1
  GPIOB->CRL &= ~(1<<6);
	
	GPIOB->CRH |= ((1<<8)|(1<<9)|(1<<11));// 50 MHz altarnate PP PB10
  GPIOB->CRH &= ~((1<<10));
	
  GPIOB->CRH |=((1<<12)|(1<<13)|(1<<15)); //50 MHz, alternate PP PB11
  GPIOB->CRH &= ~(1<<14);
}

void motor_control (int motorspeedl, int motorspeedr) 
{
		if(motorspeedl < 0)
	{
		//sensor_left (ARR_const*0,ARR_const*pos_left);
	  sensor_right (ARR_const*0,-1*ARR_const*motorspeedl);
	}
	else
	{
		//sensor_left (ARR_const*pos_left,ARR_const*0);
	  sensor_right (ARR_const*motorspeedl,ARR_const*0);	
	}
	if(motorspeedr < 0)
	{
		sensor_left (ARR_const*0,-1*ARR_const*motorspeedr);
	  //sensor_right (ARR_const*0,ARR_const*pos_left);
	}
	else
	{
		sensor_left (ARR_const*motorspeedr,ARR_const*0);
	  //sensor_right (ARR_const*pos_left,ARR_const*0);	
	}

		//sensor_left (ARR_const*0,ARR_const*motorspeedl);
	  //sensor_right (ARR_const*0,ARR_const*motorspeedr);		
}

void QTR8_config (void)
{
	GPIOA->CRL |= (3<<28) | (3<<24) | (3<<20) | (3<<16) | (3<<12) | (3<<8) | (3<<4) | (3<<0);  // set pin as output
	GPIOA->CRL &= ~(GPIO_CRL_CNF0) & ~(GPIO_CRL_CNF1) & ~(GPIO_CRL_CNF2) & ~(GPIO_CRL_CNF3) & ~(GPIO_CRL_CNF4) & ~(GPIO_CRL_CNF5) & ~(GPIO_CRL_CNF6) & ~(GPIO_CRL_CNF7);
	GPIOA->ODR |= (1 << 7) | (1 << 6)| (1 << 5)| (1 << 4)| (1 << 3)| (1 << 2)| (1 << 1)| (1 << 0);		// set pin as high
	
	delay_us(12);			
	
	GPIOA->CRL &= ~(3<<28) & ~(3<<24) & ~(3<<20) & ~(3<<16) & ~(3<<12) & ~(3<<8) & ~(3<<4) & ~(3<<0); // set pin as input
	GPIOA->CRL |= (2<<30) | (2<<26) | (2<<22) | (2<<18) | (2<<14) | (2<<10) | (2<<6) | (2<<2);	// set pin with pull-up/pull-down  
	delay_ms(6);	
	
}

int QTR8_Read()
{	
	QTR8_config();
    int pos = 0;
		int var = 0;
    int active = 0;
		if(((GPIOA->IDR & (1<<0))==(1<<0)))  //sensor1
			{
				SensorData = 0x00000001;	
				pos+=1000;
				active++;
				last_end=1;
			}
		if(((GPIOA->IDR & (1<<1))==(1<<1)))  //sensor2	
			{
				SensorData = 0x00000010;	
        pos+=2000;
				active++;
			}	
		if(((GPIOA->IDR & (1<<2))==(1<<2)))  //sensor3
			{	
				SensorData = 0x00000100;	
				pos+=3000;
				active++;
			}
		if(((GPIOA->IDR & (1<<3))==(1<<3)))  //sensor4
			{	
				SensorData = 0x00001000;	
				pos+=4000;
				active++;
			}
		if(((GPIOA->IDR & (1<<4))==(1<<4)))  //sensor5
			{	
				SensorData = 0x00010000;	
				pos+=5000;
				active++;
			}
		if(((GPIOA->IDR & (1<<5))==(1<<5)))  //sensor6
			{	
				SensorData = 0x00100000;	
        pos+=6000;
				active++;
			}
		if(((GPIOA->IDR & (1<<6))==(1<<6)))  //sensor7
			{	
				SensorData = 0x01000000;	
				pos+=7000;
				active++;
			}
		if(((GPIOA->IDR & (1<<7))==(1<<7)))  //sensor8
			{	
				SensorData = 0x10000000;	
				pos+=8000;
				active++;
				last_end=0;		
			}
		data[0] = (GPIOA->IDR & (1<<0));
		data[1] = (GPIOA->IDR & (1<<1));
		data[2] = (GPIOA->IDR & (1<<2));
		data[3] = (GPIOA->IDR & (1<<3));
		data[4] = (GPIOA->IDR & (1<<4));
		data[5] = (GPIOA->IDR & (1<<5));
		data[6] = (GPIOA->IDR & (1<<6));
		data[7] = (GPIOA->IDR & (1<<7));	
			position = pos/active;
        actives = active;
				varan = var;			
		if (actives == 0)
		{
			last_idle++;
		}
		else
		{
			last_idle = 0;
		}
			return pos/active;
}

void pwm (void)
{
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; 
 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable timer2
 AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
 TIM2->CCER |= TIM_CCER_CC4E;	// capture/compare 4 enabled
 TIM2->CCER |= TIM_CCER_CC3E;	// capture/compare 3 enabled
 TIM2->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
 TIM2->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE; //lost 4
 TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; //lost 3
 
 //PWM freq = Fclk/PSC/ARR  72MHz/1000
 //PWM Duty = CCR4/ARR

 TIM2->PSC = 72-1; //72 MHz divided by:
 TIM2->ARR = 1000; //1000, to get 72e6/1e3

}

void pwm2 (void)
{
 RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN; 
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable timer3
 
 TIM3->CCER |= TIM_CCER_CC4E;	// capture/compare 4 enabled
 TIM3->CCER |= TIM_CCER_CC3E;	// capture/compare 3 enabled
 TIM3->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
 TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE; //lost 4
 TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE; //lost 3
 
 //PWM freq = Fclk/PSC/ARR  72MHz/1000
 //PWM Duty = CCR4/ARR

 TIM3->PSC = 72-1; //72 MHz divided by:
 TIM3->ARR = 1000; //1000, to get 72e6/1e3
 
}

void sensor_left (int Cl3,int Cl4)
{

TIM2->CCR4= Cl4; //duty cycle ch4 
TIM2->CCR3= Cl3; //duty cycle ch3
TIM2->EGR |= TIM_EGR_UG; //
TIM2->CR1 |= TIM_CR1_CEN; //
}

void sensor_right (int Cr3,int Cr4)
{

TIM3->CCR4= Cr4; //duty cycle ch4 
TIM3->CCR3= Cr3; //duty cycle ch3
TIM3->EGR |= TIM_EGR_UG; //
TIM3->CR1 |= TIM_CR1_CEN; //
}

//void control_forward() {
//	if((data[0]==1 && data[1]==2 && data[2]==4 && data[3]==8) || ((data[0]==1 && data[1]==2 && data[2]==4 && data[3]==8 && data[4]==16))){
//		motor_control(-45, 45);
//	}
//	else if((data[7]==128 && data[6]==64 && data[5]==32 && data[4]==16) || (data[7]==128 && data[6]==64 && data[5]==32 && data[4]==16 && data[3]==8)){
//	motor_control(45, -45);
//}
//	else{
//		motor_control(motorspeedl, motorspeedr);
//	}
//}

void sharp_turn() {	
	if (last_idle < 25)
	{
		if (last_end == 1){
			motorspeedl=15;
			motorspeedr=-25;
			motor_control(motorspeedl, motorspeedr);
		}
		else if(last_end==0)
		{
			motorspeedl=-25;
			motorspeedr=15;
			motor_control(motorspeedl, motorspeedr);
		}
		 else;
	}
	else 
	{
		if (last_end == 1){
			motorspeedl=70;
			motorspeedr=-53;
		}
		else
		{
			motorspeedl=-53;
			motorspeedr=70;
		}
	}
}

//void forward_brake(int pos_left) 
//{
//	
//	  motor_control(pos_left);
//}

//void forward_brake(int motorspeedl) 
//{
//	
//	  motor_control(motorspeedl);
//}

void past_errors (int error) 
{
  for (i = 9; i > 0; i--) 
      errors[i] = errors[i-1];
  errors[0] = error;
}

int errors_sum (int index) 
{
  int sum = 0;
  for (k = 0; k < index; k++) 
  {
      sum += errors[k];
  }
  return sum;
}

void PID_control() 
	{
	position = QTR8_Read();	
  int error = 4500 - position;
	past_errors(error);

  P = error;
  I = errors_sum(5);
  D = error - lastError;

  lastError = error;
	
  motorspeed = P*Kp + I*Ki + D*Kd;
  
  motorspeedl = basespeedl + motorspeed;
  motorspeedr = basespeedr - motorspeed;
  
  if (motorspeedl > maxspeedl)
	{
    motorspeedl = maxspeedl;
	}
  if (motorspeedr > maxspeedr)
	{
    motorspeedr = maxspeedr;
	}
	
//		if (actives>2)
//	{
//		if(varan >= 4000)
//		{
//				motorspeedl=-30;
//				motorspeedr=90;
//			//motor_control(90,-30);
//			delay_ms(6);
//		}
//		if(varan <= -4000){
//			motorspeedl=90;
//				motorspeedr=-30;
//			//motor_control(0,0);
//			delay_ms(6);
//		}
//	}
	
	if (actives==0)
	{
		sharp_turn();
//	motorspeedl=0;
//		motorspeedr=0;
	}

	motor_control(motorspeedl, motorspeedr);
}



int main()
{	
	initClockPLL();
	initGPIO();
	TIM2Config();
	pwm ();
  pwm2();

while(1)
{
		
	 	PID_control();
	

}
return 0;
}