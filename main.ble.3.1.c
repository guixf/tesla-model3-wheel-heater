#include "stc8g.h"
#include "intrins.h"
#include "stdio.h"

#define FOSC 11059200UL
#define BRT (65536 - FOSC / 115200 / 4)

//#define ADC_low 870;
//#define ADC_high 800;



typedef     unsigned char   u8;
typedef     unsigned int    u16;
typedef     unsigned long   u32;

//#define     Tmp_Length          14      //读写EEPROM缓冲长度

#define     UART1_BUF_LENGTH    (6)  //串口缓冲长度


#define PWM2_NORMAL() PCA_PWM2 &=~3//P55输出pwm

#define PWM2_OUT_0() PCA_PWM2 |=3//P55关闭pwm输出

#define PWM2_OUT_1() PCA_PWM2 &=~3,CCAP2H = 0 //P55 全开输出


/*EEPROM部分数据地址定义*/
//预设值部分
#define ADC_autoopen_low_Address 0x0000
#define ADC_autoopen_high_Address 0x0001
#define ADC_set_low_Address 0x0002
#define ADC_set_high_Address 0x0003
#define ADC_low_pwm_Address 0x0004
#define ADC_high_pwm_Address 0x0005

//上次保存的状态
#define Heater_status_Address 0x0200

//控制功能
#define Heater_unable_Address 0x0400//方向盘加热按键控制功能激活状态保存地址，00为未激活，其他为激活
#define Button_unable_Address 0x0401//Save_Button_activated);//00为关闭，01为激活使用保存的方向盘加热状态，02为激活防冻手功能

u16 ADC_low = 0;
u16 ADC_high = 0;
u8  RX1_TimeOut;
u8  TX1_Cnt;    //发送计数
u8  RX1_Cnt;    //接收计数
bit B_TX1_Busy; //发送忙标志

u8  BCC_State;    //异或结果

u8   RX1_Buffer[UART1_BUF_LENGTH]; //接收缓冲
u8   tmp[5];       				 //EEPROM操作缓冲
u8   tmp_value[5];        //EEPROM操作缓冲

bit busy;





void IapIdle()
{
    IAP_CONTR = 0;                              //关闭IAP功能
    IAP_CMD = 0;                                //清除命令寄存器
    IAP_TRIG = 0;                               //清除触发寄存器
    IAP_ADDRH = 0x80;                           //将地址设置到非IAP区域
    IAP_ADDRL = 0;
}

char IapRead(int addr)
{
    char dat;

    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 1;                                //设置IAP读命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    dat = IAP_DATA;                             //读IAP数据
    IapIdle();                                  //关闭IAP功能

    return dat;
}

void IapProgram(int addr, char dat)
{
    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 2;                                //设置IAP写命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_DATA = dat;                             //写IAP数据
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();
    IapIdle();                                  //关闭IAP功能
}

void IapErase(int addr)
{
    IAP_CONTR = 0x80;                           //使能IAP
    IAP_TPS = 12;                               //设置等待参数12MHz
    IAP_CMD = 3;                                //设置IAP擦除命令
    IAP_ADDRL = addr;                           //设置IAP低地址
    IAP_ADDRH = addr >> 8;                      //设置IAP高地址
    IAP_TRIG = 0x5a;                            //写触发命令(0x5a)
    IAP_TRIG = 0xa5;                            //写触发命令(0xa5)
    _nop_();                                    //
    IapIdle();                                  //关闭IAP功能
}

/*unsigned int ReadIapADC(int addr)
{
	u16 Itmp = 0 ;
			Itmp = IapRead(addr);
			addr++;
			Itmp += IapRead(addr)<<8;
	return  Itmp;
}
*/
unsigned int ReadIapADC(int addr)
{
	u16 Itmp = 0 ;
			Itmp = IapRead(addr);
			//addr++;
			Itmp += 0x03<<8;
	return  Itmp;
}
unsigned int ReadIapPWM(int addr)
{
	u16 Itmp = 0 ;
			Itmp = IapRead(addr);
			//addr++;
			Itmp += 0x00<<8;
	return  Itmp;
}


void ReadIapDefault(char tmp[])
{
	
										tmp[0]=IapRead(ADC_autoopen_low_Address);//获取预设低温值
										tmp[1]=IapRead(ADC_autoopen_high_Address);//获取高温预设值
										tmp[2]=IapRead(ADC_set_low_Address);//获取预设低温值
										tmp[3]=IapRead(ADC_set_high_Address);//获取高温预设值
										tmp[4]=IapRead(ADC_low_pwm_Address);//获取预设低温值
										tmp[5]=IapRead(ADC_high_pwm_Address);//获取高温预设值
	
}

void WriteIapDefault(char *s)
{
										IapErase(ADC_autoopen_low_Address);//删除原有预设数据
	
										IapProgram(ADC_autoopen_low_Address,s[0]);//写入预设低温值
										IapProgram(ADC_autoopen_high_Address, s[1]);//写入高温预设值
										IapProgram(ADC_set_low_Address, s[2]);//获取预设低温值
										IapProgram(ADC_set_high_Address,s[3]);//获取高温预设值
										IapProgram(ADC_low_pwm_Address,s[4]);//获取预设低温值
										IapProgram(ADC_high_pwm_Address,s[5]);//获取高温预设值
	
}

void	Eeprom_system_init(void)
{
									tmp_value[0] = 0x70;//写入自动保暖功能时低于温度自动开启的ADC值，默认+0x0300	
									tmp_value[1] = 0x5c;//写入自动保暖功能时保暖温度的ADC值，默认+0x0300
									tmp_value[2] = 0x52;//写入	850默认值
									tmp_value[3] = 0x34;//写入	820默认值
									tmp_value[4] = 0xe6;//写入PWM值90%
									tmp_value[5] = 0xdd;//写入PWM值80%
								
								  WriteIapDefault(tmp_value);
									
}

void UartIsr() interrupt 4 
{
 if (TI)
 {
  TI = 0;
  busy = 0;
 }
 if (RI)
 {
        RI = 0;
        if(RX1_Cnt >= UART1_BUF_LENGTH) RX1_Cnt = 0;
        RX1_Buffer[RX1_Cnt] = SBUF;
        RX1_Cnt++;
        RX1_TimeOut = 5;
  }
}


//在数据传输或者数据下载过程中;通常要保证数据的可靠性和安全性;所以
//发送方和接收方要约定共同的协议;而这个协议中常常会出现校验和的运算。
 unsigned char calc_nmea_checksum(const char *setence)
{
     unsigned char checksum = 0;
			u8 n;
//	     n=sizeof(*setence);
 //          while(*setence)
		for(n=0;n<5;n++)
								{
                checksum ^=(unsigned char)*setence++;
                }

     return  checksum;
}


void UartInit(void)		//115200bps@11.0592MHz
{
	SCON = 0x50;		//8位数据,可变波特率
	AUXR &= 0xBF;		//定时器时钟12T模式
	AUXR &= 0xFE;		//串口1选择定时器1为波特率发生器
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0xFE;		//设置定时初始值
	TH1 = 0xFF;		//设置定时初始值
	ET1 = 0;		//禁止定时器%d中断
	TR1 = 1;		//定时器1开始计时
	
	  B_TX1_Busy = 0;
    TX1_Cnt = 0;
    RX1_Cnt = 0;
    RX1_TimeOut = 0;
}

void UartSend(char dat)
{
 while (busy);
 busy = 1;
 SBUF = dat;
}
void SendString(   char *s)
{
 
    while (*s)                   //检测字符串结束标志
    {
        UartSend(*s++);         //发送当前字符
    }
		
}

void hal_init_ADC()
{
 

P3M0=P3M0&~(0x04);//P32高阻 P33双向
P3M1=P3M1|0x04;//P32高阻


     
	 //ADCCFG寄存器
	 //    B7__B6__ B5    __B4__B3__B2__B1__B0__ 
	 //	  - __- __RESFMT __- __   SPEED[3:0] __
	 //    RESFMT：=1 右对齐	:=0 左对齐
	 //    SPEED adc时钟控制	 ADC时钟:SYSclk/2/16/SPEED 
	 //          如： SPEED=1111，SYSclk=24MHz。 则Fadc=24M/2/16/16=46.875kHz, tadc=21ms
    ADCCFG = 0x22; //f_设置ADC时钟:Fadc=11M/2/16/2=171kHz,tadc=1.3us
                   //2_设置为右对齐
									 //adc
    //ADC_CONTR寄存器
	//    B7__________B6__________B5 ________B4__B3__B2__B1__B0__ 
	//	  ADC_POWER___ADC_STRAT___ADC_FLAG __- __  ADC_CHS[3:0] __
	//    ADC_STRAT：=1 开始转换，完成自动清零	:=0 无作用
	//    ADC_FLAG：转换完成硬件置1，必须软件清零。 
	//	  ADC_CHS[3:0]选择通道0-14是(P1.0—P1.7,P0.0-P0.6),15是内部refv的电压1.344
  //              选择通道1001 是P01
       
    //ADC_CONTR = 0x8A;        //打开ADC模块电源,选择通道1010 就是P02
}

void api_read_adc(unsigned int *Adc_result,unsigned char adc_channel)
{
     
    ADC_CONTR = (0xC0|adc_channel);     //启动AD转换
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //查询ADC完成标志
    ADC_CONTR &= ~0x20;                      //清完成标志
    *Adc_result = (ADC_RES<<8|ADC_RESL);    //读取ADC结果。右对齐，高位自动填充为0


    ADC_CONTR = (0xC0|adc_channel);   //启动AD转换
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //查询ADC完成标志
    ADC_CONTR &= ~0x20;                     //清完成标志
	  *Adc_result = (ADC_RES<<8|ADC_RESL);     //读取ADC结果。右对齐，高位自动填充为0
}


u16  adc_value;
u16  ADC_set=0;
u8 string[30];
u16  pwm=0;
//unsigned int  pwm_count=0;
u16 	key=0; 
u16  time_50ms_ok=0;
//unsigned int  key_time=0;
//unsigned int  start_state=0;

 /*=============
低层按键（I/0）扫描函数，即低层按键设备驱动，只返回无键、短按和长按。
===============*/

#define key_input    P33    // 按键输入口

#define N_key    0             //无键
#define S_key    1             //单键
#define D_key    2             //双键
#define L_key    3             //长键

#define key_state_0 0
#define key_state_1 1
#define key_state_2 2
#define key_state_3 3

 
unsigned char key_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;

    key_press = key_input;                    // 读按键I/O电平

			P54 = key_press;//蜂鸣器输出
	
    switch (key_state)
    {
      case key_state_0:                              // 按键初始态
        if (key_press) key_state = key_state_1;      // 键被按下，状态转换到按键消抖和确认状态;上升沿状态激活
        break;
      
      case key_state_1:                      // 按键消抖与确认态
        if (key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // 按键仍然处于按下，消抖完成，状态转换到按下键时间的计时状态，但返回的还是无键事件
        }
        else
             key_state = key_state_0;   // 按键已抬起，转换到按键初始态。此处完成和实现软件消抖，其实按键的按下和释放都在此消抖的。
        break;
      
      case key_state_2:
        if(!key_press)
        {
             key_return = S_key;        // 此时按键释放，说明是产生一次短操作，回送S_key
             key_state = key_state_0;   // 转换到按键初始态
        }
        else if (++key_time >= 10)     // 继续按下，计时加10ms（10ms为本函数循环执行间隔）
        {
             key_return = L_key;        // 按下时间>1000ms，此按键为长按操作，返回长键事件
             key_state = key_state_3;   // 转换到等待按键释放状态
        }
        break;

      case key_state_3:                 // 等待按键释放状态，此状态只返回无按键事件
        if (!key_press) key_state = key_state_0; //按键已释放，转换到按键初始态
        break;
    }
		
//      	P54 = key_press;                       //蜂鸣器 
    return key_return;
}

/*=============
中间层按键处理函数，调用低层函数一次，处理双击事件的判断，返回上层正确的无键、单键、双键、长键4个按键事件。
本函数由上层循环调用，间隔10ms
===============*/

unsigned char key_read(void)
{
    static unsigned char key_m = key_state_0, key_time_1 = 0;
    unsigned char key_return = N_key,key_temp;
     
    key_temp = key_driver();


	
    switch(key_m)
    {
        case key_state_0:
            if (key_temp == S_key )
            {
                 key_time_1 = 0;               // 第1次单击，不返回，到下个状态判断后面是否出现双击
                 key_m = key_state_1;
            }
            else
								{
									key_return = key_temp;        // 对于无键、长键，返回原事件
								}
            break;

        case key_state_1:
            if (key_temp == S_key)             // 又一次单击（间隔肯定<500ms）
            {
                 key_return = D_key;           // 返回双击键事件，回初始状态
                 key_m = key_state_0;
            }
            else                                
            {                                  // 这里500ms内肯定读到的都是无键事件，因为长键>1000ms，在1s前低层返回的都是无键
                 if(++key_time_1 >= 20)
                 {
                      key_return = S_key;      // 500ms内没有再次出现单键事件，返回上一次的单键事件
                      key_m = key_state_0;     // 返回初始状态
                 }

             break;
        case key_state_3:
            if (key_temp == L_key )
            {
                 key_return = L_key;           // 返回长击键事件，回初始状态
                 key_m = key_state_0;
            }
            break;
		}
    return key_return;
	}     
}

void Timer0Init(void)		//50000微秒@11.0592MHz
{
	AUXR &= 0x7F;		//定时器时钟12T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x00;		//设置定时初始值
	TH0 = 0x4C;		//设置定时初始值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
}

/***************************************************************************************/
/*PWM初始化*/
void PWM_Init(void)
{
	P_SW1 &= ~0x10;//设置P55为输出口
	P_SW1 |= 0x20;
	
  CCON = 0x00;
  CMOD = (CMOD &~0xe0)|0x08;//PCA时钟为系统时钟
  CL = 0x00;  //PCA计数器初始值低8位
  CH = 0x00;  //PCA计数器初始值高8位
  CCAPM2 = 0x42; //PCA模块2为PWM工作模式
	
  PCA_PWM2 = (PCA_PWM2 &~0xc0)|0x00;//PCA模块0输出8位PWM
	
  CCAP2L = 0x00;
  CCAP2H = 0x00;//PCA模块用在PWM 模式中时,用来控制输出的占空比。
  CR = 1; //启动PCA计时器
}
/*调节PWM占空比*/
void Pwm_outset(u16 grad)
{
if(grad == 0) PWM2_OUT_0();
	else CCAP2H = (u8)(256 - grad), PWM2_NORMAL();

}

/////////////////////////////

void main()
{
			u8 debug =0;//0关闭调试模式，简单输出，1打开调试模式，不间断输出
			u8 beep =0;//0关闭蜂鸣器，1打开蜂鸣器
	    u8  i;
	    u8  Wheel_Heater_activated = 0;//方向盘加热功能是否激活标志
			u8 Save_Button_activated;
			u16 ADC_LOW_SET_PWM,ADC_HIGH_SET_PWM;//eeprom中保存的高低温度PWM设置值
	
if(beep){	
			P5M0 = 0x30;                                //设置P5.5和P5.4为推挽输出模式
			P5M1 = 0x00;
}else{
			P5M0 = 0x20;                                //只设置P5.5为推挽输出模式
			P5M1 = 0x00;
}	
			Timer0Init();	 //	启动定时器0
			UartInit();    //启动uart
    	PWM_Init();     //启动PWM 	

    	hal_init_ADC();    //初始化ADC


			P54 = 0;  
	    P33=0;



      ES = 1;	
			ET0 = 1;    //Timer0 interrupt enable
			TR0 = 1;    //Tiner0 run

			EA = 1;     //打开总中断

//默认值准备写入
				if(IapRead(ADC_autoopen_low_Address)==0xff)	Eeprom_system_init();
//写入结束

					Save_Button_activated = IapRead(Button_unable_Address);
					if(Save_Button_activated ==0x02){
										Wheel_Heater_activated = 1;//使用自动防冻手功能时，方向盘按键必须打开 
										ADC_low = ReadIapADC(ADC_autoopen_low_Address);
										ADC_high = ReadIapADC(ADC_autoopen_high_Address);
					}else{
										Wheel_Heater_activated	= IapRead(Heater_unable_Address);//读取方向盘加热按键控制功能激活状态，00为未激活，其他为激活
										ADC_low = ReadIapADC(ADC_set_low_Address);					
										ADC_high = ReadIapADC(ADC_set_high_Address);
					}
					if (ADC_low > 1000||ADC_low < 800) ADC_low=850;
					if (ADC_high <800 || ADC_high >1000) ADC_high=820;
					
						ADC_LOW_SET_PWM = ReadIapPWM(ADC_low_pwm_Address);
					if(ADC_LOW_SET_PWM==0xff||ADC_LOW_SET_PWM==0) {
										ADC_LOW_SET_PWM = 0x00e6;		//默认0.9比例
					}
					
						ADC_HIGH_SET_PWM = ReadIapPWM(ADC_high_pwm_Address);
					if(ADC_HIGH_SET_PWM==0xff||ADC_HIGH_SET_PWM==0) {
												ADC_HIGH_SET_PWM = 0x00dd;		//默认0.8比例
					}										

					
									sprintf(string,"Adc_set_Low is %d!\r\nAdc_set_High is %d!\r\n",ADC_low,ADC_high);
								  SendString(string);										
									sprintf(string,"Adc_auto_Low is %d!\r\nAdc_auto_High is %d!\r\n",ReadIapADC(ADC_autoopen_low_Address),ReadIapADC(ADC_autoopen_high_Address));
								  SendString(string);		
									sprintf(string,"LOW_PWM & HIGH_PWM is 0x%04X.\r\n",ADC_LOW_SET_PWM,ADC_HIGH_SET_PWM);
								  SendString(string);


  				if(!Wheel_Heater_activated) 
						{
												SendString("Wheel Heater button be Inactivated!\r\n");
						}else{
												SendString("Wheel Heater button be Activated!\r\n");	
						}

							
					switch(Save_Button_activated){
							case 0x00:
												SendString("Wheel Heater Save Status be Inactivated!\r\n");	
												break;
							case 0x01:
												SendString("Wheel Heater Save Status be Activated!\r\n");	
												break;
							case 0x02:
												SendString("Wheel Heater autoopen be Activated!\r\n");	
												break;
							default:
												SendString("Heater save Is Wrong!send 15 command!\r\n");	
												break;
						}
										ADC_high = ReadIapADC(ADC_set_high_Address);										
	
						
			if(Save_Button_activated==0x01){			//获取是否使用上次的保存状态					
					switch(IapRead(Heater_status_Address))  //获取上次保存的加热状态
					{
						case 0x00:
								ADC_set = 0;
								break;
						case 0x12:				
								ADC_set = ADC_low;
								break;
						case 0x24:					
								ADC_set = ADC_high;
								break;
						case 0xff:
								ADC_set = -1;
								break;
					}
			}

					api_read_adc(&adc_value,2 );			//读取ntc10k阻值，获取温度信息		
			
			if(Save_Button_activated==0x02 && adc_value>ReadIapADC(ADC_autoopen_low_Address)){			//如果为自动模式，则判断是否低于预设温度					
							ADC_set = ReadIapADC(ADC_autoopen_high_Address);
//									sprintf(string,"ADC_autoopen_SET is write %d!\r\nADC_autoclose_SET is write %d!\r\n",ReadIapADC(0x0200),ReadIapADC(0x0201));
//								  SendString(string);	
			}
					sprintf(string,"ADC_value is  %d!\r\n",adc_value);			  
    			SendString(string);		
			

		  key=0;	
	   WDT_CONTR = 0x27; //启动看门狗，8s重启
			
		while (1)
		{
   
		//检查uart1收到的数据
        if(RX1_TimeOut > 0)     //超时计数
        {
//					for(i=0; i<RX1_Cnt; i++)    UartSend(RX1_Buffer[i]);//原样传回，用于测试

				 if(--RX1_TimeOut == 0 && RX1_Cnt == 6)
					 {
							for(i=0; i<5; i++){
								tmp[i]=RX1_Buffer[i];//取最后一个以外的数据，用于校验
//								UartSend(tmp[i]);//原样传回，用于测试
							}
							BCC_State = RX1_Buffer[5];

            if((tmp[0] == 0x55)&&(tmp[1] == 0x01)&&(BCC_State == calc_nmea_checksum(tmp)))//判断包头(0x11)同时判断是否有数据是否4个
            {
							switch(tmp[2])
							{
        				case 0x11://将接收的值设置成ADC_set
										P54 =1 ;
										ADC_set =0;
										ADC_set += tmp[3];
										ADC_set += tmp[4]<<8;

    							  sprintf(string,"ADC_SET is %d!\r\n",ADC_set);
									  SendString(string);
   								P54 = 0;
									break;
								case 0x12://存储ADC_auto
									P54 =1 ;	
									ReadIapDefault(tmp_value);
									tmp_value[0] = tmp[3];//写入自动保暖功能时低于温度自动开启的ADC值，默认+0x0300	
									tmp_value[1] = tmp[4];//写入自动保暖功能时保暖温度的ADC值，默认+0x0300
								  WriteIapDefault(tmp_value);
								  sprintf(string,"ADC_autoopen_SET is write %d!\r\nADC_autoclose_SET is write %d!\r\n",ReadIapADC(ADC_autoopen_low_Address),ReadIapADC(ADC_autoopen_high_Address));
								  SendString(string);	
									P54 =0;
											IAP_CONTR = 0x60;//重启单片机
									break;
								case 0x13://存储ADC_NOM_Low_High
									P54 =1 ;
									ReadIapDefault(tmp_value);
									tmp_value[2] = tmp[3];//写入低温预设的ADC值，默认+0x0300	
									tmp_value[3] = tmp[4];//写入高温预设的ADC值，默认+0x0300
								  WriteIapDefault(tmp_value);

/*						for(i=0; i<6; i++){
//								tmp[i]=tmp_value[i];//取最后一个以外的数据，用于校验
								UartSend(tmp_value[i]);//原样传回，用于测试
							}
													
*/
  								sprintf(string,"ADC_Low_SET is write %d!\r\nADC_High_SET is write %d!\r\n",ReadIapADC(ADC_set_low_Address),ReadIapADC(ADC_set_high_Address));
								  SendString(string);	
								P54 = 0;
											IAP_CONTR = 0x60;//重启单片机
									break;
								case 0x14://复位ADC_High，ADC_Low设置,写入低温pwm值和高温pwm值
  									P54 =1 ;
								
									ReadIapDefault(tmp_value);
									tmp_value[0] = 0x70;//写入自动保暖功能时低于温度自动开启的ADC值，默认+0x0300	
									tmp_value[1] = 0x5c;//写入自动保暖功能时保暖温度的ADC值，默认+0x0300
									tmp_value[2] = 0x52;//写入	850默认值
									tmp_value[3] = 0x34;//写入	820默认值
									if(tmp[3]!=0)  tmp_value[4] = tmp[3];
											else tmp_value[4] = (u8)ADC_LOW_SET_PWM;
									if(tmp[4]!=0)  tmp_value[5] = tmp[4];
											else tmp_value[5] = (u8)ADC_HIGH_SET_PWM;
								
								  WriteIapDefault(tmp_value);
							

								sprintf(string,"ADC_HIGH_SET reset to %d!\r\n",ReadIapADC(ADC_set_high_Address));
									SendString(string);	
								  sprintf(string,"ADC_LOW_SET reset to %d!\r\n",ReadIapADC(ADC_set_low_Address));
								  SendString(string);	
								sprintf(string,"Heater,SaveBut,LOW_PWM,HIGH_PWM is 0x%02X,0x%02X,0x%02X\r\n",(u16)IapRead(Heater_unable_Address),(u16)IapRead(Button_unable_Address),ReadIapPWM(ADC_low_pwm_Address),ReadIapPWM(ADC_low_pwm_Address));
								  SendString(string);									
											P54 = 0;
											IAP_CONTR = 0x60;//重启单片机
										break;
								case 0x15://设置方向盘加热器为夏天模式和是否启用保存的方向盘加热状态，防止误触
											P54 =1 ;
										IapErase(Heater_unable_Address);
										IapProgram(Heater_unable_Address, tmp[3]);//写入低位，00为关闭，01为激活方向盘加热触摸控制功能
										IapProgram(Button_unable_Address, tmp[4]);//写入低位，00为关闭，01为激活使用保存的方向盘加热状态功能；02为激活方向盘自动加热功能

										
								if(!IapRead(Heater_unable_Address)) 
											{
												SendString("Wheel Heater button be Inactivated!\r\n");
											}else{
												SendString("Wheel Heater button be Activated!\r\n");		
											}

								switch(IapRead(Button_unable_Address)){
											case 0x00:
												SendString("Wheel Heater Save Status be Inactivated!\r\n");	
												break;
											case 0x01:
												SendString("Wheel Heater Save Status be Activated!\r\n");	
												break;
											case 0x02:
												SendString("Wheel Heater autoopen be Activated!\r\n");	
												break;
											}
											P54 = 0;

									break;
								default:
								  SendString("Command is wrong!\r\n");
									SendString("Please Tx 0x55 0x01 0x11--0x15");
									SendString("value BCC_code\r\n");	
									break;
							}
						}else{
							
							//SendString("Wrong Command or Address or ");
							//SendString("CRC errror!\r\nPlease Tx ");
							//SendString("0x55 0x01 0x11--0x15 value BCC_code\r\n");
							RX1_Cnt = 0;
							
						}
							RX1_Cnt = 0;  //							  // 
					}	
				}
            

				


      if (time_50ms_ok)            //每50ms执行一次，  
        {  
             time_50ms_ok =0;  

							 if(Wheel_Heater_activated != 0){ //如果方向盘按键控制加热功能激活
										key = key_read();       //《====== 50ms一次调用按键中间层函数  
									if(key!=0){ 
										sprintf(string,"Key_value is %d!\r\n",key);
										SendString(string);
										sprintf(string,"ADC_value is  %d!\r\n",adc_value);			  
										SendString(string);		
									}
							 }else{
										ADC_set = 0;//关闭方向盘加热功能
							 }
							 
							 api_read_adc(&adc_value,2 );			//读取ntc10k阻值，获取温度信息					
//									sprintf(string,"Adc_Value is %d!\r\n",adc_value);
//								  SendString(string);

//						adc_value =840;	 
//						ADC_set =820;
							 
    					WDT_CONTR = 0x37;  //看门狗复位			
					
							if(key == S_key)
							{
										ADC_set = ADC_low; //30C
								    if(IapRead(Heater_unable_Address)==0x01){
											IapErase(Heater_status_Address);
											IapProgram(Heater_status_Address, 0x12);//记录温度低加热初始状态
										}		
																		  sprintf(string,"Adc is set to Low %d!\r\n",ADC_set);
																			SendString(string);										
							}
							if(key == D_key){
										ADC_set = ADC_high;  //40C
								    if(IapRead(Heater_unable_Address)==0x01){
											IapErase(Heater_status_Address);
										  IapProgram(Heater_status_Address, 0x24);//记录温度高加热初始状态
										}
																		  sprintf(string,"Adc is set to High %d!\r\n",ADC_high);
																			SendString(string);
							}
							if(key == L_key){
										ADC_set=0;
								    if(IapRead(Heater_unable_Address)==0x01){
											IapErase(Heater_status_Address);
											IapProgram(Heater_status_Address,0x00);//复位加热初始状态
										}
																			SendString("Close!Adc is set to 0!\r\n");
										pwm = 0;//强制关闭输出
							}
							if(ADC_set > 0){
								
										if(adc_value < ADC_set-5)//到温度最高点，关闭输出
											{
												pwm = 0;
																		if(debug)		  SendString("Temp is High ,Hearter is close!\r\n");
											}
											else  if(adc_value > ADC_set)//不到温度
											{

																		if(adc_value  < ReadIapADC(ADC_set_low_Address)){//限制输出，防止车辆保险报错
																			
																			pwm = ADC_HIGH_SET_PWM;//0.85比例
																			if(debug)		SendString("Temp set is High ,PWM is Low !\r\n");
																		}else{
																			
																			pwm = ADC_LOW_SET_PWM;//0.9比例
																			if(debug)		SendString("Temp set is low ,PWM is High!\r\n");
																		}
											}else{//到达预设值，采取低功率保持温度

																			if(debug)		SendString("Temp is Setting ,PWM is Mid!\r\n");
																			pwm = 205; //0.8比例
												}
										}else//如果adc_set <= 0
											{
												pwm = 0;
																			if(debug)		SendString("Temp set is <=0 ,Hearter is close!\r\n");
											}

  									 sprintf(string,"PWM_Value is %d,ADC_SET iS %d!\r\n",pwm,ADC_set);
  								  SendString(string);
											Pwm_outset(pwm);//pwm 输出

				}
	}
}


void timer0() interrupt 1     //定时器T0中断函数入口,检测按键输入情况
{
     TH0=0X4C;             //初值重载
     TL0=0X00;           //定时50ms=50000us; 50000/2=25000
     time_50ms_ok = 1;
//     key_time++;        //50MS++
}
