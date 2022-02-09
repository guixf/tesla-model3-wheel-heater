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

#define     Tmp_Length          5      //读写EEPROM缓冲长度

#define     UART1_BUF_LENGTH    (Tmp_Length+6)  //串口缓冲长度

u16 ADC_low = 0;
u16 ADC_high = 0;
u8  RX1_TimeOut;
u8  TX1_Cnt;    //发送计数
u8  RX1_Cnt;    //接收计数
bit B_TX1_Busy; //发送忙标志

u8  BCC_State;    //异或结果

u8  xdata RX1_Buffer[UART1_BUF_LENGTH]; //接收缓冲
u8  xdata   tmp[Tmp_Length];        //EEPROM操作缓冲

bit busy;


char *ID="test\r\n";


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

unsigned int ReadIapADC(int addr)
{
	u16 Itmp = 0 ;
			Itmp = IapRead(addr);
			addr++;
			Itmp += IapRead(addr)<<8;
	return  Itmp;
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
	     n=sizeof(*setence);
           while(*setence)
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
 

P3M0=P3M0&~(0x0C);//P32、P33高阻
P3M1=P3M1|0x0c;//P32、P33高阻


     
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


unsigned int  adc_value;
unsigned int  ADC_set=0;
unsigned char string[20];
unsigned int  pwm=0;
unsigned int  pwm_count=0;
unsigned int 	key=0; 
unsigned int  time_50ms_ok=0;
unsigned int  key_time=0;
unsigned int  start_state=0;

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



void main()
{
	    u8  i;
	    u8  Wheel_Heater_activated = 0;

			Timer0Init();	 //	启动定时器0
			UartInit();    //启动uart
      ES = 1;
      WDT_CONTR = 0x27; //启动看门狗，8s重启

	

      hal_init_ADC();    //初始化ADC
			P5M0 = 0x10;                                //设置P5.4为推挽输出模式
			P5M1 = 0x00;

			P54 = 0;  
			P55 = 0;  

			ET0 = 1;    //Timer0 interrupt enable
			TR0 = 1;    //Tiner0 run

			EA = 1;     //打开总中断
		


										ADC_low = ReadIapADC(0x0200);
										if (ADC_low > 1000||ADC_low < 700) ADC_low=860;
										ADC_high = ReadIapADC(0x0000);
										if (ADC_high <700 || ADC_high >1000) ADC_high=810;
									sprintf(string,"Adc_set_Low is %d!\r\n",ADC_low);
								  SendString(string);										
									sprintf(string,"Adc_set_High is %d!\r\n",ADC_high);
								  SendString(string);
									
		switch(IapRead(0x0400))  //获取上次保存的加热状态
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
								  //SendString(string);		
		  key=0;	

      Wheel_Heater_activated	= IapRead(0x0600);//读取方向盘加热按键控制功能激活状态，00为未激活，其他为激活
			
		while (1)
		{
   
		//检查uart1收到的数据
        if(RX1_TimeOut > 0)     //超时计数
        {
					//for(i=0; i<RX1_Cnt; i++)    UartSend(RX1_Buffer[i]);//原样传回，用于测试

				 if(--RX1_TimeOut == 0 && RX1_Cnt == 6)
					 {
							//tmp = 0x00;
							for(i=0; i<5; i++){
								tmp[i]=RX1_Buffer[i];//取最后一个以外的数据，用于校验
//								UartSend(tmp[i]);//原样传回，用于测试
							}
							BCC_State = RX1_Buffer[5];

            if((tmp[0] == 0x55)&&(tmp[1] == 0x01)&&(BCC_State == calc_nmea_checksum(tmp)))//判断包头(0x11)同时判断是否有数据是否4个
            {
							switch(tmp[2])
							{
        				case  0x11://将接收的值设置成ADC_set
										ADC_set =0;
										ADC_set += tmp[3];
										ADC_set += tmp[4]<<8;
    							sprintf(string,"ADC_SET is %d!\r\n",ADC_set);
									SendString(string);
									break;
								case 0x12://存储ADC_Low
										IapErase(0x0200);
										IapProgram(0x0200, tmp[3]);//写入低位	
										IapProgram(0x0201, tmp[4]);//写入高位									
								  sprintf(string,"ADC_LOW_SET is write %d!\r\n",ReadIapADC(0x0200));
								  SendString(string);	
									break;
								case 0x13://存储ADC_High
										IapErase(0x0000);
										IapProgram(0x0000, tmp[3]);//写入低位	
										IapProgram(0x0001, tmp[4]);//写入高位									
								  sprintf(string,"ADC_HIGH_SET is write %d!\r\n",ReadIapADC(0x0000));
								  SendString(string);	
									break;
								case 0x14://复位ADC_High，ADC_Low设置
										IapErase(0x0000);
										IapErase(0x0200);
										IapProgram(0x0000, 0x34);//写入低位	
										IapProgram(0x0001, 0x03);//写入高位		
										IapProgram(0x0200, 0x66);//写入低位	
										IapProgram(0x0201, 0x03);//写入高位	
								    IapErase(0x0400);
										IapProgram(0x0400,0x00);//复位加热初始状态								
								  sprintf(string,"ADC_HIGH_SET reset to %d!\r\n",ReadIapADC(0x0000));
								  SendString(string);	
									sprintf(string,"ADC_LOW_SET reset to %d!\r\n",ReadIapADC(0x0200));
								  SendString(string);	
									break;
								case 0x15://设置方向盘加热器为夏天模式，防止误触
										IapErase(0x0600);
										IapProgram(0x0600, tmp[3]);//写入低位，00为关闭，01为激活方向盘加热触摸控制功能
										if(ReadIapADC(0x0600)) 
											{
												SendString("Wheel Heater button be Activated!\r\n");	
											}else{
												SendString("Wheel Heater button be Inactivated!\r\n");	
											}
									break;
								default:
								  SendString("Command is wrong!\r\nPlease Tx 0x55 0x01 0x11--0x15 value BCC_code\r\n");									  
							}
							

						}else{
									SendString("Wrong Command or Address or CRC errror!\r\nPlease Tx 0x55 0x01 0x11--0x15 value BCC_code\r\n");
						}
							RX1_Cnt = 0;  //							  // 
					}	
				}
            

				
			if(pwm_count>20)   pwm_count=0;	 //每1S重新开始一次计时

      if (time_50ms_ok)            //每50ms执行一次，  
        {  
             time_50ms_ok =0;  

							 if(Wheel_Heater_activated != 0){ //如果方向盘按键控制加热功能激活
										key = key_read();       //《====== 50ms一次调用按键中间层函数  
							 }else{
										ADC_set = 0;//关闭方向盘加热功能
							 }
							 
							 api_read_adc(&adc_value,2 );			//读取ntc10k阻值，获取温度信息					

    					WDT_CONTR = 0x37;  //看门狗复位			
					
							if(key == S_key)
							{
										ADC_set = ADC_low; //30C
								    IapErase(0x0400);
										IapProgram(0x0400, 0x12);//记录温度低加热初始状态	
							}
							if(key == D_key){
										ADC_set = ADC_high;  //40C
								    IapErase(0x0400);
										IapProgram(0x0400, 0x24);//记录温度高加热初始状态
							}
							if(key == L_key){
										ADC_set=0;
								    IapErase(0x0400);
										IapProgram(0x0400,0x00);//复位加热初始状态
										P55 = 0;//强制关闭输出
							}
							if(ADC_set > 0){
								
										if(adc_value < ADC_set-5)//到温度最高点，关闭输出
											{
												pwm = 0;
												P55=0;
											}
											else  if(adc_value > ADC_set)//不到
											{
													if(adc_value < ADC_set+20)// 接近预设值
														{
																			pwm = 17;
														}
														else  
														{
																		if(ADC_set<840){//限制输出，防止车辆保险报错
																			pwm = 17;
																		}else{
																			pwm = 18;																			
																		}
														}
											}
												else{//到达预设值，采取低功率保持温度

																			pwm = 16;
												}
										}else//如果adc_set <= 0
											{
												pwm = 0;
											}

							if(pwm > pwm_count){//开始 PWM输出
									P55 = 1;
								}else{
									P55 = 0;
							}
							pwm_count++;//pwm计数
				}
	}
}


void timer0() interrupt 1     //定时器T0中断函数入口,检测按键输入情况
{
     TH0=0X4C;             //初值重载
     TL0=0X00;           //定时50ms=50000us; 50000/2=25000
     time_50ms_ok = 1;
     key_time++;        //50MS++
}

