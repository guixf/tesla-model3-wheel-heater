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

#define     Tmp_Length          5      //��дEEPROM���峤��

#define     UART1_BUF_LENGTH    (Tmp_Length+6)  //���ڻ��峤��

u16 ADC_low = 0;
u16 ADC_high = 0;
u8  RX1_TimeOut;
u8  TX1_Cnt;    //���ͼ���
u8  RX1_Cnt;    //���ռ���
bit B_TX1_Busy; //����æ��־

u8  BCC_State;    //�����

u8  xdata RX1_Buffer[UART1_BUF_LENGTH]; //���ջ���
u8  xdata   tmp[Tmp_Length];        //EEPROM��������

bit busy;


char *ID="test\r\n";


void IapIdle()
{
    IAP_CONTR = 0;                              //�ر�IAP����
    IAP_CMD = 0;                                //�������Ĵ���
    IAP_TRIG = 0;                               //��������Ĵ���
    IAP_ADDRH = 0x80;                           //����ַ���õ���IAP����
    IAP_ADDRL = 0;
}

char IapRead(int addr)
{
    char dat;

    IAP_CONTR = 0x80;                           //ʹ��IAP
    IAP_TPS = 12;                               //���õȴ�����12MHz
    IAP_CMD = 1;                                //����IAP������
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();
    dat = IAP_DATA;                             //��IAP����
    IapIdle();                                  //�ر�IAP����

    return dat;
}

void IapProgram(int addr, char dat)
{
    IAP_CONTR = 0x80;                           //ʹ��IAP
    IAP_TPS = 12;                               //���õȴ�����12MHz
    IAP_CMD = 2;                                //����IAPд����
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_DATA = dat;                             //дIAP����
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();
    IapIdle();                                  //�ر�IAP����
}

void IapErase(int addr)
{
    IAP_CONTR = 0x80;                           //ʹ��IAP
    IAP_TPS = 12;                               //���õȴ�����12MHz
    IAP_CMD = 3;                                //����IAP��������
    IAP_ADDRL = addr;                           //����IAP�͵�ַ
    IAP_ADDRH = addr >> 8;                      //����IAP�ߵ�ַ
    IAP_TRIG = 0x5a;                            //д��������(0x5a)
    IAP_TRIG = 0xa5;                            //д��������(0xa5)
    _nop_();                                    //
    IapIdle();                                  //�ر�IAP����
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


//�����ݴ�������������ع�����;ͨ��Ҫ��֤���ݵĿɿ��ԺͰ�ȫ��;����
//���ͷ��ͽ��շ�ҪԼ����ͬ��Э��;�����Э���г��������У��͵����㡣
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
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR &= 0xBF;		//��ʱ��ʱ��12Tģʽ
	AUXR &= 0xFE;		//����1ѡ��ʱ��1Ϊ�����ʷ�����
	TMOD &= 0x0F;		//���ö�ʱ��ģʽ
	TL1 = 0xFE;		//���ö�ʱ��ʼֵ
	TH1 = 0xFF;		//���ö�ʱ��ʼֵ
	ET1 = 0;		//��ֹ��ʱ��%d�ж�
	TR1 = 1;		//��ʱ��1��ʼ��ʱ
	
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
 
    while (*s)                   //����ַ���������־
    {
        UartSend(*s++);         //���͵�ǰ�ַ�
    }
		
}

void hal_init_ADC()
{
 

P3M0=P3M0&~(0x0C);//P32��P33����
P3M1=P3M1|0x0c;//P32��P33����


     
	 //ADCCFG�Ĵ���
	 //    B7__B6__ B5    __B4__B3__B2__B1__B0__ 
	 //	  - __- __RESFMT __- __   SPEED[3:0] __
	 //    RESFMT��=1 �Ҷ���	:=0 �����
	 //    SPEED adcʱ�ӿ���	 ADCʱ��:SYSclk/2/16/SPEED 
	 //          �磺 SPEED=1111��SYSclk=24MHz�� ��Fadc=24M/2/16/16=46.875kHz, tadc=21ms
    ADCCFG = 0x22; //f_����ADCʱ��:Fadc=11M/2/16/2=171kHz,tadc=1.3us
                   //2_����Ϊ�Ҷ���
									 //adc
    //ADC_CONTR�Ĵ���
	//    B7__________B6__________B5 ________B4__B3__B2__B1__B0__ 
	//	  ADC_POWER___ADC_STRAT___ADC_FLAG __- __  ADC_CHS[3:0] __
	//    ADC_STRAT��=1 ��ʼת��������Զ�����	:=0 ������
	//    ADC_FLAG��ת�����Ӳ����1������������㡣 
	//	  ADC_CHS[3:0]ѡ��ͨ��0-14��(P1.0��P1.7,P0.0-P0.6),15���ڲ�refv�ĵ�ѹ1.344
  //              ѡ��ͨ��1001 ��P01
       
    //ADC_CONTR = 0x8A;        //��ADCģ���Դ,ѡ��ͨ��1010 ����P02
}

void api_read_adc(unsigned int *Adc_result,unsigned char adc_channel)
{
     
    ADC_CONTR = (0xC0|adc_channel);     //����ADת��
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //��ѯADC��ɱ�־
    ADC_CONTR &= ~0x20;                      //����ɱ�־
    *Adc_result = (ADC_RES<<8|ADC_RESL);    //��ȡADC������Ҷ��룬��λ�Զ����Ϊ0


    ADC_CONTR = (0xC0|adc_channel);   //����ADת��
    _nop_();
    _nop_();
    while (!(ADC_CONTR & 0x20));            //��ѯADC��ɱ�־
    ADC_CONTR &= ~0x20;                     //����ɱ�־
	  *Adc_result = (ADC_RES<<8|ADC_RESL);     //��ȡADC������Ҷ��룬��λ�Զ����Ϊ0
}


unsigned int  adc_value;
unsigned int  ADC_set=0;
unsigned char string[20];
unsigned int  pwm=0;
unsigned int  pwm_count=0;
unsigned int 	key=0; 
unsigned int  time_10ms_ok=0;
unsigned int  key_time=0;
unsigned int  start_state=0;

 /*=============
�Ͳ㰴����I/0��ɨ�躯�������Ͳ㰴���豸������ֻ�����޼����̰��ͳ�����
===============*/

#define key_input    P33    // ���������

#define N_key    0             //�޼�
#define S_key    1             //����
#define D_key    2             //˫��
#define L_key    3             //����

#define key_state_0 0
#define key_state_1 1
#define key_state_2 2
#define key_state_3 3

 
unsigned char key_driver(void)
{
    static unsigned char key_state = key_state_0, key_time = 0;
    unsigned char key_press, key_return = N_key;

    key_press = key_input;                    // ������I/O��ƽ

			P54 = key_press;//���������
	
    switch (key_state)
    {
      case key_state_0:                              // ������ʼ̬
        if (key_press) key_state = key_state_1;      // �������£�״̬ת��������������ȷ��״̬;������״̬����
        break;
      
      case key_state_1:                      // ����������ȷ��̬
        if (key_press)
        {
             key_time = 0;                   //  
             key_state = key_state_2;   // ������Ȼ���ڰ��£�������ɣ�״̬ת�������¼�ʱ��ļ�ʱ״̬�������صĻ����޼��¼�
        }
        else
             key_state = key_state_0;   // ������̧��ת����������ʼ̬���˴���ɺ�ʵ�������������ʵ�����İ��º��ͷŶ��ڴ������ġ�
        break;
      
      case key_state_2:
        if(!key_press)
        {
             key_return = S_key;        // ��ʱ�����ͷţ�˵���ǲ���һ�ζ̲���������S_key
             key_state = key_state_0;   // ת����������ʼ̬
        }
        else if (++key_time >= 10)     // �������£���ʱ��10ms��10msΪ������ѭ��ִ�м����
        {
             key_return = L_key;        // ����ʱ��>1000ms���˰���Ϊ�������������س����¼�
             key_state = key_state_3;   // ת�����ȴ������ͷ�״̬
        }
        break;

      case key_state_3:                 // �ȴ������ͷ�״̬����״ֻ̬�����ް����¼�
        if (!key_press) key_state = key_state_0; //�������ͷţ�ת����������ʼ̬
        break;
    }
		
//      	P54 = key_press;                       //������ 
    return key_return;
}

/*=============
�м�㰴�������������õͲ㺯��һ�Σ�����˫���¼����жϣ������ϲ���ȷ���޼���������˫��������4�������¼���
���������ϲ�ѭ�����ã����10ms
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
                 key_time_1 = 0;               // ��1�ε����������أ����¸�״̬�жϺ����Ƿ����˫��
                 key_m = key_state_1;
            }
            else
								{
									key_return = key_temp;        // �����޼�������������ԭ�¼�
								}
            break;

        case key_state_1:
            if (key_temp == S_key)             // ��һ�ε���������϶�<500ms��
            {
                 key_return = D_key;           // ����˫�����¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            else                                
            {                                  // ����500ms�ڿ϶������Ķ����޼��¼�����Ϊ����>1000ms����1sǰ�Ͳ㷵�صĶ����޼�
                 if(++key_time_1 >= 20)
                 {
                      key_return = S_key;      // 500ms��û���ٴγ��ֵ����¼���������һ�εĵ����¼�
                      key_m = key_state_0;     // ���س�ʼ״̬
                 }

             break;
        case key_state_3:
            if (key_temp == L_key )
            {
                 key_return = L_key;           // ���س������¼����س�ʼ״̬
                 key_m = key_state_0;
            }
            break;
		}
    return key_return;
	}     
}

void Timer0Init(void)		//50000΢��@11.0592MHz
{
	AUXR &= 0x7F;		//��ʱ��ʱ��12Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0x00;		//���ö�ʱ��ʼֵ
	TH0 = 0x4C;		//���ö�ʱ��ʼֵ
	TF0 = 0;		//���TF0��־
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
}



void main()
{
	    u8  i,j;
	u16 Iap_tmp;
 //char i;
  Timer0Init();
 UartInit();
 ES = 1;
  WDT_CONTR = 0x27; //8s
	//								  SendString(P3M1);
	

 hal_init_ADC();
     P5M0 = 0x10;                                //����P5.4Ϊ�������ģʽ
     P5M1 = 0x00;

     P54 = 0;  
     P55 = 0;  

    ET0 = 1;    //Timer0 interrupt enable
    TR0 = 1;    //Tiner0 run

	  EA = 1;     //�����ж�
		
//									sprintf(string,"Start!\r\n");
//								  SendString(string);
			//P0 = ;	//read eeprom
//											 UartSend(IapRead(0x0400));	

										ADC_low = ReadIapADC(0x0200);
										if (ADC_low > 1000||ADC_low < 700) ADC_low=860;
										ADC_high = ReadIapADC(0x0000);
										if (ADC_high <700 || ADC_high >1000) ADC_high=810;
									sprintf(string,"Adc_set_Low is %d!\r\n",ADC_low);
								  SendString(string);										
									sprintf(string,"Adc_set_High is %d!\r\n",ADC_high);
								  SendString(string);
switch(IapRead(0x0400))
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
	while (1)
	{
   
		//���uart1�յ�������
        if(RX1_TimeOut > 0)     //��ʱ����
        {
					//for(i=0; i<RX1_Cnt; i++)    UartSend(RX1_Buffer[i]);//ԭ�����أ����ڲ���

				 if(--RX1_TimeOut == 0 && RX1_Cnt == 6)
					 {
							//tmp = 0x00;
							for(i=0; i<5; i++){
								tmp[i]=RX1_Buffer[i];//ȡ���һ����������ݣ�����У��
//								UartSend(tmp[i]);//ԭ�����أ����ڲ���
							}
							BCC_State = RX1_Buffer[5];

            if((tmp[0] == 0x55)&&(tmp[1] == 0x01)&&(BCC_State == calc_nmea_checksum(tmp)))//�жϰ�ͷ(0x11)ͬʱ�ж��Ƿ��������Ƿ�4��
            {
							switch(tmp[2])
							{
        				case  0x11://�����յ�ֵ���ó�ADC_set
									ADC_set =0;
								  ADC_set += tmp[3];
									ADC_set += tmp[4]<<8;
    								sprintf(string,"ADC_SET is %d!\r\n",ADC_set);
								  SendString(string);
									break;
								case 0x12://�洢ADC_Low
									
										IapErase(0x0200);
										//IapErase(0x0411);
										IapProgram(0x0200, tmp[3]);//д���λ	
										IapProgram(0x0201, tmp[4]);//д���λ									
    								//sprintf(string,"ADC_LOW_SET is write\r\n",ADC_set);
								  sprintf(string,"ADC_LOW_SET is write %d!\r\n",ReadIapADC(0x0200));
								  SendString(string);	
									break;
								case 0x13://�洢ADC_High
										IapErase(0x0000);
										//IapErase(0x0421);
										IapProgram(0x0000, tmp[3]);//д���λ	
										IapProgram(0x0001, tmp[4]);//д���λ									
    								//sprintf(string,"ADC_LOW_SET is write\r\n",ADC_set);
								  sprintf(string,"ADC_HIGH_SET is write %d!\r\n",ReadIapADC(0x0000));
								  SendString(string);	
									break;
								case 0x14://��λADC_High��ADC_Low����
										IapErase(0x0000);
										IapErase(0x0200);
										IapProgram(0x0000, 0x34);//д���λ	
										IapProgram(0x0001, 0x03);//д���λ		
										IapProgram(0x0200, 0x66);//д���λ	
										IapProgram(0x0201, 0x03);//д���λ										
    								//sprintf(string,"ADC_LOW_SET is write\r\n",ADC_set);
								  sprintf(string,"ADC_HIGH_SET reset to %d!\r\n",ReadIapADC(0x0000));
								  SendString(string);	
									sprintf(string,"ADC_LOW_SET reset to %d!\r\n",ReadIapADC(0x0200));
								  SendString(string);	
									break;
								default:
								  SendString("Command is wrong!\r\nPlease Tx 0x55 0x01 0x11--0x14 value BCC_code\r\n");									  
							}
							

						}else{
									SendString("Wrong Command or Address or CRC errror!\r\nPlease Tx 0x55 0x01 0x11--0x14 value BCC_code\r\n");
						}
							RX1_Cnt = 0;  //							  // 
					}	
				}
            

				
			if(pwm_count>20)   pwm_count=0;	 

       if (time_10ms_ok)            //ÿ50msִ��һ�Σ�  
        {  
             time_10ms_ok =0;  

							 key = key_read();       //��====== 50msһ�ε��ð����м�㺯��  

									api_read_adc(&adc_value,2 );								

    					WDT_CONTR = 0x37;  //watchdog clear			
					
							if(key == S_key)
							{
							ADC_set = ADC_low; //30C
								    IapErase(0x0400);
										IapProgram(0x0400, 0x12);//write low-heater	
							}
							if(key == D_key){
								ADC_set = ADC_high;  //40C
								    IapErase(0x0400);
										IapProgram(0x0400, 0x24);//write high-heater
							}
							if(key == L_key){
								ADC_set=0;
								    IapErase(0x0400);
										IapProgram(0x0400,0x00);//write close-heater
								P55 = 0;//qiangzhi close
							}
							if(ADC_set > 0){
//								sprintf(string,"ADC_SET is %d\r\n",ADC_set);
//								  SendString(string);
								
										if(adc_value < ADC_set-5)//���¶���ߵ㣬�ر����
											{
												pwm = 0;
												P55=0;
											}
											else  if(adc_value > ADC_set)//����
											{
											if(adc_value < ADC_set+10)// near adc_set
												{
														pwm = 17;
												}
												else if(adc_value < ADC_set+20)  //adc_set mid
												{
													  pwm=17;
												}
												else  //adc_set low
												{
														pwm=18;
												}
											}
											else //adc_set -- adc_set-5
											{
									      pwm = 17;
											}
									}else
							{
										pwm = 0;
							}

							if(pwm > pwm_count){//start PWM
								P55 = 1;
							}else{
								P55 = 0;
							}
							pwm_count++;
				}
	}
}


void timer0() interrupt 1     //��ʱ��T0�жϺ������,��ⰴ���������
{
     TH0=0X4C;             //��ֵ����
     TL0=0X00;           //��ʱ50ms=50000us; 50000/2=25000
     time_10ms_ok = 1;
     key_time++;        //50MS++
}

