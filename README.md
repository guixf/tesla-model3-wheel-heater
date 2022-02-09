# tesla-model3-wheel-heater
此项目为个人闲着蛋疼瞎折腾，非商业目的，遂不提供技术支持。
如有商家、厂家利用此项目进行商业开发，皆与本人无关，本人不对此类商业行为负责。

感谢 鬣狗三号 提供的思路和源码。原贴见：https://www.xiaote.com/r/6076f6e07ebff14410b5958f

实现原理：利用触摸开关对方向盘加热进行控制，并利用原厂方向盘内的ntc10K温度传感器实现温控。

改装方案：

1.在方向盘内合适位置粘贴触摸开关。短按触摸，将温度设置成低，约30度；快速双击触摸，将温度设成高，约35度；长按触摸，关闭方向盘加热。

注意：下次车辆启动时方向盘加热的状态，与下车前方向盘加热的设置一致。

2.原理图，pcb及bom单由立创eda创建，已经导出并上传GitHub。可以在某宝自行搜索pcb打样厂家直接投厂。

3.pcb制作好后，将编译好的程序下载进单片机，或者利用源码自行修改编译，

将原厂不带加热的方向盘拆下，更换新款6点位置带4p线束的加热方向盘

将方向盘与游丝连接线束进行退针，并制作3P并联无损线，将方向盘与自制加热模块（插接件1）并联，

将模块（插接件2）与原厂方向盘加热线束连接。

复原。

4.用离线语音控制SU-03T识别语音并通过ble蓝牙模块，传送到控制板上，实现控制温度功能。

5.用手机易加蓝牙app可写入高低温度作为预设值。

6.蓝牙传送格式为 0x55 0x01 0x11 36 03 75,其中0x55 为命令引导符，0x01 为被操作设备地址 （本控制器设置为01），第三位为命令（0x11-0x14），第四五位为十六进制数值，低位在前，高位在后，最后一位为前面5位十六进制数据的异或校验值。

0x11为直接设置温度命令，0x36 0x03 为16进制adc值（低温默认值为860 ，高温默认值为810）。
设置成功后控制板返还字符串 ADC_SET is 830! 830为第四五位十六进制数值转换后的十进制数值。

0x12为设置低温温度预设值命令，写入成功后控制板返回字符串 ADC_LOW_SET is write 830!。

0x13为设置高温温度预设值命令，写入成功后控制板返回字符串 ADC_HIGH_SET is write 810!。

0x14为复位命令，第四五位数值无效，复位成功后控制板返回字符串 ADC_HIGH_SET reset to 低数值!\r\nADC_LOW_SET reset to 高数值!\r\n。

0x15为激活或关闭方向盘加热按钮控制和是否按上次状态使用方向盘加热功能功能，第四位数值为0，则关闭方向盘加热按钮控制功能，其他数值为激活。设置成功后返回字符串 Wheel Heater button be Activated!\r\n 或者Wheel Heater button be Inactivated!\r\n。

第五位数值为0，则关闭方向盘加热使用保存状态功能，其他数值为激活。设置成功后返回字符串 Wheel Heater Save Status be Activated!\r\n 或者Wheel Heater Save Status be Inactivated!\r\n。

如第三位不是（0x11-0x15），则返回 Command is wrong!\r\nPlease Tx 0x55 0x01 0x11--0x15 value BCC_code\r\n。

如不是0x55开头，或被操作设备地址不为0x01，或者校验码错误，则无返回。

其他命令待扩展。
