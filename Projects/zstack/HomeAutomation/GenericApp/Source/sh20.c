#include "sh20.h"
#include "user_printf.h"  
bool SHT2X_StartMeasureNHM(char whatdo){//0xF3:温度测量，0xF5:湿度测量，都是非主机模式
    delay_us(50);
    IIC_Start();
    IIC_Send_Byte(0x80);
    if(IIC_Wait_Ack()==0){
        IIC_Send_Byte(whatdo);
    	if(IIC_Wait_Ack()==0){
//		    do {
//		        delay_us(50);
//		        IIC_Start();
//		    }
//			while(IIC_Send_Byte(0x81),IIC_Wait_Ack()==1); //中间是逗号运算符
			return TRUE;
		}else
		return FALSE;
	}//此处必须等待应答，不可用延时代替，否则无法读出，读数128 都是数据错误
	
	return FALSE;
}


bool SHT2X_MeasureReady(char whatdo){
	delay_us(50);
	IIC_Start();
	IIC_Send_Byte(0x81);
	
	if (IIC_Wait_Ack())
		return FALSE;
	else
		return TRUE;
}

int16 SHT2X_ReadMeasure(char whatdo){
    float Humidity,Temperature;
    uint8 tmp1,tmp2;
    uint16 ST;
	delay_us(50);
	tmp1= IIC_Read_Byte();
	delay_us(50);
	tmp2= IIC_Read_Byte();
	IIC_Read_Byte();
	IIC_Stop();
	delay_us(50);
	ST = (tmp1 << 8) | (tmp2 << 0);
	ST &= ~0x0003;
	if(whatdo==((char)0xF3)){
		Temperature = ((float)ST * 0.00268127) - 46.85;
		return ((int16)(Temperature*100));
	}
	else{
		//这里默认另一种就是非主机湿度测量，如需要主机测量模式请自己添加
		Humidity = ((float)ST * 0.00190735) - 6;
		return ((int16)(Humidity*100));
	}
}

//SHT20 软件复位
void SoftReset(void) //软重启
{
	IIC_Start();
	IIC_Send_Byte(0x80); //SHT20 器件地址+write
	IIC_Send_Byte(0xfe);
	IIC_Stop();
}
//设置分辨率，可以不用，默认就行
void Set_Resolution(uint8 resolution)
{
	resolution |= 0x02 ;
	IIC_Start();
	IIC_Send_Byte(0x80); //发送写命令
	if(IIC_Wait_Ack()==0){
		IIC_Send_Byte(0xE6); //写用户寄存器
		if(IIC_Wait_Ack()==0){
			if(IIC_Send_Byte(0x83),IIC_Wait_Ack()==0); //11bit RH%; 11bit temp
		}
	}
}
