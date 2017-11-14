#include "sh20.h"
#include "user_printf.h"
#define u8  uint8
#define u16 uint16

float SHT2X_MeasureNHM(char whatdo){//0xF3:�¶Ȳ�����0xF5:ʪ�Ȳ��������Ƿ�����ģʽ
    float Humidity,Temperature;
    u8 tmp1,tmp2;
    u16 ST;
    delay_us(50);
    IIC_Start();
    IIC_Send_Byte(0x80);
    if(IIC_Wait_Ack()==0){
        IIC_Send_Byte(whatdo);
    	if(IIC_Wait_Ack()==0){
		    do {
		        delay_us(50);
		        IIC_Start();
		    }
			while(IIC_Send_Byte(0x81),IIC_Wait_Ack()==1); //�м��Ƕ��������
		}
	}//�˴�����ȴ�Ӧ�𣬲�������ʱ���棬�����޷�����������128 �������ݴ���
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
		return (Temperature);
	}
	else{
		//����Ĭ����һ�־��Ƿ�����ʪ�Ȳ���������Ҫ��������ģʽ���Լ����
		Humidity = ((float)ST * 0.00190735) - 6;
		return (Humidity);
	}
	
}
//SHT20 �����λ
void SoftReset(void) //������
{
	IIC_Start();
	IIC_Send_Byte(0x80); //SHT20 ������ַ+write
	IIC_Send_Byte(0xfe);
	IIC_Stop();
}
//���÷ֱ��ʣ����Բ��ã�Ĭ�Ͼ���
void Set_Resolution(void)
{
	IIC_Start();
	IIC_Send_Byte(0x80); //����д����
	if(IIC_Wait_Ack()==0){
		IIC_Send_Byte(0xE6); //д�û��Ĵ���
		if(IIC_Wait_Ack()==0){
			if(IIC_Send_Byte(0x83),IIC_Wait_Ack()==0); //11bit RH%; 11bit temp
		}
	}
}