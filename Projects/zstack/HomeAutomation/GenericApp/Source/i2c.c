#include "i2c.h"
#include "ioCC2530.h"


/*�ҵĹܽŶ�����
SDA����ΪP1.5
//SCL����ΪP1.4 */
#define SCL 	P1_5
#define SDA 	P1_4
#define PDIR 	P1DIR
#define PSEL    P1SEL
#define SCLDIR  0x20
#define SDADIR  0x10

//#define SDA 	P0_0
//#define PDIR 	P0DIR
//#define SCLDIR  0x02
//#define SDADIR  0x01



/*
һ��nop����һ������ָ������ = 1/32MHz
��32��nop����1us��
----������outman���������Ľ���,�ڴ�������л
*/
void delay_us(uint8 microSecs) {
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}

unsigned char error; /*������ʾ,ȫ�ֱ���*/
    
/*
��ͨ��CC2430��IO�˿�������д���ݵ�ʱ��
���뽫��Ӧ��IO�˿����ݷ�������Ϊ���
CC2430��DIRPX_YΪ1ʱIO��Ϊ�������
DIRPX_YΪ0ʱIO��Ϊ���빦��
�ҵ�SDA��P1.5,��SDAΪ�������ʱ
P1��DIRӦ����0010 0000
��0x20,����������������......
�������,��������....
*/

void IIC_Config(void){
    PSEL &=~(SDADIR|SCLDIR);
}
void WriteSDA1(void)//SDA ���1,�൱��51�����SDA=1
{
         PDIR |= SDADIR;
         SDA = 1;
    }
    
void WriteSDA0(void)//SDA ���0  
{
     PDIR |= SDADIR;
     SDA = 0;
}
    
void WriteSCL1(void)//SCL ���1  
{
     PDIR |= SCLDIR;
     SCL = 1;
}

void WriteSCL0(void)//SCL ���1 
{
     PDIR |= SCLDIR;
     SCL = 0;
}

void ReadSDA(void)//��������SDA��ӦIO��DIR���Խ�������
{
     PDIR &= ~SDADIR;
}

/*����I2C���ߵĺ�������SCLΪ�ߵ�ƽʱʹSDA����һ��������*/        
void IIC_Start(void)
{
    WriteSDA1();
    WriteSCL1();
    delay_us(5);
    WriteSDA0();
    delay_us(5);
    WriteSCL0();
    delay_us(5);//��סIIC ���ߣ�׼�����ͻ��������
}

/*��ֹI2C���ߣ���SCLΪ�ߵ�ƽʱʹSDA����һ��������*/
void IIC_Stop(void)
{
    WriteSDA0();
    delay_us(5);
    WriteSCL1();
    delay_us(5);
    WriteSDA1();
    delay_us(5);
    WriteSCL0();
    delay_us(5);
}

/*����0����SCLΪ�ߵ�ƽʱʹSDA�ź�Ϊ��*/
void SEND_0(void)   /* SEND ACK */
{
    WriteSDA0();
    WriteSCL1();
    delay_us(5);
    WriteSCL0();
    delay_us(5);
}

/*����1����SCLΪ�ߵ�ƽʱʹSDA�ź�Ϊ��*/
void SEND_1(void)
{
    WriteSDA1();
    WriteSCL1();
    delay_us(5);
    WriteSCL0();
    delay_us(5);
}

/*������һ���ֽں�����豸��Ӧ���ź�*/    
char IIC_Wait_Ack(void)
{
	bool bit;
    WriteSDA1();
    WriteSCL1();
    delay_us(5);
	ReadSDA();
    bit=SDA;
    delay_us(5);
    WriteSCL0();
    delay_us(5);
    return bit;
}

void Write_Acknowledge(void)
{
    WriteSDA0();   
    delay_us(5);
    WriteSCL1();   
    delay_us(5);
    WriteSCL0();   
    delay_us(5);
}

/*��I2C����дһ���ֽ�*/
void IIC_Send_Byte(uint8 b)
{
    uint8 i;
    for(i=0;i<8;i++)
    {
      if((b<<i)&0x80)
      {
         SEND_1();
      }
      else
      {
         SEND_0();
      }
    }
}

/*��I2C���߶�һ���ֽ�*/
uint8 IIC_Read_Byte(void)
{
    uint8 receive=0,i;
//    WriteSDA1();

     ReadSDA();
	 

    for(i=0;i<8;i++)
    {   
        WriteSCL0();
        delay_us(5);
        WriteSCL1(); 
        delay_us(5);
		receive <<=1;
        if(SDA) receive ++;
    }
//    WriteSCL0();
    Write_Acknowledge();
    return receive; 
}
