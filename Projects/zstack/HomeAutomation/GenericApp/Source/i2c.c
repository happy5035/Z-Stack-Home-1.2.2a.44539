#include "i2c.h"
#include "ioCC2530.h"


/*我的管脚定义是
SDA定义为P1.5
//SCL定义为P1.4 */
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
一个nop就是一条机器指令周期 = 1/32MHz
那32个nop就是1us啦
----这里是outman给我做出的讲解,在此再作感谢
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

unsigned char error; /*错误提示,全局变量*/
    
/*
当通过CC2430的IO端口往外面写数据的时候
必须将对应的IO端口数据方向设置为输出
CC2430中DIRPX_Y为1时IO口为输出功能
DIRPX_Y为0时IO口为输入功能
我的SDA是P1.5,则SDA为输出功能时
P1口DIR应该是0010 0000
即0x20,其他可以依次类推......
啰嗦完毕,继续程序....
*/

void IIC_Config(void){
    PSEL &=~(SDADIR|SCLDIR);
}
void WriteSDA1(void)//SDA 输出1,相当于51里面的SDA=1
{
         PDIR |= SDADIR;
         SDA = 1;
    }
    
void WriteSDA0(void)//SDA 输出0  
{
     PDIR |= SDADIR;
     SDA = 0;
}
    
void WriteSCL1(void)//SCL 输出1  
{
     PDIR |= SCLDIR;
     SCL = 1;
}

void WriteSCL0(void)//SCL 输出1 
{
     PDIR |= SCLDIR;
     SCL = 0;
}

void ReadSDA(void)//这里设置SDA对应IO口DIR可以接收数据
{
     PDIR &= ~SDADIR;
}

/*启动I2C总线的函数，当SCL为高电平时使SDA产生一个负跳变*/        
void IIC_Start(void)
{
    WriteSDA1();
    WriteSCL1();
    delay_us(5);
    WriteSDA0();
    delay_us(5);
    WriteSCL0();
    delay_us(5);//锁住IIC 总线，准备发送或接收数据
}

/*终止I2C总线，当SCL为高电平时使SDA产生一个正跳变*/
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

/*发送0，在SCL为高电平时使SDA信号为低*/
void SEND_0(void)   /* SEND ACK */
{
    WriteSDA0();
    WriteSCL1();
    delay_us(5);
    WriteSCL0();
    delay_us(5);
}

/*发送1，在SCL为高电平时使SDA信号为高*/
void SEND_1(void)
{
    WriteSDA1();
    WriteSCL1();
    delay_us(5);
    WriteSCL0();
    delay_us(5);
}

/*发送完一个字节后检验设备的应答信号*/    
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

/*向I2C总线写一个字节*/
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

/*从I2C总线读一个字节*/
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
