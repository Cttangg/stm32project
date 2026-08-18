#include "touch.h"
#include "lcd.h"
#include "delay.h"
#include <stdlib.h>
#include <math.h>

/**
 ******************************************************************************
 * @file    touch.c
 * @brief   电阻触摸屏驱动实现 — 软件模拟 SPI
 *
 *          支持 ADS7843/7846/UH7843/7846/XPT2046/TSC2046/OTT2001A 等驱动 IC,
 *          由正点原子 STM32F4 触摸驱动 (V1.1 20140721) 改造而来:
 *           - 修正 MDK -O2 优化下触摸数据读取失败 (TP_Write_Byte 内加延时)
 *           - 板载无 24CXX EEPROM, 校准数据不保存, 使用预存校准参数
 *
 * 采样策略:
 *   - TP_Read_XOY(): 多次采样排序去极值取平均, 抑制随机噪声
 *   - TP_Read_XY2(): 连续两次采样偏差校验 (±ERR_RANGE), 提高准确度
 ******************************************************************************
 */

/* 板载无 24CXX EEPROM: 校准数据不保存, 每次上电重新校准.
 * 以下为模拟 EEPROM 接口 (空实现) */
static u8  AT24CXX_ReadOneByte(u16 addr)      { (void)addr; return 0; }
static void AT24CXX_WriteOneByte(u16 addr, u8 v) { (void)addr; (void)v; }
static u32 AT24CXX_ReadLenByte(u16 addr, u8 n)   { (void)addr; (void)n; return 0; }
static void AT24CXX_WriteLenByte(u16 addr, u32 v, u8 n) { (void)addr; (void)v; (void)n; }


/* 历史说明 (正点原子 STM32F4 触摸驱动 V1.1 20140721):
 * 修正 MDK -O2 优化时触摸屏数据无法读取的 bug: 在 TP_Write_Byte 函数中添加延时解决 */
//////////////////////////////////////////////////////////////////////////////////

/** 全局触摸设备实例: 绑定 init/scan/adjust 回调 */
_m_tp_dev tp_dev=
{
	.init   = TP_Init,
	.scan   = TP_Scan,
	.adjust = TP_Adjust,
};					
/** 读 X 坐标指令 (默认 touchtype=0) */
u8 CMD_RDX=0XD0;
/** 读 Y 坐标指令 (默认 touchtype=0) */
u8 CMD_RDY=0X90;

#if (TP_PEN_INT_ENABLE == 1)
/** PEN 按下事件标志 (ISR 置位, TP_Scan 消费) */
static volatile u8 g_tp_pen_down;
/** PEN 释放事件标志 (ISR 置位, TP_Scan 消费) */
static volatile u8 g_tp_pen_up;
#endif
 	 			    					   
/**
 * @brief  软件 SPI 写入 1 字节 (MSB 先行, 时钟上升沿采样)
 * @param  num: 要写入的数据
 * @note   含 1us 空转延时, 兼容 MDK -O2 优化
 */
void TP_Write_Byte(u8 num)    
{  
	u8 count=0;   
	for(count=0;count<8;count++)  
	{ 	  
		if(num&0x80)TP_TDIN(GPIO_PIN_SET);  
		else TP_TDIN(GPIO_PIN_RESET);   
		num<<=1;    
		TP_TCLK(GPIO_PIN_RESET); 
		delay_us(1);
		TP_TCLK(GPIO_PIN_SET);		//上升沿有效	        
	}		 			    
} 		 
/**
 * @brief  软件 SPI 读取 12 位 ADC 转换值
 * @param  CMD: 控制指令 (如 CMD_RDX/CMD_RDY)
 * @retval 读到的数据 (高 12 位有效, 已右移 4 位)
 * @note   内含 ADS7846 最长转换时间 (6us) 等待与 O2 优化兼容延时
 */
u16 TP_Read_AD(u8 CMD)	  
{ 	 
	u8 count=0; 	  
	u16 Num=0; 
	TP_TCLK(GPIO_PIN_RESET);		//先拉低时钟 	 
	TP_TDIN(GPIO_PIN_RESET); 	//拉低数据线
	TP_TCS(GPIO_PIN_RESET); 		//选中触摸屏IC
	TP_Write_Byte(CMD);//发送命令字
	delay_us(6);//ADS7846的转换时间最长为6us
	TP_TCLK(GPIO_PIN_RESET); 	     	    
	delay_us(1);    	   
	TP_TCLK(GPIO_PIN_SET);		//给1个时钟，清除BUSY
	delay_us(1);    
	TP_TCLK(GPIO_PIN_RESET); 	     	    
	for(count=0;count<16;count++)//读出16位数据,只有高12位有效 
	{ 				  
		Num<<=1; 	 
		TP_TCLK(GPIO_PIN_RESET);	//下降沿有效  	    	   
		delay_us(1);    
 		TP_TCLK(GPIO_PIN_SET);
 		if(TP_DOUT)Num++; 		 
	}  	
	Num>>=4;   	//只有高12位有效.
	TP_TCS(GPIO_PIN_SET);		//释放片选	 
	return(Num);   
}
/**
 * @brief  读取一个轴的坐标值 (X 或 Y)
 *         连续采样 READ_TIMES 次并升序排序, 去掉最高最低各 LOST_VAL 个
 *         极值后取平均, 以抑制随机噪声
 * @param  xy: 指令 (CMD_RDX / CMD_RDY)
 * @retval 滤波后的读数
 * @note   采样次数/丢弃个数由 touch_conf.h 的 TP_READ_TIMES/TP_LOST_VAL 配置
 */
u16 TP_Read_XOY(u8 xy)
{
	u16 i, j;
	u16 buf[TP_READ_TIMES];
	u16 sum=0;
	u16 temp;
	for(i=0;i<TP_READ_TIMES;i++)buf[i]=TP_Read_AD(xy);		 		    
	for(i=0;i<TP_READ_TIMES-1; i++)//排序
	{
		for(j=i+1;j<TP_READ_TIMES;j++)
		{
			if(buf[i]>buf[j])//升序排列
			{
				temp=buf[i];
				buf[i]=buf[j];
				buf[j]=temp;
			}
		}
	}	  
	sum=0;
	for(i=TP_LOST_VAL;i<TP_READ_TIMES-TP_LOST_VAL;i++)sum+=buf[i];
	temp=sum/(TP_READ_TIMES-2*TP_LOST_VAL);
	return temp;   
} 
/**
 * @brief  读取 X/Y 坐标
 * @param  x: 读取到的 X 坐标指针
 * @param  y: 读取到的 Y 坐标指针
 * @retval 0: 失败; 1: 成功
 */
u8 TP_Read_XY(u16 *x,u16 *y)
{
	u16 xtemp,ytemp;			 	 		  
	xtemp=TP_Read_XOY(CMD_RDX);
	ytemp=TP_Read_XOY(CMD_RDY);	  												   
	if(xtemp<TP_READ_MIN||ytemp<TP_READ_MIN)return 0;//读数失败 (阈值见 touch_conf.h)
	*x=xtemp;
	*y=ytemp;
	return 1;//读数成功
}
/**
 * @brief  双采样校验读坐标
 *         连续 2 次读取触摸屏 IC, 两次偏差不超过 TP_ERR_RANGE 才认为读数
 *         正确并取平均, 否则返回失败. 能大幅提高准确度
 * @param  x: 读取到的坐标值指针
 * @param  y: 读取到的坐标值指针
 * @retval 0: 失败; 1: 成功
 */
u8 TP_Read_XY2(u16 *x,u16 *y) 
{
	u16 x1,y1;
 	u16 x2,y2;
 	u8 flag;    
    flag=TP_Read_XY(&x1,&y1);   
    if(flag==0)return(0);
    flag=TP_Read_XY(&x2,&y2);	   
    if(flag==0)return(0);   
    if(((x2<=x1&&x1<x2+TP_ERR_RANGE)||(x1<=x2&&x2<x1+TP_ERR_RANGE))//前后两次采样在+-ERR_RANGE内
    &&((y2<=y1&&y1<y2+TP_ERR_RANGE)||(y1<=y2&&y2<y1+TP_ERR_RANGE)))
    {
        *x=(x1+x2)/2;
        *y=(y1+y2)/2;
        return 1;
    }else return 0;	  
}  
/**
 * @brief  画一个触摸校准点 (十字线 + 中心圈)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  color: 颜色
 */
void TP_Drow_Touch_Point(u16 x,u16 y,u16 color)
{
	POINT_COLOR=color;
	LCD_DrawLine(x-12,y,x+13,y);//横线
	LCD_DrawLine(x,y-12,x,y+13);//竖线
	LCD_DrawPoint(x+1,y+1);
	LCD_DrawPoint(x-1,y+1);
	LCD_DrawPoint(x+1,y-1);
	LCD_DrawPoint(x-1,y-1);
	LCD_Draw_Circle(x,y,6);//画中心圈
}	  
/**
 * @brief  画一个大点 (2x2 像素)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  color: 颜色
 */
void TP_Draw_Big_Point(u16 x,u16 y,u16 color)
{	    
	POINT_COLOR=color;
	LCD_DrawPoint(x,y);//中心点 
	LCD_DrawPoint(x+1,y);
	LCD_DrawPoint(x,y+1);
	LCD_DrawPoint(x+1,y+1);	 	  	
}						  
/**
 * @brief  触摸扫描 (主循环周期调用)
 * @param  tp: 0=输出屏幕坐标; 1=输出物理坐标 (校准等特殊场合用)
 * @retval 当前触屏状态: 0=无触摸; 1=有触摸
 * @note   当前坐标存入 tp_dev.x[0]/y[0], 按下瞬间坐标记录在 x[4]/y[4];
 *         TP_PEN_INT_ENABLE=1 时由 PEN 中断事件驱动, 无按压时快速返回
 */
u8 TP_Scan(u8 tp)
{
#if (TP_PEN_INT_ENABLE == 1)
	u16 xt,yt;
	/* 释放事件 (上升沿) */
	if(g_tp_pen_up)
	{
		g_tp_pen_up=0;
		g_tp_pen_down=0;
		if(tp_dev.sta&TP_PRES_DOWN)//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);//标记按键松开
		}else//之前就没有被按下
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}
		return 0;
	}
	/* 无事件且未处于按压中: 仅当 PEN 物理释放时才快速返回
	 * (按下但采样失败时不清除按压, 持续重试, 避免整次按压丢失) */
	if(!g_tp_pen_down&&!(tp_dev.sta&TP_PRES_DOWN)&&(TP_PEN!=GPIO_PIN_RESET))return 0;
	g_tp_pen_down=0;//消费按下事件; 按压期间每轮持续采样
	if(TP_Read_XY2(&xt,&yt))//读数成功
	{
		if(tp)//物理坐标 (校准等特殊场合用)
		{
			tp_dev.x[0]=xt;
			tp_dev.y[0]=yt;
		}else//转换为屏幕坐标
		{
			tp_dev.x[0]=(u16)(tp_dev.xfac*xt+tp_dev.xoff);
			tp_dev.y[0]=(u16)(tp_dev.yfac*yt+tp_dev.yoff);
		}
		if((tp_dev.sta&TP_PRES_DOWN)==0)//之前没有被按下
		{
			tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//按键按下
			tp_dev.x[4]=tp_dev.x[0];//记录第一次按下时的坐标
			tp_dev.y[4]=tp_dev.y[0];
		}
	}else//读数失败 (通信异常/阈值以下): 坐标置无效, 不产生按压事件
	{
		tp_dev.x[0]=0xffff;
		tp_dev.y[0]=0xffff;
		tp_dev.sta&=~(TP_CATH_PRES|TP_PRES_DOWN);
	}
	return tp_dev.sta&TP_PRES_DOWN;//返回当前的触屏状态
#else
	u16 xt,yt;
	if(TP_PEN==GPIO_PIN_RESET)//有按键按下
	{
		if(TP_Read_XY2(&xt,&yt))//读数成功
		{
			if(tp)//物理坐标 (校准等特殊场合用)
			{
				tp_dev.x[0]=xt;
				tp_dev.y[0]=yt;
			}else//转换为屏幕坐标
			{
				tp_dev.x[0]=(u16)(tp_dev.xfac*xt+tp_dev.xoff);
				tp_dev.y[0]=(u16)(tp_dev.yfac*yt+tp_dev.yoff);
			}
			if((tp_dev.sta&TP_PRES_DOWN)==0)//之前没有被按下
			{
				tp_dev.sta=TP_PRES_DOWN|TP_CATH_PRES;//按键按下
				tp_dev.x[4]=tp_dev.x[0];//记录第一次按下时的坐标
				tp_dev.y[4]=tp_dev.y[0];
			}
		}else//读数失败 (通信异常/阈值以下): 坐标置无效, 不产生按压事件
		{
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
			tp_dev.sta&=~(TP_CATH_PRES|TP_PRES_DOWN);
		}
	}else
	{
		if(tp_dev.sta&TP_PRES_DOWN)//之前是被按下的
		{
			tp_dev.sta&=~(1<<7);//标记按键松开
		}else//之前就没有被按下
		{
			tp_dev.x[4]=0;
			tp_dev.y[4]=0;
			tp_dev.x[0]=0xffff;
			tp_dev.y[0]=0xffff;
		}
	}
	return tp_dev.sta&TP_PRES_DOWN;//返回当前的触屏状态
#endif
}

#if (TP_PEN_INT_ENABLE == 1)
/**
 * @brief  PEN 中断服务函数: 清除挂起位并记录按下/释放事件
 * @note   由 EXTI5_10_IRQHandler 调用 (驱动内已提供弱定义入口)
 */
void TP_PenIRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(TP_PEN_PIN);
	if(HAL_GPIO_ReadPin(TP_PEN_GPIO, TP_PEN_PIN)==GPIO_PIN_RESET)g_tp_pen_down=1;
	else g_tp_pen_up=1;
}

/**
 * @brief  EXTI9-5 中断入口 (强定义: 覆盖 startup 的 Default_Handler 兜底)
 * @note   startup_stm32f407xx.s 中 EXTI9_5_IRQHandler 为 .weak 指向
 *         Default_Handler (死循环), 若此处仍用 __weak, 链接器会优先解析
 *         到 startup 的兜底, 使能 EXTI 后直接死机. 故必须强定义.
 *         若需在 CubeMX 中另行使能 EXTI9-5 引脚, 会与生成的强定义冲突
 *         (链接报错), 此时应删除本定义, 在 CubeMX 生成的
 *         EXTI9_5_IRQHandler USER CODE 段调用 TP_PenIRQHandler()
 */
void EXTI9_5_IRQHandler(void)
{
	TP_PenIRQHandler();
}
#endif
/** EEPROM 校准数据基址 (占用 14 字节: SAVE_ADDR_BASE ~ SAVE_ADDR_BASE+13) */
#define SAVE_ADDR_BASE 40

/**
 * @brief  保存校准参数到 EEPROM
 * @note   板载无 24CXX EEPROM, 实际为空操作
 */
void TP_Save_Adjdata(void)
{
	s32 temp;			 
	//保存校正结果!		   							  
	temp=tp_dev.xfac*100000000;//保存x校正因素      
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE,temp,4);   
	temp=tp_dev.yfac*100000000;//保存y校正因素    
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE+4,temp,4);
	//保存x偏移量
    AT24CXX_WriteLenByte(SAVE_ADDR_BASE+8,tp_dev.xoff,2);		    
	//保存y偏移量
	AT24CXX_WriteLenByte(SAVE_ADDR_BASE+10,tp_dev.yoff,2);	
	//保存触屏类型
	AT24CXX_WriteOneByte(SAVE_ADDR_BASE+12,tp_dev.touchtype);	
	temp=0X0A;//标记校准过了
	AT24CXX_WriteOneByte(SAVE_ADDR_BASE+13,temp); 
}
/**
 * @brief  从 EEPROM 读取校准值
 * @retval 1: 成功获取校准数据; 0: 获取失败, 需重新校准
 * @note   板载无 24CXX EEPROM, 恒返回 0
 */
u8 TP_Get_Adjdata(void)
{					  
	s32 tempfac;
	tempfac=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+13);//读取标记字,看是否校准过！ 		 
	if(tempfac==0X0A)//触摸屏已经校准过了			   
	{    												 
		tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE,4);		   
		tp_dev.xfac=(float)tempfac/100000000;//得到x校准参数
		tempfac=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+4,4);			          
		tp_dev.yfac=(float)tempfac/100000000;//得到y校准参数
	    //得到x偏移量
		tp_dev.xoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+8,2);			   	  
 	    //得到y偏移量
		tp_dev.yoff=AT24CXX_ReadLenByte(SAVE_ADDR_BASE+10,2);				 	  
 		tp_dev.touchtype=AT24CXX_ReadOneByte(SAVE_ADDR_BASE+12);//读取触屏类型标记
		if(tp_dev.touchtype)//X,Y方向与屏幕相反
		{
			CMD_RDX=0X90;
			CMD_RDY=0XD0;	 
		}else				   //X,Y方向与屏幕相同
		{
			CMD_RDX=0XD0;
			CMD_RDY=0X90;	 
		}		 
		return 1;	 
	}
	return 0;
}	 
/** 校准提示字符串 (英文) */
const char* const TP_REMIND_MSG_TBL="Please use the stylus click the cross on the screen.The cross will always move until the screen adjustment is completed.";
 					  
/**
 * @brief  显示校准过程信息 (各采样点坐标与比例因子)
 * @param  x0,y0: 第 1 点采样坐标
 * @param  x1,y1: 第 2 点采样坐标
 * @param  x2,y2: 第 3 点采样坐标
 * @param  x3,y3: 第 4 点采样坐标
 * @param  fac: 比例因子 (x100, 正常范围 95~105)
 */
void TP_Adj_Info_Show(u16 x0,u16 y0,u16 x1,u16 y1,u16 x2,u16 y2,u16 x3,u16 y3,u16 fac)
{	  
	POINT_COLOR=RED;
	LCD_ShowString(40,160,lcddev.width,lcddev.height,16,"x1:");
 	LCD_ShowString(40+80,160,lcddev.width,lcddev.height,16,"y1:");
 	LCD_ShowString(40,180,lcddev.width,lcddev.height,16,"x2:");
 	LCD_ShowString(40+80,180,lcddev.width,lcddev.height,16,"y2:");
	LCD_ShowString(40,200,lcddev.width,lcddev.height,16,"x3:");
 	LCD_ShowString(40+80,200,lcddev.width,lcddev.height,16,"y3:");
	LCD_ShowString(40,220,lcddev.width,lcddev.height,16,"x4:");
 	LCD_ShowString(40+80,220,lcddev.width,lcddev.height,16,"y4:");  
 	LCD_ShowString(40,240,lcddev.width,lcddev.height,16,"fac is:");     
	LCD_ShowNum(40+24,160,x0,4,16);		//显示数值
	LCD_ShowNum(40+24+80,160,y0,4,16);	//显示数值
	LCD_ShowNum(40+24,180,x1,4,16);		//显示数值
	LCD_ShowNum(40+24+80,180,y1,4,16);	//显示数值
	LCD_ShowNum(40+24,200,x2,4,16);		//显示数值
	LCD_ShowNum(40+24+80,200,y2,4,16);	//显示数值
	LCD_ShowNum(40+24,220,x3,4,16);		//显示数值
	LCD_ShowNum(40+24+80,220,y3,4,16);	//显示数值
 	LCD_ShowNum(40+56,240,fac,3,16); 	//显示数值,该数值必须在95~105范围之内.

}
		 
/**
 * @brief  触摸屏四角校准
 *         依次点击屏幕 4 个十字校准点, 通过对边/对角线距离校验与
 *         比例因子检查, 计算 xfac/yfac/xoff/yoff 四个校准参数
 * @note   10 秒内无按压操作则自动退出; 结果仅内存生效 (无 EEPROM)
 */
void TP_Adjust(void)
{								 
	u16 pos_temp[4][2];//坐标缓存值
	u8  cnt=0;	
	u16 d1,d2;
	u32 tem1,tem2;
	double fac; 	
	u16 outtime=0;
 	cnt=0;				
	POINT_COLOR=BLUE;
	BACK_COLOR =WHITE;
	LCD_Clear(WHITE);//清屏   
	POINT_COLOR=RED;//红色 
	LCD_Clear(WHITE);//清屏 	   
	POINT_COLOR=BLACK;
	LCD_ShowString(40,40,200,200,16,TP_REMIND_MSG_TBL);//显示提示信息
	TP_Drow_Touch_Point(20,20,RED);//画点1 
	tp_dev.sta=0;//消除触发信号 
	tp_dev.xfac=0;//xfac用来标记是否校准过,所以校准之前必须清掉!以免错误	 
	while(1)//如果连续10秒钟没有按下,则自动退出
	{
		tp_dev.scan(1);//扫描物理坐标
		if((tp_dev.sta&0xc0)==TP_CATH_PRES)//按键按下了一次(此时按键松开了.)
		{	
			outtime=0;		
			tp_dev.sta&=~(1<<6);//标记按键已经被处理过了.
						   			   
			pos_temp[cnt][0]=tp_dev.x[0];
			pos_temp[cnt][1]=tp_dev.y[0];
			cnt++;	  
			switch(cnt)
			{			   
				case 1:						 
					TP_Drow_Touch_Point(20,20,WHITE);				//清除点1 
					TP_Drow_Touch_Point(lcddev.width-20,20,RED);	//画点2
					break;
				case 2:
 					TP_Drow_Touch_Point(lcddev.width-20,20,WHITE);	//清除点2
					TP_Drow_Touch_Point(20,lcddev.height-20,RED);	//画点3
					break;
				case 3:
 					TP_Drow_Touch_Point(20,lcddev.height-20,WHITE);			//清除点3
 					TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,RED);	//画点4
					break;
				case 4:	 //全部四个点已经得到
	    		    //对边相等
					tem1=abs(pos_temp[0][0]-pos_temp[1][0]);//x1-x2
					tem2=abs(pos_temp[0][1]-pos_temp[1][1]);//y1-y2
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,2的距离
					
					tem1=abs(pos_temp[2][0]-pos_temp[3][0]);//x3-x4
					tem2=abs(pos_temp[2][1]-pos_temp[3][1]);//y3-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到3,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05||d1==0||d2==0)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
 						continue;
					}
					tem1=abs(pos_temp[0][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[0][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,3的距离
					
					tem1=abs(pos_temp[1][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[1][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,4的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了
								   
					//对角线相等
					tem1=abs(pos_temp[1][0]-pos_temp[2][0]);//x1-x3
					tem2=abs(pos_temp[1][1]-pos_temp[2][1]);//y1-y3
					tem1*=tem1;
					tem2*=tem2;
					d1=sqrt(tem1+tem2);//得到1,4的距离
	
					tem1=abs(pos_temp[0][0]-pos_temp[3][0]);//x2-x4
					tem2=abs(pos_temp[0][1]-pos_temp[3][1]);//y2-y4
					tem1*=tem1;
					tem2*=tem2;
					d2=sqrt(tem1+tem2);//得到2,3的距离
					fac=(float)d1/d2;
					if(fac<0.95||fac>1.05)//不合格
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
 						TP_Adj_Info_Show(pos_temp[0][0],pos_temp[0][1],pos_temp[1][0],pos_temp[1][1],pos_temp[2][0],pos_temp[2][1],pos_temp[3][0],pos_temp[3][1],fac*100);//显示数据   
						continue;
					}//正确了
					//计算结果
					tp_dev.xfac=(float)(lcddev.width-40)/(pos_temp[1][0]-pos_temp[0][0]);//得到xfac		 
					tp_dev.xoff=(lcddev.width-tp_dev.xfac*(pos_temp[1][0]+pos_temp[0][0]))/2;//得到xoff
						  
					tp_dev.yfac=(float)(lcddev.height-40)/(pos_temp[2][1]-pos_temp[0][1]);//得到yfac
					tp_dev.yoff=(lcddev.height-tp_dev.yfac*(pos_temp[2][1]+pos_temp[0][1]))/2;//得到yoff  
					if(abs(tp_dev.xfac)>2||abs(tp_dev.yfac)>2)//触屏和预设的相反了.
					{
						cnt=0;
 				    	TP_Drow_Touch_Point(lcddev.width-20,lcddev.height-20,WHITE);	//清除点4
   	 					TP_Drow_Touch_Point(20,20,RED);								//画点1
						LCD_ShowString(40,26,lcddev.width,lcddev.height,16,"TP Need readjust!");
						tp_dev.touchtype=!tp_dev.touchtype;//修改触屏类型.
						if(tp_dev.touchtype)//X,Y方向与屏幕相反
						{
							CMD_RDX=0X90;
							CMD_RDY=0XD0;	 
						}else				   //X,Y方向与屏幕相同
						{
							CMD_RDX=0XD0;
							CMD_RDY=0X90;	 
						}			    
						continue;
					}		
					POINT_COLOR=BLUE;
					LCD_Clear(WHITE);//清屏
					LCD_ShowString(35,110,lcddev.width,lcddev.height,16,"Touch Screen Adjust OK!");//校正完成
					delay_ms(1000);
					TP_Save_Adjdata();  
 					LCD_Clear(WHITE);//清屏   
					return;//校正完成				 
			}
		}
		delay_ms(10);
		outtime++;
		if(outtime>1000)
		{
			TP_Get_Adjdata();
			break;
	 	} 
 	}
}	 
/**
 * @brief  触摸屏初始化: GPIO 配置 + 装载预存校准参数
 * @retval 0: 未进行四角校准 (使用预存校准值, 跳过校准流程)
 * @note   如触摸不准, 可调用 TP_Adjust() 重新四角校准
 */
u8 TP_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* T_MISO (DOUT): input pull-up */
	GPIO_InitStruct.Pin = TP_MISO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TP_MISO_GPIO, &GPIO_InitStruct);

#if (TP_PEN_INT_ENABLE == 1)
	/* T_PEN: EXTI 双边沿中断 (下降沿=按下, 上升沿=释放) */
	GPIO_InitStruct.Pin = TP_PEN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TP_PEN_GPIO, &GPIO_InitStruct);
	HAL_NVIC_SetPriority(TP_PEN_EXTI_IRQn, TP_PEN_EXTI_PRIO, 0);
	HAL_NVIC_EnableIRQ(TP_PEN_EXTI_IRQn);
#else
	/* T_PEN: 输入上拉, 轮询检测 */
	GPIO_InitStruct.Pin = TP_PEN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TP_PEN_GPIO, &GPIO_InitStruct);
#endif

	/* T_CS / T_SCK / T_MOSI: output */
	GPIO_InitStruct.Pin = TP_CS_PIN | TP_SCK_PIN | TP_MOSI_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(TP_CS_GPIO, &GPIO_InitStruct);

	TP_Read_XY(&tp_dev.x[0], &tp_dev.y[0]);   /* first read to init */

#if (TP_CAL_PRESET_ENABLE == 1)
	/* 使用预存校准值, 跳过四角校准 (touch_conf.h) */
	tp_dev.xfac = (float)(lcddev.width - 40) / (TP_PRESET_CAL[1][0] - TP_PRESET_CAL[0][0]);
	tp_dev.xoff = (short)((lcddev.width - tp_dev.xfac * (TP_PRESET_CAL[1][0] + TP_PRESET_CAL[0][0])) / 2);
	tp_dev.yfac = (float)(lcddev.height - 40) / (TP_PRESET_CAL[2][1] - TP_PRESET_CAL[0][1]);
	tp_dev.yoff = (short)((lcddev.height - tp_dev.yfac * (TP_PRESET_CAL[2][1] + TP_PRESET_CAL[0][1])) / 2);
	tp_dev.touchtype = 0;
	CMD_RDX = 0XD0;
	CMD_RDY = 0X90;
#else
	/* 未启用预存校准 (新屏): 调用方应执行 TP_Adjust() 四角校准 */
	tp_dev.xfac = 0; tp_dev.yfac = 0;
	tp_dev.xoff = 0; tp_dev.yoff = 0;
	tp_dev.touchtype = 0;
	CMD_RDX = 0XD0;
	CMD_RDY = 0X90;
#endif
	return 0;
}

/* ========================================================================= */
/*  手势识别状态机 (上层只收到 4 种手势事件, 不暴露 DOWN/UP)                   */
/* ========================================================================= */

/** 手势引擎内部状态 */
typedef struct {
    TouchState_t state;     /* 当前状态 */
    u16  start_x, start_y;  /* 按下起点坐标 (滑动判定基准) */
    u32  press_tick;        /* 按下时刻 (最短按压/长按计时) */
    u32  release_tick;      /* 候选释放时刻 (释放消抖计时, 0=未进入候选) */
} TouchGesture_t;

static TouchGesture_t g_gesture;

TouchState_t TP_GetGestureState(void)
{
    return g_gesture.state;
}

TouchEvent_t TP_GetGesture(void)
{
    TouchEvent_t ev = TOUCH_EVENT_NONE;
    u32 now = HAL_GetTick();
    u8  pen_down;
    u8  coord_ok;
    u16 x, y;
    int dx, dy;

    /* 原始层采样: 更新 tp_dev 坐标与状态 (PEN 中断/轮询两种模式自适应) */
    TP_Scan(0);
    pen_down = (TP_PEN == GPIO_PIN_RESET);  /* 引脚级真实按压状态 */
    x = tp_dev.x[0];
    y = tp_dev.y[0];
    /* 坐标有效性: 读数失败时 TP_Scan 会置 0xFFFF, 此时跳过移动判定 */
    coord_ok = (x < lcddev.width) && (y < lcddev.height);

    switch (g_gesture.state)
    {
    case TOUCH_STATE_IDLE:
        if (pen_down && coord_ok) {
            g_gesture.start_x = x;
            g_gesture.start_y = y;
            g_gesture.press_tick = now;
            g_gesture.release_tick = 0;
            g_gesture.state = TOUCH_STATE_PRESSING;
        }
        break;

    case TOUCH_STATE_PRESSING:
        if (!pen_down) {
            /* 最短按压时间内松开: 误触/抖动, 忽略 (不产生事件) */
            g_gesture.state = TOUCH_STATE_IDLE;
        } else if (now - g_gesture.press_tick >= TOUCH_MIN_PRESS_TIME) {
            /* 确认为有效按压 */
            g_gesture.state = TOUCH_STATE_PRESSED;
        }
        break;

    case TOUCH_STATE_PRESSED:
        if (!pen_down) {
            /* 释放消抖: 保持松开超过 TOUCH_RELEASE_DEBOUNCE 才确认释放 */
            if (g_gesture.release_tick == 0) g_gesture.release_tick = now;
            if (now - g_gesture.release_tick >= TOUCH_RELEASE_DEBOUNCE) {
                g_gesture.release_tick = 0;
                g_gesture.state = TOUCH_STATE_IDLE;
                ev = TOUCH_EVENT_SINGLE_CLICK;   /* 单击: 松开确认后立即上报 */
            }
        } else {
            g_gesture.release_tick = 0;   /* PEN 弹跳回到按下, 取消候选释放 */
            if (coord_ok) {
                /* 移动优先于长按 */
                dx = (int)x - (int)g_gesture.start_x;
                dy = (int)y - (int)g_gesture.start_y;
                if (dx * dx + dy * dy >= TOUCH_SWIPE_THRESHOLD * TOUCH_SWIPE_THRESHOLD) {
                    g_gesture.state = TOUCH_STATE_SWIPE;
                    ev = TOUCH_EVENT_SWIPE;
                } else if (now - g_gesture.press_tick >= TOUCH_LONG_PRESS_TIME) {
                    g_gesture.state = TOUCH_STATE_LONG_PRESS;
                    ev = TOUCH_EVENT_LONG_PRESS;
                }
            }
        }
        break;

    case TOUCH_STATE_LONG_PRESS:
    case TOUCH_STATE_SWIPE:
        /* 终态: 松开后回到 IDLE, 不产生单击 */
        if (!pen_down) {
            g_gesture.state = TOUCH_STATE_IDLE;
        }
        break;

    default:
        g_gesture.state = TOUCH_STATE_IDLE;
        break;
    }

    return ev;
}

