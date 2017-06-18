/** 项目整体架构（11号数字口损坏）
*	1、电机驱动部分：A电机（en--5 A ,B--2,3） B(en--6 A,B--4,7)
2、超声波测距（Trig --控制口--8  echo--数据脉冲口--9）
3、红外遥控（信号线-- 10）
4、蓝牙遥控（TXD---13 ，RXD---12）
5、串口（前期调试  0口  1口）
6、避障模块     
版本信息：  此版本从10版本而来：
zdm  2015/6/7 21:00 开始修改    
===========================上期版本修复信息======================
修改记录1 - 转弯时间太长
修改记录2 - 加入及时暂停，引入谁转弯谁调用pause 和谁转舵机谁恢复 
修改记录3 - 修改探测上界
修改记录4 - 较小左向纠正模糊边界			


% 放弃%修改记录5 - 改变转弯纠正效果--使之能连续修正
修改记录6 - 修改舵机测距不准确的问题  
修改记录7 - 修改autoRun函数 先测右距离 再测前距离 
修改记录8 - 左转时加一定距离
修复记录9 -  修复前进距离不够问题
修复记录10 - 待修复问题-->走直线问题
======================本版本11.0修改息 zdm 2015/6/7 21:03===========================
修复记录1 - 更正在窄巷中的处理函数  
修复记录2 - 在autorun中加入斜左放的距离判断  21:55 修改完成
版本11.0 信息：  已经完成 上述1、2的修改 
				 发现问题---走时会撞墙--可能原因-->避障下界太小
zdm 2015/6/8 11:34 添加
=====================本版本12.0修改信息 zdm 2015/6/8 11:52==============================
修改记录1 - 加入右距离太小检测处理 
！！！！注意：13-15版本废弃----除发行版外 加入距离太小检测
 此版本信息:
			修改记录1：修复loop中舵机恢复buge
			修复记录2：减小边界探索中的小车速度--->开始：car_auto 结束：stop
			修复记录3：加入太小修正
*/

#include <IRremote.h>
#define GET_DIS1 0xFF30CF						//红外线编码
#define FORWORD2 0xFF18E7
#define RIGHT4 0xFF10EF
#define LEFT6 0xFF5AA5
#define BACK8 0xFF4AB5
#define REPEAT 0xFFFFFFFF
#define AUTORUN3 0xFF7A85
#define STOP5 0xFF38C7
#define TURNLEFT7 0xFF42BD
#define TURNRIGHT9 0xFF52AD
#define  DIS_ERR 100					//异常信号的定义

#define TIME_CAR_LENGTH 30				//前进车身长所需时间
#define TIME_CAR_RIGHT  6				//车右转 所需时间
#define TIME_CAR_LEFT   6				//车左转 所需时间
#define TIME_DUO_TURN   13

#define  ANGLE_SERV_RIGHT 0				//舵机右转角度
#define  ANGLE_SERV_LEFT  180			//舵机左转角度
#define  ANGLE_SERV_DEFAULT 80			//舵机默认角度
#define  ANGLE_SERV_LEFT2 100			//舵机转斜左方角度
#define  LENGTH_TOO_FAR  15				//最小需要探索的距离
#define  LENGTH_TOO_NEAR   8           //最大需要需要躲避的距离10

int pinI1 = 2; //定义I1接口
int pinI2 = 3; //定义I2接口
int speedpin = 5; //定义EA(PWM调速)接口
int pinI3 = 4; //定义I3接口
int pinI4 = 7; //定义I4接口
int speedpin1 = 6; //定义EB(PWM调速)接口
const int TripIin = 8;
const int EchoPin = 9;

//定义时间变量
unsigned long last_time;
unsigned long current_time;
unsigned char loopVar;
unsigned char forward_flag;

float distance;						//距离
float dis_pre_Turn;					//转弯之前的调整距离		
float dis_result;
float dis_temp;//定义红外接收变量
const int REV_PIN = 10;
IRrecv recv(REV_PIN);
decode_results result;
unsigned int last_state;
unsigned int BT_maskCode=0;
unsigned int IR_maskCode=0;									//状态变量
unsigned char mode;					//低四位 前进 2 后退 1 左转 3 右转 4  测距 5  高四位为0   
//自动循迹  高四位为1 //虚拟串口
//SoftwareSerial myserial(12,13);			//rx ,tx
int servopin=13;					//舵机信号输出口变量
int pulsewidth;						//脉冲宽度
unsigned char autoflag;				//自动巡航变量
unsigned char isResetServ;			//是否需要 重新调整舵机
unsigned char direction;			//1 2 3 4 表示前 后 左 右
unsigned char last_BT_state;		
unsigned char BT_state;
unsigned int  IR_state;

void setup() {
	// 初始化端口
	Serial.begin(9600);					//初始化电机口										//	myserial.begin(9600);	
	pinMode(pinI1, OUTPUT);
	pinMode(pinI2, OUTPUT);
	pinMode(speedpin, OUTPUT);
	pinMode(pinI3, OUTPUT);
	pinMode(pinI4, OUTPUT);
	pinMode(speedpin1, OUTPUT);	
	analogWrite(speedpin, 240);	//初始化电机转速PWM输出
	analogWrite(speedpin1, 255);
	pinMode(TripIin, OUTPUT);		//超声波测距
	pinMode(EchoPin, INPUT);
	pinMode(servopin,OUTPUT);		//舵机初始化	
	Serial.println("Hello");
	recv.enableIRIn();
	mode=0;							//默认停止
	autoflag=2;
	//	myserial.listen();
	last_BT_state=0xff;				//最大值
	serV_default();
	IR_maskCode=STOP5;
	BT_maskCode='5';
	forward_flag=0;
	isResetServ=false;
}

float get_distence()
{
	digitalWrite(TripIin, LOW);
	delayMicroseconds(2);
	digitalWrite(TripIin, HIGH);
	delayMicroseconds(10);
	digitalWrite(TripIin, LOW);
	distance = pulseIn(EchoPin, HIGH) / 58.00;
	Serial.print(distance);
	Serial.println("cm");
	//delay(100);
	return distance;
}

void car_Left()					//前进 2
{
	Serial.println("car_Left");
	mode=(mode&0xf0)|0x03;
	digitalWrite(pinI4, LOW); //使直流电机（右）逆时针转
	digitalWrite(pinI3, HIGH);
	digitalWrite(pinI1, LOW); //使直流电机（左）顺时针转
	digitalWrite(pinI2, HIGH);
}
void car_Right()			
{
	Serial.println("car_Right");
	mode=(mode&0xf0)|0x04;		
	digitalWrite(pinI4, HIGH); //使直流电机（右）顺时针转
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, HIGH); //使直流电机（左）逆时针转
	digitalWrite(pinI2, LOW);
}

void car_back()
{
	Serial.println("car_back");
	mode=(mode&0xf0)|0x01;
	digitalWrite(pinI4, LOW); //使直流电机（右）逆时针转
	digitalWrite(pinI3, HIGH);
	digitalWrite(pinI1, HIGH); //使直流电机（左）逆时针转
	digitalWrite(pinI2, LOW);
}

void car_Stop()
{
	Serial.println("stop");
	if ((mode&0xf0)==0x10)		//处于边界探索状态
	{
		analogWrite(speedpin, 240);	//恢复正常速度
		analogWrite(speedpin1, 255);
	}
	mode=0;
	digitalWrite(pinI4, LOW); //使直流电机（右）逆时针转
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW); //使直流电机（左）逆时针转
	digitalWrite(pinI2, LOW);
}
void car_Pause()
{
	Serial.println("pause");
	digitalWrite(pinI4, LOW); //使直流电机（右）逆时针转
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW); //使直流电机（左）逆时针转
	digitalWrite(pinI2, LOW);
}


float get_Rightdis()			//舵机右转测距---执行时间---最少为：13*20=260ms
{
	for (loopVar=0;loopVar<TIME_DUO_TURN;loopVar++)				//舵机旋转 100ms
	{
		servopulse(ANGLE_SERV_RIGHT);
		scanSingCtrl();
		if (mode==0)
		{
			return 0;
		}
	}
	Serial.print("right-dis:");
	return get_distence();
}

float get_Leftdis()			//舵机右转测距---执行时间---最少为：10*20+5*10=250ms
{	
	for (loopVar=0;loopVar<TIME_DUO_TURN;loopVar++)				//舵机旋转 100ms
	{
		servopulse(ANGLE_SERV_LEFT);
		scanSingCtrl();
		if (mode==0)
		{
			return 0;
		}
	}
	Serial.print("right-dis:");
	return get_distence();
}

float get_midLeftDis()
{
	for (loopVar=0;loopVar<3;loopVar++)				//舵机旋转 100ms
	{
		servopulse(ANGLE_SERV_LEFT2);				
		scanSingCtrl();
		if (mode==0)
		{
			return 0;
		}
	}
	Serial.print("midLeft-dis:");
	return get_distence();
}

void serV_default()
{
	
	for (loopVar=0;loopVar<TIME_DUO_TURN;loopVar++)				//15*20=200ms
	{
		servopulse(ANGLE_SERV_DEFAULT);
		
	}
}

void car_turnLeft()			//原先转弯延时时间300,若要转弯测距，则必须在调用之前转舵机  	//6*5*10 至少300ms延时  测距6次
{
	Serial.println("---***--car_turnLeft start");
	car_Left();		
	for (unsigned char i=0;i<TIME_CAR_LEFT;i++)				
	{	if(get_distence()<=3)		//转弯失败
		{	Serial.println("!!! car_turnLeft failed");
			car_Right_Corret();		//修正
			return;
		}		
		for (loopVar=0;loopVar<5;loopVar++)	//50ms延时
		{
			delay(10);
			if (mode==0)
			{	car_Pause();
				return ;
			}
		}
	}
	car_Pause();
	Serial.println("---***--car_turnLeft succeed");						//小车停止转弯 进入下一次循环
}

unsigned char  car_turnRight()										//小车右转90度
{
	Serial.println("---***--car_turnRight start");
	car_Right();
	for (unsigned char i=0;i<TIME_CAR_RIGHT;i++)		//6*5*10 至少300ms延时  测距6次
	{	if(get_distence()<=3)			//转弯失败
	       {	Serial.println("!!! car_turnRight failed");
	            car_left_Corret();				//修正
	            return 3;
	      }		
			for (loopVar=0;loopVar<5;loopVar++)	//50ms延时
			{
				delay(10);
				if (mode==0)
				{	car_Pause();
				    return 0;
				}
			}
	}//250ms延时
	car_Pause();
	return 0;			//正常 转弯结束
	Serial.println("---***--car_turnRight succed");
}
/*
  说明:窄巷中的处理函数--在此之前舵机方向向前
  小车减速
	  步骤1-->设置向前运动
	  步骤2-->探测到右向空白-->结束；
	  步骤3-->设置电机向前
	  步骤4-->检测到前方距离小于8或左前方距离小于等于3-->后退函数
	  
//保证右半部分在本次探测，而左半部分在后退中探测。
*/	
void car_Narrow()				
{   bool isRun=true;
	car_Forword();				//--1
	while(isRun){
		if (get_Rightdis()>=8)		//--2 右方空白-->结束
		{	if(mode==0) { isResetServ=true;return;}
				car_Pause();
				return;
			}
			if(mode==0) { isResetServ=true;	return;}
			serV_default();			//--3  
			if(mode==0) { isResetServ=true;return;}
			if (get_distence()<=8||get_midLeftDis()<=3)  //--4
			{
				car_turn_Back();
				return;
			}
		}
}

void car_turn_Back()							//后退--左转
{ 	Serial.println("++++ car_turn_Back start");
	car_back();
	if (get_Leftdis()>=5||get_Rightdis()>=5)
	{	Serial.println("++++ car_turn_Back succeed");
		car_turnLeft();
		car_Pause();
		return;
	}

}
void car_left_Corret()								//左向矫正--
{	Serial.println("++++ car_left_Corret start");
	car_Left();										//小车左转
	for (unsigned char i=0;i<5;i++)						//6次测距共300ms
	{
		dis_result= get_distence();
		if (abs(dis_result-dis_pre_Turn)<=5)		//矫正成功
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause();						//进入下一次循环
			return ;								//完成
		}
		for (loopVar=0;loopVar<5;loopVar++)			//50毫秒延时
		{
			delay(10);
			scanSingCtrl();
			if (mode==0)
			{	car_Pause();
			return ;
			}
		}
	}	
	car_Pause();
	Serial.println("++++ car_left_Corret failed");
}

void car_left_Corret_run()						//注意 每次校正 不超过18度 3*6次
{	Serial.println("++++ car_left_Corret start");
	car_Left();										//小车左转
	for (loopVar=0;loopVar<6;loopVar++)					//6次测距共60ms
	{		
		delay(10);									//10ms 大概3度									
		scanSingCtrl();
		if (mode==0)
		{   car_Pause(); 
			return ;
		}
		dis_result= get_distence();
		if (abs(dis_result-8)<=3)		//矫正成功
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause(); 						//进入下一次循环
			return ;								//完成
		}
	}	
	car_Pause();
	Serial.println("++++ car_left_Corret failed");
}

void car_Right_Corret()							//右向矫正--
{	Serial.println("++++ car_Right_Corret start");
	car_Right();										//小车左转
	for (unsigned char i=0;i<TIME_CAR_RIGHT;i++)					//6次测距共300ms
	{
		dis_result= get_distence();
		if (abs(dis_result-dis_pre_Turn)<=5)		//矫正成功
		{	Serial.println("++++ car_Right_Corret suceed");										//成功
			car_turn_Back();						//汽车后退 保证跳过此节点 右探索
			return ;								//完成
		}
			for (loopVar=0;loopVar<5;loopVar++)			//50毫秒延时   15度
			{
				delay(10);
				scanSingCtrl();
				if (mode==0)
				{	 car_Pause();
				return ;
				}
			}
	}
	car_Pause();
	 Serial.println("++++ car_Right_Corret failed");
}

void car_Right_Corret_run()							// 18度
{	Serial.println("++++ car_left_Corret start");
	car_Right();										    //小车左转
	for (loopVar=0;loopVar<6;loopVar++)					//6次测距共60ms
	{		
		delay(10);									       //10ms 大概3度									
		scanSingCtrl();
		if (mode==0)
		{   car_Pause(); 
			return ;
		}
		dis_result= get_distence();
		if (abs(dis_result-8)<=5)		//矫正成功
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause(); 						//进入下一次循环
			return ;								//完成
		}
	}	
	car_Pause();
    Serial.println("++++ car_left_Corret failed");
}

void car_Forword()
{
	Serial.println("car_Forword");
	mode=(mode&0xf0)|0x02;
	digitalWrite(pinI4, HIGH);				 //使直流电机（右）顺时针转
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW);					//使直流电机（左）顺时针转
	digitalWrite(pinI2, HIGH);
}
void car_AutoRun()
{
	Serial.println("======auto Run=======");
	analogWrite(speedpin, 140);	//初始化电机转速PWM输出
	analogWrite(speedpin1, 155);
	dis_pre_Turn=8;
	mode=mode|0x10;							//设置小车状态
	float mindis=0;								//初始化程序  判断前后左右的距离  寻找最近的边界
	float Auototemp;
	serV_default();							//300ms
	Serial.print("farward-dis:");
	mindis=get_distence();
	delay(50);
	direction=1;
	if (mindis>(Auototemp=get_Rightdis()))		//右方比前方小
	{	if(mode==0) { isResetServ=true;return;}
		 direction=4;
		mindis=Auototemp;
	}
	if (mindis>(Auototemp=get_Leftdis()))		//左方小
	{	if(mode==0) { isResetServ=true;return;}
		direction=3;
		mindis=Auototemp;
	}
	car_turnLeft();
	if(mode==0) { isResetServ=true;return;}
	Serial.print("back-dis:");
	if (mindis>(Auototemp=get_Leftdis()))		//后方小
	{	if(mode==0) { isResetServ=true;return;}
		direction=2;
		mindis=Auototemp;
	}													//执行完毕后舵机朝后
	serV_default();										//纠正舵机方向与小车方向一致
	Serial.print("=====the Best direct:");
	Serial.print(direction);
	switch (direction)									//小车方向调整
	{
		case 1:	Serial.println("--forword");car_turnRight();break;						//前方
		case 3:	Serial.println("--left");;break;										//左方
		case 4:	Serial.println("--Right");car_turnLeft();car_turnLeft();break;			//右方
		case 2:	Serial.println("--bank");car_turnLeft();break;						      //后方
		default:break;
	}
	
	while(get_distence()>=8.0)							//小车一直前进
	{
		action(2);
		scanSingCtrl();
		if (mode==0) return;							//返回
		delay(50);										//一直前进
	}													//调整完毕
	
	car_turnLeft();										//调整为边界状态
	Serial.println("eeeeeeeeeeeeeeeend");
	if(mode==0) { return;}	
	autoAction();
									//开始边界检测			
}

void autoAction()
{	Serial.println("--------autoAction---------");
	dis_temp=get_Rightdis();						    //260ms
	if (dis_temp>=6&&dis_temp<=10)					//正常
	{	Serial.println(" *** forward normal ***");
	}else if (dis_temp<6)							   //太近调整
	{	if (dis_temp<3)
		{
	    	Serial.println(" *** too near***  to back");
	    	back_CanLenght();
			if(mode==0) { isResetServ=true;return;}
			return;
		}else
		{
			Serial.println(" *** too near***  to left");
			dis_pre_Turn=8;
	     	car_left_Corret_run();
		}
	}else if (dis_temp>10&&dis_temp<=LENGTH_TOO_FAR)	//太远调整
	{	Serial.println(" *** too far***  to right");
		dis_pre_Turn=8;
		car_Right_Corret_run();
	}else if (dis_temp>LENGTH_TOO_FAR&&dis_temp<500)
	{	Serial.println(" !!!  start trun Right !!!");
		car_Pause();
		car_To_right();
	}else if(dis_temp>=500){
		Serial.println(" *** too near***  to error");
		back_CanLenght();
		if(mode==0) { isResetServ=true;return;}
			return;
	}
	if(mode==0) { isResetServ=true;return;}
	serV_default();										//250ms
	action(2);	
	Serial.print("farward-dis:");				
	dis_temp=get_distence();					//判断前方
	if (dis_temp<=10)
	{	if (dis_temp<3)
		{
	    	Serial.println(" ***forward too near***  to back");
		    back_CanLenght();
			return;	
		}else
		{
			Serial.println("forward --to-- left");
			car_Pause();
			dis_pre_Turn=get_Leftdis();				//转舵机
			if(mode==0) { isResetServ=true;return;}
			car_turnLeft();							//避障转弯
			if(mode==0) { isResetServ=true;return;}
			forward_CanLenght();
			if(mode==0) { isResetServ=true;return;}
			serV_default();							//舵机恢复
			return ;
		}
	}else if(dis_temp>=500){
		Serial.println(" *** too near***  to error");
		back_CanLenght();
		if(mode==0) { isResetServ=true;return;}
		return;
	}
	dis_temp=get_midLeftDis();					//判断前方
	if (dis_temp<=3||dis_temp>500)				//太近-->执行之前舵机向默认方向
	{	//后退-->左转-->进入下一次循环;
		back_CanLenght();			//后退必须成功
		if(mode==0) { isResetServ=true;return;}
		car_turnLeft();				//必须成功
		if(mode==0) { isResetServ=true;return;}
		serV_default();
	}
}

/*
	  步骤1--旋转舵机--向前
	  步骤2--前进车长度--避免卡住车身--返回是否前进成功标志
	  步骤3--旋转舵机--向右-->设置修正值
	  步骤4--向右转弯--并返回是否转弯成功成功标记-->失败则直接返回
	  步骤5--旋转舵机--向前
	  步骤6--转完后前进车长度--避免进入死循环
	  步骤7--由标志位判断是否为窄巷
*/
void car_To_right()
{	forward_flag=0;
	serV_default();						// --1		
	if (forward_CanLenght()==DIS_ERR) {    //--2
		forward_flag=1; Serial.println(" !!! forward failed!!!");}
	if (mode==0) return;
	dis_pre_Turn=get_Rightdis();		    //--3
	
	if(car_turnRight()!=0)					//--4   0为转弯成功 不为0为转弯失败 --> 设置舵机恢复使能     
	{ isResetServ=true; return ;}
	
 	if(mode==0) { isResetServ=true;return;}
	serV_default();					   //--5
	
	if(forward_CanLenght()==DIS_ERR) { return ;}	//--6  前进失败直接返回                        
 	if(mode==0) { isResetServ=true;return;}
	if (forward_flag==1)					//--7
	{ car_Narrow();}
}

unsigned char forward_CanLenght()
{	car_Forword();
	for (loopVar=0;loopVar<TIME_CAR_LENGTH;loopVar++)		//延时：30*10=300ms
	{
		delay(10);
		scanSingCtrl();
		if (mode==0||get_distence()<=5)	//停止条件
		{	car_Pause();
			return DIS_ERR;				//前进失败
		}
	}
	car_Pause();
	return 1;
}

void back_CanLenght()
{	car_back();
	for (loopVar=0;loopVar<TIME_CAR_LENGTH;loopVar++)		//延时：30*10=300ms
	{
		delay(10);
		scanSingCtrl();
		if (mode==0||get_distence()>5)	//停止条件
		{	car_Pause();
	    	return ;				//前进失败
		}
	}
	car_Pause();
	return ;
}

float delayWithScan(int time)
{
	for (loopVar=0;loopVar<time;loopVar++)
	{
		delay(10);
		scanSingCtrl();
		if (mode==0)
		{
			return 0;
		}
	}
}

void servopulse(int myangle)					//产生指定角度的PWM并发送
{
	pulsewidth=(myangle*11)+500;				//将角度转化为500-2480的脉宽值
	digitalWrite(servopin,HIGH);				//将舵机接口电平至高
	delayMicroseconds(pulsewidth);				//延时脉宽值的微秒数
	digitalWrite(servopin,LOW);					//将舵机接口电平至低
	delay(20-pulsewidth/1000);						//延时 使舵机转动结束
}

void action( int data )
{
	switch (data)
	{
	case 1 : get_distence(); break;
	case 2 : car_Forword(); break;
	case 3 : car_AutoRun();break;
	case 5 : car_Stop();break;
	case 8 : car_back(); break;
	case 4 : car_Left(); break;
	case 6: car_Right(); break;
	case 7: car_turnLeft();break;
	case 9:car_turnRight();break;
	default :Serial.print("**********error Msg*********:");
		Serial.println(data);break;
	}
	
}

unsigned int IRtoAction(unsigned int state)		//转化红外线命令，无效信号为-1,且判断是否是有效信号 
{
	recv.resume();
	switch (state)
	{
    case GET_DIS1 : return 1; break;
    case FORWORD2 : return 2; break;
    case BACK8 : return 8; break;
    case RIGHT4 : return 4; break;
    case LEFT6: return 6; break;
	case AUTORUN3 :return 3;break;
	case STOP5: return 5;break;
	case TURNLEFT7:return 7;break;
	case TURNRIGHT9 :return 9;break;
    default : Serial.println("**********IR error Msg*********");return -1;			//无效信号
	}	
}

void scan()
{
	if (recv.decode(&result))						//有有效的信号输入
	{ 		
		Serial.print("----recevice a IR MSG:");
        Serial.println(result.value,HEX);
		if (result.value != 0xFFFFFFFF)				// 不是重复信号
		{
			last_state=result.value;
		}
		IR_state=IRtoAction(last_state);
		if (IR_state!=-1)			//有效新号 包括上一次 与本次
		{
			last_time=millis();	 
			action(IR_state);
		}
		recv.resume();				//进行下一次接收
	}
	//蓝牙接收程序
	if(Serial.available()>0)		//有信号输入，输入的全部为有效信号 
	{
		//        Serial.print(" Bluetooth Message ");
		BT_state= Serial.read();
		//        Serial.print(BT_state-'0');
		last_time=millis();
		action(BT_state-'0');
	}
	
}

void scanSingCtrl()
{
	if (recv.decode(&result))						//有有效的信号输入
	{ 		
		Serial.print("+++++++recevice a IR MSG:");
        Serial.println(result.value,HEX);
		last_state=result.value;
		if (last_state==IR_maskCode)
		{
			Serial.println("~~~~~~~~~~~~~~stop~~~~~~~~");
			action(IRtoAction(last_state));
		}
		recv.resume();				//进行下一次接收
	}
	//蓝牙接收程序
	if(Serial.available()>0)		//有信号输入，输入的全部为有效信号 
	{	last_BT_state=Serial.read();
	if (last_BT_state==BT_maskCode)
	{
		Serial.println("~~~~~~~~~~~~~~stop~~~~~~~~");
		action(last_BT_state-'0');
	}	
	}	
}

void loop() {
	current_time=millis();
	if ((mode==1||mode==2))							//前进后退
	{
		if (current_time-last_time>500)
		{
			car_Stop();
		}
		if (((mode-1)&&get_distence()<10.0))		//距离太短避障
		{
			car_Stop();
		}
	}else if ((mode==3||mode==4))			//左转右转
	{
		if (current_time-last_time>80)
		{
			car_Stop();
		}
	}else if ((mode&0xf0)==0x10)			//自动寻找边界
	{
		autoAction();
	}else if(mode!=0)						//异常模式
	{
		Serial.print("exception State:");
		Serial.println(mode);
	}
	scan();
	if ((mode&0xf0)!=0x10&&isResetServ)
	{	isResetServ=false;
		serV_default();					//调整舵机转向
	}
}

/*
	buge 修复记录 23:01：
	1、修改前方避障距离为8
	2、修复判断距离    5为下界    8位核心   11为上界   11-15为上界调整距离 模糊边界为两厘米
	3、修正超声波延时   
=======================================================
	buge 修复记录 12:51
	修复左右转弯      修复
*/