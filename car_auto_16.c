/** ��Ŀ����ܹ���11�����ֿ��𻵣�
*	1������������֣�A�����en--5 A ,B--2,3�� B(en--6 A,B--4,7)
2����������ࣨTrig --���ƿ�--8  echo--���������--9��
3������ң�أ��ź���-- 10��
4������ң�أ�TXD---13 ��RXD---12��
5�����ڣ�ǰ�ڵ���  0��  1�ڣ�
6������ģ��     
�汾��Ϣ��  �˰汾��10�汾������
zdm  2015/6/7 21:00 ��ʼ�޸�    
===========================���ڰ汾�޸���Ϣ======================
�޸ļ�¼1 - ת��ʱ��̫��
�޸ļ�¼2 - ���뼰ʱ��ͣ������˭ת��˭����pause ��˭ת���˭�ָ� 
�޸ļ�¼3 - �޸�̽���Ͻ�
�޸ļ�¼4 - ��С�������ģ���߽�			


% ����%�޸ļ�¼5 - �ı�ת�����Ч��--ʹ֮����������
�޸ļ�¼6 - �޸Ķ����಻׼ȷ������  
�޸ļ�¼7 - �޸�autoRun���� �Ȳ��Ҿ��� �ٲ�ǰ���� 
�޸ļ�¼8 - ��תʱ��һ������
�޸���¼9 -  �޸�ǰ�����벻������
�޸���¼10 - ���޸�����-->��ֱ������
======================���汾11.0�޸�Ϣ zdm 2015/6/7 21:03===========================
�޸���¼1 - ������խ���еĴ�����  
�޸���¼2 - ��autorun�м���б��ŵľ����ж�  21:55 �޸����
�汾11.0 ��Ϣ��  �Ѿ���� ����1��2���޸� 
				 ��������---��ʱ��ײǽ--����ԭ��-->�����½�̫С
zdm 2015/6/8 11:34 ���
=====================���汾12.0�޸���Ϣ zdm 2015/6/8 11:52==============================
�޸ļ�¼1 - �����Ҿ���̫С��⴦�� 
��������ע�⣺13-15�汾����----�����а��� �������̫С���
 �˰汾��Ϣ:
			�޸ļ�¼1���޸�loop�ж���ָ�buge
			�޸���¼2����С�߽�̽���е�С���ٶ�--->��ʼ��car_auto ������stop
			�޸���¼3������̫С����
*/

#include <IRremote.h>
#define GET_DIS1 0xFF30CF						//�����߱���
#define FORWORD2 0xFF18E7
#define RIGHT4 0xFF10EF
#define LEFT6 0xFF5AA5
#define BACK8 0xFF4AB5
#define REPEAT 0xFFFFFFFF
#define AUTORUN3 0xFF7A85
#define STOP5 0xFF38C7
#define TURNLEFT7 0xFF42BD
#define TURNRIGHT9 0xFF52AD
#define  DIS_ERR 100					//�쳣�źŵĶ���

#define TIME_CAR_LENGTH 30				//ǰ����������ʱ��
#define TIME_CAR_RIGHT  6				//����ת ����ʱ��
#define TIME_CAR_LEFT   6				//����ת ����ʱ��
#define TIME_DUO_TURN   13

#define  ANGLE_SERV_RIGHT 0				//�����ת�Ƕ�
#define  ANGLE_SERV_LEFT  180			//�����ת�Ƕ�
#define  ANGLE_SERV_DEFAULT 80			//���Ĭ�ϽǶ�
#define  ANGLE_SERV_LEFT2 100			//���תб�󷽽Ƕ�
#define  LENGTH_TOO_FAR  15				//��С��Ҫ̽���ľ���
#define  LENGTH_TOO_NEAR   8           //�����Ҫ��Ҫ��ܵľ���10

int pinI1 = 2; //����I1�ӿ�
int pinI2 = 3; //����I2�ӿ�
int speedpin = 5; //����EA(PWM����)�ӿ�
int pinI3 = 4; //����I3�ӿ�
int pinI4 = 7; //����I4�ӿ�
int speedpin1 = 6; //����EB(PWM����)�ӿ�
const int TripIin = 8;
const int EchoPin = 9;

//����ʱ�����
unsigned long last_time;
unsigned long current_time;
unsigned char loopVar;
unsigned char forward_flag;

float distance;						//����
float dis_pre_Turn;					//ת��֮ǰ�ĵ�������		
float dis_result;
float dis_temp;//���������ձ���
const int REV_PIN = 10;
IRrecv recv(REV_PIN);
decode_results result;
unsigned int last_state;
unsigned int BT_maskCode=0;
unsigned int IR_maskCode=0;									//״̬����
unsigned char mode;					//����λ ǰ�� 2 ���� 1 ��ת 3 ��ת 4  ��� 5  ����λΪ0   
//�Զ�ѭ��  ����λΪ1 //���⴮��
//SoftwareSerial myserial(12,13);			//rx ,tx
int servopin=13;					//����ź�����ڱ���
int pulsewidth;						//������
unsigned char autoflag;				//�Զ�Ѳ������
unsigned char isResetServ;			//�Ƿ���Ҫ ���µ������
unsigned char direction;			//1 2 3 4 ��ʾǰ �� �� ��
unsigned char last_BT_state;		
unsigned char BT_state;
unsigned int  IR_state;

void setup() {
	// ��ʼ���˿�
	Serial.begin(9600);					//��ʼ�������										//	myserial.begin(9600);	
	pinMode(pinI1, OUTPUT);
	pinMode(pinI2, OUTPUT);
	pinMode(speedpin, OUTPUT);
	pinMode(pinI3, OUTPUT);
	pinMode(pinI4, OUTPUT);
	pinMode(speedpin1, OUTPUT);	
	analogWrite(speedpin, 240);	//��ʼ�����ת��PWM���
	analogWrite(speedpin1, 255);
	pinMode(TripIin, OUTPUT);		//���������
	pinMode(EchoPin, INPUT);
	pinMode(servopin,OUTPUT);		//�����ʼ��	
	Serial.println("Hello");
	recv.enableIRIn();
	mode=0;							//Ĭ��ֹͣ
	autoflag=2;
	//	myserial.listen();
	last_BT_state=0xff;				//���ֵ
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

void car_Left()					//ǰ�� 2
{
	Serial.println("car_Left");
	mode=(mode&0xf0)|0x03;
	digitalWrite(pinI4, LOW); //ʹֱ��������ң���ʱ��ת
	digitalWrite(pinI3, HIGH);
	digitalWrite(pinI1, LOW); //ʹֱ���������˳ʱ��ת
	digitalWrite(pinI2, HIGH);
}
void car_Right()			
{
	Serial.println("car_Right");
	mode=(mode&0xf0)|0x04;		
	digitalWrite(pinI4, HIGH); //ʹֱ��������ң�˳ʱ��ת
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, HIGH); //ʹֱ�����������ʱ��ת
	digitalWrite(pinI2, LOW);
}

void car_back()
{
	Serial.println("car_back");
	mode=(mode&0xf0)|0x01;
	digitalWrite(pinI4, LOW); //ʹֱ��������ң���ʱ��ת
	digitalWrite(pinI3, HIGH);
	digitalWrite(pinI1, HIGH); //ʹֱ�����������ʱ��ת
	digitalWrite(pinI2, LOW);
}

void car_Stop()
{
	Serial.println("stop");
	if ((mode&0xf0)==0x10)		//���ڱ߽�̽��״̬
	{
		analogWrite(speedpin, 240);	//�ָ������ٶ�
		analogWrite(speedpin1, 255);
	}
	mode=0;
	digitalWrite(pinI4, LOW); //ʹֱ��������ң���ʱ��ת
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW); //ʹֱ�����������ʱ��ת
	digitalWrite(pinI2, LOW);
}
void car_Pause()
{
	Serial.println("pause");
	digitalWrite(pinI4, LOW); //ʹֱ��������ң���ʱ��ת
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW); //ʹֱ�����������ʱ��ת
	digitalWrite(pinI2, LOW);
}


float get_Rightdis()			//�����ת���---ִ��ʱ��---����Ϊ��13*20=260ms
{
	for (loopVar=0;loopVar<TIME_DUO_TURN;loopVar++)				//�����ת 100ms
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

float get_Leftdis()			//�����ת���---ִ��ʱ��---����Ϊ��10*20+5*10=250ms
{	
	for (loopVar=0;loopVar<TIME_DUO_TURN;loopVar++)				//�����ת 100ms
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
	for (loopVar=0;loopVar<3;loopVar++)				//�����ת 100ms
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

void car_turnLeft()			//ԭ��ת����ʱʱ��300,��Ҫת���࣬������ڵ���֮ǰת���  	//6*5*10 ����300ms��ʱ  ���6��
{
	Serial.println("---***--car_turnLeft start");
	car_Left();		
	for (unsigned char i=0;i<TIME_CAR_LEFT;i++)				
	{	if(get_distence()<=3)		//ת��ʧ��
		{	Serial.println("!!! car_turnLeft failed");
			car_Right_Corret();		//����
			return;
		}		
		for (loopVar=0;loopVar<5;loopVar++)	//50ms��ʱ
		{
			delay(10);
			if (mode==0)
			{	car_Pause();
				return ;
			}
		}
	}
	car_Pause();
	Serial.println("---***--car_turnLeft succeed");						//С��ֹͣת�� ������һ��ѭ��
}

unsigned char  car_turnRight()										//С����ת90��
{
	Serial.println("---***--car_turnRight start");
	car_Right();
	for (unsigned char i=0;i<TIME_CAR_RIGHT;i++)		//6*5*10 ����300ms��ʱ  ���6��
	{	if(get_distence()<=3)			//ת��ʧ��
	       {	Serial.println("!!! car_turnRight failed");
	            car_left_Corret();				//����
	            return 3;
	      }		
			for (loopVar=0;loopVar<5;loopVar++)	//50ms��ʱ
			{
				delay(10);
				if (mode==0)
				{	car_Pause();
				    return 0;
				}
			}
	}//250ms��ʱ
	car_Pause();
	return 0;			//���� ת�����
	Serial.println("---***--car_turnRight succed");
}
/*
  ˵��:խ���еĴ�����--�ڴ�֮ǰ���������ǰ
  С������
	  ����1-->������ǰ�˶�
	  ����2-->̽�⵽����հ�-->������
	  ����3-->���õ����ǰ
	  ����4-->��⵽ǰ������С��8����ǰ������С�ڵ���3-->���˺���
	  
//��֤�Ұ벿���ڱ���̽�⣬����벿���ں�����̽�⡣
*/	
void car_Narrow()				
{   bool isRun=true;
	car_Forword();				//--1
	while(isRun){
		if (get_Rightdis()>=8)		//--2 �ҷ��հ�-->����
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

void car_turn_Back()							//����--��ת
{ 	Serial.println("++++ car_turn_Back start");
	car_back();
	if (get_Leftdis()>=5||get_Rightdis()>=5)
	{	Serial.println("++++ car_turn_Back succeed");
		car_turnLeft();
		car_Pause();
		return;
	}

}
void car_left_Corret()								//�������--
{	Serial.println("++++ car_left_Corret start");
	car_Left();										//С����ת
	for (unsigned char i=0;i<5;i++)						//6�β�๲300ms
	{
		dis_result= get_distence();
		if (abs(dis_result-dis_pre_Turn)<=5)		//�����ɹ�
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause();						//������һ��ѭ��
			return ;								//���
		}
		for (loopVar=0;loopVar<5;loopVar++)			//50������ʱ
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

void car_left_Corret_run()						//ע�� ÿ��У�� ������18�� 3*6��
{	Serial.println("++++ car_left_Corret start");
	car_Left();										//С����ת
	for (loopVar=0;loopVar<6;loopVar++)					//6�β�๲60ms
	{		
		delay(10);									//10ms ���3��									
		scanSingCtrl();
		if (mode==0)
		{   car_Pause(); 
			return ;
		}
		dis_result= get_distence();
		if (abs(dis_result-8)<=3)		//�����ɹ�
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause(); 						//������һ��ѭ��
			return ;								//���
		}
	}	
	car_Pause();
	Serial.println("++++ car_left_Corret failed");
}

void car_Right_Corret()							//�������--
{	Serial.println("++++ car_Right_Corret start");
	car_Right();										//С����ת
	for (unsigned char i=0;i<TIME_CAR_RIGHT;i++)					//6�β�๲300ms
	{
		dis_result= get_distence();
		if (abs(dis_result-dis_pre_Turn)<=5)		//�����ɹ�
		{	Serial.println("++++ car_Right_Corret suceed");										//�ɹ�
			car_turn_Back();						//�������� ��֤�����˽ڵ� ��̽��
			return ;								//���
		}
			for (loopVar=0;loopVar<5;loopVar++)			//50������ʱ   15��
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

void car_Right_Corret_run()							// 18��
{	Serial.println("++++ car_left_Corret start");
	car_Right();										    //С����ת
	for (loopVar=0;loopVar<6;loopVar++)					//6�β�๲60ms
	{		
		delay(10);									       //10ms ���3��									
		scanSingCtrl();
		if (mode==0)
		{   car_Pause(); 
			return ;
		}
		dis_result= get_distence();
		if (abs(dis_result-8)<=5)		//�����ɹ�
		{	Serial.println("++++ car_left_Corret suceed");	
			car_Pause(); 						//������һ��ѭ��
			return ;								//���
		}
	}	
	car_Pause();
    Serial.println("++++ car_left_Corret failed");
}

void car_Forword()
{
	Serial.println("car_Forword");
	mode=(mode&0xf0)|0x02;
	digitalWrite(pinI4, HIGH);				 //ʹֱ��������ң�˳ʱ��ת
	digitalWrite(pinI3, LOW);
	digitalWrite(pinI1, LOW);					//ʹֱ���������˳ʱ��ת
	digitalWrite(pinI2, HIGH);
}
void car_AutoRun()
{
	Serial.println("======auto Run=======");
	analogWrite(speedpin, 140);	//��ʼ�����ת��PWM���
	analogWrite(speedpin1, 155);
	dis_pre_Turn=8;
	mode=mode|0x10;							//����С��״̬
	float mindis=0;								//��ʼ������  �ж�ǰ�����ҵľ���  Ѱ������ı߽�
	float Auototemp;
	serV_default();							//300ms
	Serial.print("farward-dis:");
	mindis=get_distence();
	delay(50);
	direction=1;
	if (mindis>(Auototemp=get_Rightdis()))		//�ҷ���ǰ��С
	{	if(mode==0) { isResetServ=true;return;}
		 direction=4;
		mindis=Auototemp;
	}
	if (mindis>(Auototemp=get_Leftdis()))		//��С
	{	if(mode==0) { isResetServ=true;return;}
		direction=3;
		mindis=Auototemp;
	}
	car_turnLeft();
	if(mode==0) { isResetServ=true;return;}
	Serial.print("back-dis:");
	if (mindis>(Auototemp=get_Leftdis()))		//��С
	{	if(mode==0) { isResetServ=true;return;}
		direction=2;
		mindis=Auototemp;
	}													//ִ����Ϻ�������
	serV_default();										//�������������С������һ��
	Serial.print("=====the Best direct:");
	Serial.print(direction);
	switch (direction)									//С���������
	{
		case 1:	Serial.println("--forword");car_turnRight();break;						//ǰ��
		case 3:	Serial.println("--left");;break;										//��
		case 4:	Serial.println("--Right");car_turnLeft();car_turnLeft();break;			//�ҷ�
		case 2:	Serial.println("--bank");car_turnLeft();break;						      //��
		default:break;
	}
	
	while(get_distence()>=8.0)							//С��һֱǰ��
	{
		action(2);
		scanSingCtrl();
		if (mode==0) return;							//����
		delay(50);										//һֱǰ��
	}													//�������
	
	car_turnLeft();										//����Ϊ�߽�״̬
	Serial.println("eeeeeeeeeeeeeeeend");
	if(mode==0) { return;}	
	autoAction();
									//��ʼ�߽���			
}

void autoAction()
{	Serial.println("--------autoAction---------");
	dis_temp=get_Rightdis();						    //260ms
	if (dis_temp>=6&&dis_temp<=10)					//����
	{	Serial.println(" *** forward normal ***");
	}else if (dis_temp<6)							   //̫������
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
	}else if (dis_temp>10&&dis_temp<=LENGTH_TOO_FAR)	//̫Զ����
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
	dis_temp=get_distence();					//�ж�ǰ��
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
			dis_pre_Turn=get_Leftdis();				//ת���
			if(mode==0) { isResetServ=true;return;}
			car_turnLeft();							//����ת��
			if(mode==0) { isResetServ=true;return;}
			forward_CanLenght();
			if(mode==0) { isResetServ=true;return;}
			serV_default();							//����ָ�
			return ;
		}
	}else if(dis_temp>=500){
		Serial.println(" *** too near***  to error");
		back_CanLenght();
		if(mode==0) { isResetServ=true;return;}
		return;
	}
	dis_temp=get_midLeftDis();					//�ж�ǰ��
	if (dis_temp<=3||dis_temp>500)				//̫��-->ִ��֮ǰ�����Ĭ�Ϸ���
	{	//����-->��ת-->������һ��ѭ��;
		back_CanLenght();			//���˱���ɹ�
		if(mode==0) { isResetServ=true;return;}
		car_turnLeft();				//����ɹ�
		if(mode==0) { isResetServ=true;return;}
		serV_default();
	}
}

/*
	  ����1--��ת���--��ǰ
	  ����2--ǰ��������--���⿨ס����--�����Ƿ�ǰ���ɹ���־
	  ����3--��ת���--����-->��������ֵ
	  ����4--����ת��--�������Ƿ�ת��ɹ��ɹ����-->ʧ����ֱ�ӷ���
	  ����5--��ת���--��ǰ
	  ����6--ת���ǰ��������--���������ѭ��
	  ����7--�ɱ�־λ�ж��Ƿ�Ϊխ��
*/
void car_To_right()
{	forward_flag=0;
	serV_default();						// --1		
	if (forward_CanLenght()==DIS_ERR) {    //--2
		forward_flag=1; Serial.println(" !!! forward failed!!!");}
	if (mode==0) return;
	dis_pre_Turn=get_Rightdis();		    //--3
	
	if(car_turnRight()!=0)					//--4   0Ϊת��ɹ� ��Ϊ0Ϊת��ʧ�� --> ���ö���ָ�ʹ��     
	{ isResetServ=true; return ;}
	
 	if(mode==0) { isResetServ=true;return;}
	serV_default();					   //--5
	
	if(forward_CanLenght()==DIS_ERR) { return ;}	//--6  ǰ��ʧ��ֱ�ӷ���                        
 	if(mode==0) { isResetServ=true;return;}
	if (forward_flag==1)					//--7
	{ car_Narrow();}
}

unsigned char forward_CanLenght()
{	car_Forword();
	for (loopVar=0;loopVar<TIME_CAR_LENGTH;loopVar++)		//��ʱ��30*10=300ms
	{
		delay(10);
		scanSingCtrl();
		if (mode==0||get_distence()<=5)	//ֹͣ����
		{	car_Pause();
			return DIS_ERR;				//ǰ��ʧ��
		}
	}
	car_Pause();
	return 1;
}

void back_CanLenght()
{	car_back();
	for (loopVar=0;loopVar<TIME_CAR_LENGTH;loopVar++)		//��ʱ��30*10=300ms
	{
		delay(10);
		scanSingCtrl();
		if (mode==0||get_distence()>5)	//ֹͣ����
		{	car_Pause();
	    	return ;				//ǰ��ʧ��
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

void servopulse(int myangle)					//����ָ���Ƕȵ�PWM������
{
	pulsewidth=(myangle*11)+500;				//���Ƕ�ת��Ϊ500-2480������ֵ
	digitalWrite(servopin,HIGH);				//������ӿڵ�ƽ����
	delayMicroseconds(pulsewidth);				//��ʱ����ֵ��΢����
	digitalWrite(servopin,LOW);					//������ӿڵ�ƽ����
	delay(20-pulsewidth/1000);						//��ʱ ʹ���ת������
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

unsigned int IRtoAction(unsigned int state)		//ת�������������Ч�ź�Ϊ-1,���ж��Ƿ�����Ч�ź� 
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
    default : Serial.println("**********IR error Msg*********");return -1;			//��Ч�ź�
	}	
}

void scan()
{
	if (recv.decode(&result))						//����Ч���ź�����
	{ 		
		Serial.print("----recevice a IR MSG:");
        Serial.println(result.value,HEX);
		if (result.value != 0xFFFFFFFF)				// �����ظ��ź�
		{
			last_state=result.value;
		}
		IR_state=IRtoAction(last_state);
		if (IR_state!=-1)			//��Ч�º� ������һ�� �뱾��
		{
			last_time=millis();	 
			action(IR_state);
		}
		recv.resume();				//������һ�ν���
	}
	//�������ճ���
	if(Serial.available()>0)		//���ź����룬�����ȫ��Ϊ��Ч�ź� 
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
	if (recv.decode(&result))						//����Ч���ź�����
	{ 		
		Serial.print("+++++++recevice a IR MSG:");
        Serial.println(result.value,HEX);
		last_state=result.value;
		if (last_state==IR_maskCode)
		{
			Serial.println("~~~~~~~~~~~~~~stop~~~~~~~~");
			action(IRtoAction(last_state));
		}
		recv.resume();				//������һ�ν���
	}
	//�������ճ���
	if(Serial.available()>0)		//���ź����룬�����ȫ��Ϊ��Ч�ź� 
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
	if ((mode==1||mode==2))							//ǰ������
	{
		if (current_time-last_time>500)
		{
			car_Stop();
		}
		if (((mode-1)&&get_distence()<10.0))		//����̫�̱���
		{
			car_Stop();
		}
	}else if ((mode==3||mode==4))			//��ת��ת
	{
		if (current_time-last_time>80)
		{
			car_Stop();
		}
	}else if ((mode&0xf0)==0x10)			//�Զ�Ѱ�ұ߽�
	{
		autoAction();
	}else if(mode!=0)						//�쳣ģʽ
	{
		Serial.print("exception State:");
		Serial.println(mode);
	}
	scan();
	if ((mode&0xf0)!=0x10&&isResetServ)
	{	isResetServ=false;
		serV_default();					//�������ת��
	}
}

/*
	buge �޸���¼ 23:01��
	1���޸�ǰ�����Ͼ���Ϊ8
	2���޸��жϾ���    5Ϊ�½�    8λ����   11Ϊ�Ͻ�   11-15Ϊ�Ͻ�������� ģ���߽�Ϊ������
	3��������������ʱ   
=======================================================
	buge �޸���¼ 12:51
	�޸�����ת��      �޸�
*/