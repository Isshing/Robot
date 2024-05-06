#include "IMU.h"
#include <math.h>
#define M_PI 3.14159265358979323846

SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
u8 ttl_receive;
u8 Fd_data[64];
u8 Fd_rsimu[64];
u8 Fd_rsahrs[56];
int rs_imutype =0;
int rs_ahrstype =0;
IMUData_Packet_t IMUData_Packet;
AHRSData_Packet_t AHRSData_Packet;
float angle_change(float angle){
		if (angle > 180) {
			 angle -= 360;
		} else if (angle < -180) {
			 angle += 360;
		}
		return angle;
}
/*************
ʵ��16���Ƶ�can����ת���ɸ���������
****************/
float DATA_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4)
{
  u32 transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//����λ
	//�����Ʋ������ٰ�λ����㣬���������30��23λ��Ӧ��e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//��22~0ת��Ϊ10���ƣ��õ���Ӧ��xϵ�� 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}
long long timestamp(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4)
{
  u32 transition_32;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
	return transition_32;
}

/*******************************
16����ת����������
*******************************/
float roll_deg;
float pitch_deg;
float heading_deg;
float gyro_x_deg;
float gyro_y_deg;
float gyro_z_deg;
bool_t initial_flag=0;
float  initial_angle;
u8 TTL_Hex2Dec(void)  
{
	 if(rs_ahrstype==1)
	{
		if(Fd_rsahrs[1]==TYPE_AHRS&&Fd_rsahrs[2]==AHRS_LEN)
		{	
		AHRSData_Packet.RollSpeed=DATA_Trans(Fd_rsahrs[7],Fd_rsahrs[8],Fd_rsahrs[9],Fd_rsahrs[10]);       //������ٶ�
		AHRSData_Packet.PitchSpeed=DATA_Trans(Fd_rsahrs[11],Fd_rsahrs[12],Fd_rsahrs[13],Fd_rsahrs[14]);   //�������ٶ�
		AHRSData_Packet.HeadingSpeed=DATA_Trans(Fd_rsahrs[15],Fd_rsahrs[16],Fd_rsahrs[17],Fd_rsahrs[18]); //ƫ�����ٶ�
			
    AHRSData_Packet.Roll=DATA_Trans(Fd_rsahrs[19],Fd_rsahrs[20],Fd_rsahrs[21],Fd_rsahrs[22]);      //�����
		AHRSData_Packet.Pitch=DATA_Trans(Fd_rsahrs[23],Fd_rsahrs[24],Fd_rsahrs[25],Fd_rsahrs[26]);     //������
		AHRSData_Packet.Heading=DATA_Trans(Fd_rsahrs[27],Fd_rsahrs[28],Fd_rsahrs[29],Fd_rsahrs[30]);	 //ƫ����
			
    roll_deg = AHRSData_Packet.Roll * (180.0 / M_PI);
		roll_deg = angle_change(roll_deg);
    pitch_deg = AHRSData_Packet.Pitch * (180.0 / M_PI);
		pitch_deg = angle_change(pitch_deg);
    heading_deg = AHRSData_Packet.Heading * (180.0 / M_PI);
		heading_deg = angle_change(heading_deg);
		if(initial_flag==0){
			initial_angle = heading_deg;
			initial_flag = 1;
			}
		}
	rs_ahrstype=0;
 }
	if(rs_imutype==1)
	{
		if(Fd_rsimu[1]==TYPE_IMU&&Fd_rsimu[2]==IMU_LEN)
		{
		IMUData_Packet.gyroscope_x=DATA_Trans(Fd_rsimu[7],Fd_rsimu[8],Fd_rsimu[9],Fd_rsimu[10]);  //���ٶ�
		IMUData_Packet.gyroscope_y=DATA_Trans(Fd_rsimu[11],Fd_rsimu[12],Fd_rsimu[13],Fd_rsimu[14]);
		IMUData_Packet.gyroscope_z=DATA_Trans(Fd_rsimu[15],Fd_rsimu[16],Fd_rsimu[17],Fd_rsimu[18]);
			
		IMUData_Packet.accelerometer_x=DATA_Trans(Fd_rsimu[19],Fd_rsimu[20],Fd_rsimu[21],Fd_rsimu[22]);  //�߼��ٶ�
		IMUData_Packet.accelerometer_y=DATA_Trans(Fd_rsimu[23],Fd_rsimu[24],Fd_rsimu[25],Fd_rsimu[26]);
		IMUData_Packet.accelerometer_z=DATA_Trans(Fd_rsimu[27],Fd_rsimu[28],Fd_rsimu[29],Fd_rsimu[30]);

		gyro_x_deg = IMUData_Packet.gyroscope_x * (180.0 / 3.14159265358979323846);
    gyro_y_deg = IMUData_Packet.gyroscope_y * (180.0 / 3.14159265358979323846);
    gyro_z_deg = IMUData_Packet.gyroscope_z * (180.0 / 3.14159265358979323846);

		}
		rs_imutype=0;
 }
return 0;
}
/*************
ʵ��16���Ƶ�can����ת���ɸ���������
****************/
float Data_Trans(u8 Data_1,u8 Data_2,u8 Data_3,u8 Data_4)
{
  long long transition_32;
	float tmp=0;
	int sign=0;
	int exponent=0;
	float mantissa=0;
  transition_32 = 0;
  transition_32 |=  Data_4<<24;   
  transition_32 |=  Data_3<<16; 
	transition_32 |=  Data_2<<8;
	transition_32 |=  Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;//����λ
	//�����Ʋ������ٰ�λ����㣬���������30��23λ��Ӧ��e
	exponent = ((transition_32 >> 23) & 0xff) - 127;
	//��22~0ת��Ϊ10���ƣ��õ���Ӧ��xϵ�� 
	mantissa = 1 + ((float)(transition_32 & 0x7fffff) / 0x7fffff);
	tmp=sign * mantissa * pow(2, exponent);
	return tmp;
}
void AHRSData2PC(void)
{
	  printf("**********                **********\r\n");	
	
 	 printf("AHRS: The RollSpeed =  %f\r\n",AHRSData_Packet.RollSpeed);
	 printf("AHRS: The PitchSpeed =  %f\r\n",AHRSData_Packet.PitchSpeed);
   printf("AHRS: The HeadingSpeed =  %f\r\n",AHRSData_Packet.HeadingSpeed);
   printf("AHRS: The Roll =  %f\r\n",AHRSData_Packet.Roll);
   printf("AHRS: The Pitch =  %f\r\n",AHRSData_Packet.Pitch);
   printf("AHRS: The Heading =  %f\r\n",AHRSData_Packet.Heading);
   printf("AHRS: The Quaternion.Qw =  %f\r\n",AHRSData_Packet.Qw);
   printf("AHRS: The Quaternion.Qx =  %f\r\n",AHRSData_Packet.Qx);
   printf("AHRS: The Quaternion.Qy =  %f\r\n",AHRSData_Packet.Qy);
   printf("AHRS: The Quaternion.Qz =  %f\r\n",AHRSData_Packet.Qz);
   printf("AHRS: The Timestamp =  %d\r\n",AHRSData_Packet.Timestamp);
	  printf("**********                **********\r\n");	
	
}
void IMUData2PC(void)
{
   //printf("Now start sending IMU data.\r\n");
	  printf("**********                **********\r\n");	

	 printf("IMU: The gyroscope_x =  %f\r\n",IMUData_Packet.gyroscope_x);
	 printf("IMU:The gyroscope_y =  %f\r\n",IMUData_Packet.gyroscope_y);
   printf("IMU:The gyroscope_z =  %f\r\n",IMUData_Packet.gyroscope_z);
   printf("IMU:The accelerometer_x =  %f\r\n",IMUData_Packet.accelerometer_x);
   printf("IMU:The accelerometer_y =  %f\r\n",IMUData_Packet.accelerometer_y);
   printf("IMU:The accelerometer_z =  %f\r\n",IMUData_Packet.accelerometer_z);
   printf("IMU:The magnetometer_x =  %f\r\n",IMUData_Packet.magnetometer_x);
   printf("IMU:The magnetometer_y =  %f\r\n",IMUData_Packet.magnetometer_y);
   printf("IMU:The magnetometer_z =  %f\r\n",IMUData_Packet.magnetometer_z);
   printf("IMU:The Timestamp =  %d\r\n",IMUData_Packet.Timestamp);
	 //printf("Now the data of IMU has been sent.\r\n");
   printf("**********                **********\r\n");	

}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s						
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//�Խ��յ������ݽ���У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}



