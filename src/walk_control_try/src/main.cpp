/*----------------------------------------------------------*/
/*	HAJIME ROBOT 42	for win									*/
/*	main program											*/
/*															*/
/*	file name	:	main.c									*/
/*	target		:	Renesas SH2A/7211						*/
/*	compiler	:	gcc										*/
/*	author		:	Hajime Sakamoto							*/
/*	date		:	2007.7.15								*/
/*	note		:	editor tab = 4							*/
/*															*/
/*	memo		:	����C�y����Windows����̕��s�p�^�[����	*/
/*					�����̂��߂ɕύX��						*/
/*	date		:	2011.12.29								*/
/*					2012.01.14	�������Ő��䂷�邽�߂ɒǉ�*/
/*----------------------------------------------------------*/


/*--------------------------------------*/
/*	include								*/
/*--------------------------------------*/

#include	<stdio.h>
#include	<assert.h>
#include	<stdio.h>
#include	<math.h>
#include	<KSerialPort.h>

#include	<boost/thread.hpp>
#include	<string>
#include	<HCIPC.h>
/*#ifdef VREP_SIMULATOR
#include	<SimulatorIPC.h>
#endif*/

#include "pc_motion.h"
#include	"ADIS16375.h"
#include	"OrientationEstimator.h"
#include <RTIMULib.h>
#include <RTIMUSettings.h>

extern "C" {
#include	"var.h"
#include	"func.h"
#include	"servo_rs.h"
#include	"sq_walk.h"
#include	"cntr.h"
#include	"kine.h"
#include	"serv.h"
#include	"serv_init.h"
#include	"calc_mv.h"
#include	"mvtbl.h"
#include	"sq_motion.h"
#include	"sq_start.h"
#include	"sq_straight.h"
#include	"sq_ready.h"
#include	"sq_walk.h"
#include	"motion.h"
#include	"joy.h"

#include	"acc.h"
#include	"gyro.h"
#include	"b3m.h"
}

//#pragma comment(lib ,"winmm.lib" )

constexpr int FRAME_RATE = 10;	

using namespace std;		//犯罪だろ....
using namespace boost;		//なあ....

static mutex lock_obj;
static string cmd;
static bool response_ready = false;
static string res;


extern "C"

int scif1_tx_fun()
{
	mutex::scoped_lock look(lock_obj);

	int len = 0;
	for(int i = 0; i < SFMT_SIZE-1 && sfmt[i] != EOF_CODE && sfmt[i] != EOF_CODE3; i++ )
	{
		len++;
	}
	res = string(sfmt, len);
	memset(sfmt, EOF_CODE, SFMT_SIZE);
	response_ready = true;

	return 1;
}

string recvHajimeCommand(const string &str, void *context)
{
    static const int CMD_TIMEOUT_MS = 100;
    int cnt = 0;
	if (str.size() > 0) {
		mutex::scoped_lock look(lock_obj);
		cmd = str;
		response_ready = false;
	}
	while (!response_ready && cnt++ < CMD_TIMEOUT_MS) {
#ifdef VREP_SIMULATOR
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
#else
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
#endif
	}
	{
        if (cnt >= CMD_TIMEOUT_MS) {
           std::cerr << "recvHajimeCommand: timeout" << std::endl; 
        }
		mutex::scoped_lock look(lock_obj);
		return res;	
	}
}

void ipcthread(int argc, char *argv[], int id)
{
	HCIPCServer *hcipc = HCIPCServer::createServer(argc, argv, 0, HCIPC_DEFAULT_PORT+id);
	hcipc->setCallback(recvHajimeCommand, NULL);
	hcipc->wait();
}

extern "C"
int	servo_offset[SERV_NUM];	// �I�t�Z�b�g�ۑ��p

//========================
// �I�t�Z�b�g���́i���@����̃f�[�^�j
//========================
int offset_load(char *filename, int servo_offset[SERV_NUM]){
	char off_tmp[100];
	int i, ser_ID, off_angle;
	FILE *fp;

	for(i = 0; i < SERV_NUM; i ++){
		servo_offset[i] = 0;
	}

	if(NULL == (fp = fopen(filename, "r"))){
		fprintf(stderr, "cannot open (offset_load): %s\n", filename);
		return -1;
	}
	while(fscanf(fp, "%d %d %s",&ser_ID, &off_angle, off_tmp) != EOF){
		servo_offset[ser_ID] = off_angle;
	}
	fclose(fp);

	return 0;
}

//========================
// eeprom load
//========================
int eeprom_load(char *filename)
{
	char buff_tmp[100];
	int buff_num;
	float buff_val, eeprom_buff[111];
	FILE *fp;

	if(NULL == (fp = fopen(filename,"r"))){
		fprintf(stderr, "cannot open (eeprom_load): %s\n", filename);
		return -1;
	}
	while(fscanf(fp, "%d %f %s",&buff_num, &buff_val, buff_tmp) != EOF){
		eeprom_buff[buff_num] = buff_val;
	}
	fclose(fp);

	//flag_eeprom_para_ok			=	eeprom_buff[ 0];
	//sw_soft1						=	eeprom_buff[ 1];
	//sw_soft2						=	eeprom_buff[ 2];
	//sw_soft3						=	eeprom_buff[ 3];
	//sw_soft4						=	eeprom_buff[ 4];
	xp_acc.acc_k1					=	eeprom_buff[ 5];
	xp_acc.acc_k2					=	eeprom_buff[ 6];
	xp_acc.acc_k3					=	eeprom_buff[ 7];
	xp_acc.ad_volt_offset1			=	eeprom_buff[ 8];
	xp_acc.ad_volt_offset2			=	eeprom_buff[ 9];
	xp_acc.ad_volt_offset3			=	eeprom_buff[10];
	xp_acc.t1						=	eeprom_buff[11];
	xp_acc.t2						=	eeprom_buff[12];
	xp_acc.fall_fwd					=	eeprom_buff[13];
	xp_acc.fall_bwd					=	eeprom_buff[14];
	xp_acc.fall_right				=	eeprom_buff[15];
	xp_acc.fall_left				=	eeprom_buff[16];
	xp_acc.fall_check_time			=	eeprom_buff[17];
	xp_acc.fall_pitch				=	eeprom_buff[18];
	xp_acc.fall_roll				=	eeprom_buff[19];
	xp_acc.fall_pitch_oblique		=	eeprom_buff[20];
	xp_acc.fall_roll_oblique		=	eeprom_buff[21];
			
	xp_gyro.kp1_foot				=	eeprom_buff[25];
	xp_gyro.kp2_foot				=	eeprom_buff[26];
	xp_gyro.kp1_hip					=	eeprom_buff[27];
	xp_gyro.kp2_hip					=	eeprom_buff[28];
	xp_gyro.kp1_arm					=	eeprom_buff[29];
	xp_gyro.kp2_arm					=	eeprom_buff[30];
	xp_gyro.kp2_waist				=	eeprom_buff[31];
	xp_gyro.kp3_waist				=	eeprom_buff[32];
	xp_gyro.gyro_k1					=	eeprom_buff[33];
	xp_gyro.gyro_k2					=	eeprom_buff[34];
	xp_gyro.gyro_k3					=	eeprom_buff[35];
	xp_gyro.ad_volt_offset1			=	eeprom_buff[36];
	xp_gyro.ad_volt_offset2			=	eeprom_buff[37];
	xp_gyro.ad_volt_offset3			=	eeprom_buff[38];
	xp_gyro.t1						=	eeprom_buff[39];
	xp_gyro.t2						=	eeprom_buff[40];
	xp_gyro.gyro_data3_flt2_t1		=	eeprom_buff[41];
	xp_gyro.yaw_cntl_gain			=	eeprom_buff[42];
	xp_gyro.yaw_cntl_dead			=	eeprom_buff[43];
	xp_gyro.yaw_cntl_theta			=	eeprom_buff[44];
	xp_gyro.gyro_omega				=	eeprom_buff[45];
	xp_gyro.fall_roll_deg1			=	eeprom_buff[46];
	xp_gyro.fall_pitch_deg1			=	eeprom_buff[47];
	flag_gyro.fall_cntl				=	(short)eeprom_buff[48];
			
	xp_mv_straight.time				=	eeprom_buff[50];
	xp_mv_straight.z3				=	eeprom_buff[51];
	xp_mv_straight.arm_sh_pitch		=	eeprom_buff[52];
	xp_mv_straight.arm_sh_roll		=	eeprom_buff[53];
	xp_mv_straight.arm_el_yaw		=	eeprom_buff[54];
	xp_mv_straight.arm_el_pitch		=	eeprom_buff[55];
			
	xp_mv_ready.time				=	eeprom_buff[56];
	xp_mv_ready.z3					=	eeprom_buff[57];
	xp_mv_ready.arm_sh_pitch		=	eeprom_buff[58];
	xp_mv_ready.arm_sh_roll			=	eeprom_buff[59];
	xp_mv_ready.arm_el_yaw			=	eeprom_buff[60];
	xp_mv_ready.arm_el_pitch		=	eeprom_buff[61];
	xp_mv_ready.pitch				=	eeprom_buff[62];

	xp_mv_walk.num					=	(long)eeprom_buff[65];
	xp_mv_walk.h_cog				=	eeprom_buff[66];
	xp_mv_walk.time					=	eeprom_buff[67];
	xp_mv_walk.x_fwd_swg			=	eeprom_buff[69];
	xp_mv_walk.x_fwd_spt			=	eeprom_buff[70];
	xp_mv_walk.x_bwd_swg			=	eeprom_buff[71];
	xp_mv_walk.x_bwd_spt			=	eeprom_buff[72];
	xp_mv_walk.y_swg				=	eeprom_buff[73];
	xp_mv_walk.y_spt				=	eeprom_buff[74];
	xp_mv_walk.theta				=	eeprom_buff[75];
	xp_mv_walk.z					=	eeprom_buff[76];
	xp_mv_walk.y_balance			=	eeprom_buff[77];
	xp_mv_walk.hip_roll				=	eeprom_buff[78];
	xp_mv_walk.x_fwd_pitch			=	eeprom_buff[79];
	xp_mv_walk.x_bwd_pitch			=	eeprom_buff[80];
	xp_mv_walk.arm_sh_pitch			=	eeprom_buff[81];
	xp_mv_walk.start_time_k1		=	eeprom_buff[82];
	xp_mv_walk.start_zmp_k1			=	eeprom_buff[83];
//	xp_mv_walk.start_time_k2		=	eeprom_buff[84];
	xp_mv_walk.foot_cntl_p			=	eeprom_buff[85];
	xp_mv_walk.foot_cntl_r			=	eeprom_buff[86];
	xp_mv_walk.sidestep_time_k		=	eeprom_buff[87];
	xp_mv_walk.sidestep_roll		=	eeprom_buff[88];
	xp_mv_walk.y_wide				=	eeprom_buff[90];
	xp_mv_walk.time_dutyfactor		=	eeprom_buff[91];
			
	xp_dlim_wait_x.dlim				=	eeprom_buff[92];
	xp_dlim_wait_x.wait_time		=	eeprom_buff[93];
	xp_dlim_wait_y.dlim				=	eeprom_buff[94];
	xp_dlim_wait_y.wait_time		=	eeprom_buff[95];
	xp_dlim_wait_theta.dlim			=	eeprom_buff[96];
	xp_dlim_wait_theta.wait_time	=	eeprom_buff[97];
			
	odometry_correct_para_x			=	eeprom_buff[98];
	odometry_correct_para_y			=	eeprom_buff[99];

	xp_mv_walk.x_fwd_acc_pitch		=	eeprom_buff[101];
	xp_mv_walk.x_bwd_acc_pitch		=	eeprom_buff[102];
	xp_dlim_wait_pitch.dlim			=	eeprom_buff[103];		// �s�b�`��ύX����䗦
    xp_mv_walk.accurate_x_percent_dlim = eeprom_buff[104];
    xp_mv_walk.accurate_y_percent_dlim = eeprom_buff[105];
	xp_mv_walk.accurate_th_percent_dlim = eeprom_buff[106];
	xp_mv_walk.arm_el_pitch         =   eeprom_buff[107];

	return 0;
}

/*--------------------------------------*/
/*	PC simulation main					*/
/*--------------------------------------*/
int		main( int argc, char *argv[] )
{
	int id = 0;
	//short j;
	int shutdown_flag = 0;
/*
#ifdef VREP_SIMULATOR
	OrientationEstimator orientationEst((double)FRAME_RATE / 1000.0, 0.1);
	SimulatorIPCClient client;
	for (int i=1; i<argc; i++) {
		std::string str(argv[i]);
		if (str.find("ID=") == 0) {
			std::string idstr = str.substr(3);   
			id = strtol(idstr.c_str(), NULL, 10);
		}
	}
#endif
*/

	//boost::thread thread(boost::bind(ipcthread, argc, argv, id));
	boost::posix_time::ptime time_of_previous_loop = boost::posix_time::microsec_clock::local_time(); 
	const char *servo_port = "/dev/kondoservo";
	if (argc > 1)
		servo_port = argv[1];
//#if !defined VREP_SIMULATOR
	int m_sampleCount = 0;
	int m_sampleRate = 0;
	uint64_t m_rateTimer;
	uint64_t m_displayTimer;
	uint64_t m_now;
	RTIMUSettings *m_settings = new RTIMUSettings("RTIMULib");
	RTIMU *imu = RTIMU::createIMU(m_settings);

	if((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)){
		printf("No IMU found\n");
		exit(1);
	}
	imu->IMUInit();

	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(false);

	m_rateTimer = m_displayTimer = RTMath::currentUSecsSinceEpoch();


	RSOpen(servo_port);
	B3MTorqueALLDown();

//#endif

/*	#ifdef _MSC_VER
	timeBeginPeriod(1);
#endif */

	var_init();					// �ϐ��̏�����
	serv_init();				// �T�[�{���[�^�̏�����
	calc_mv_init();				// �����̌v�Z�̏�����
	load_pc_motion("motions");
	offset_load((char *)"offset_angle.txt", servo_offset);
	eeprom_load((char *)"eeprom_list.txt");
	flag_gyro.zero = ON;

	// loop start-----------------------------------------------
	for( count_time_l = 0; ; count_time_l ++ )
	{
		bool cmd_accept = false;
		{
			// accept command	
			//ジョイスティックからのコマンドを受け取っている訳ではないが、普通にコマンドを受け取っていた。コメントを安易に信じるべきでない。
			mutex::scoped_lock look(lock_obj);
			if (cmd.size() > 0) {
				memcpy(rfmt, &cmd[0], cmd.size());
				joy_read();					//この辺でspin_someしとけばいいんじゃないか知らんけど
				cmd_accept = true;
				cmd = "";
			}
		}
//#if !defined VREP_SIMULATOR
		{
			///// MPU9250 reading sensor data, calc quaternion and settings
			static int continueous_error_count = 0;
			
			if(imu->IMURead())
			{
				continueous_error_count = 0;
				RTIMU_DATA imuData = imu->getIMUData();
				m_sampleCount++;
				m_now = RTMath::currentUSecsSinceEpoch();

				if((m_now - m_displayTimer) > 1000000)
				{
					m_displayTimer = m_now;
				}

				if((m_now - m_rateTimer) > 1000000)
				{
					m_sampleRate =m_sampleCount;
					m_sampleCount = 0;
					m_rateTimer = m_now;
				}

				xv_acc.acc_data1 =  -imu->getIMUData().accel.y();
				xv_acc.acc_data2 =  imu->getIMUData().accel.x();
				xv_acc.acc_data3 =  imu->getIMUData().accel.z();

				const float radian_to_degree = 180.0 / M_PI;
				xv_gyro.gyro_data1 = -imu->getIMUData().gyro.y() * radian_to_degree;
				xv_gyro.gyro_data2 = imu->getIMUData().gyro.x() * radian_to_degree;
				xv_gyro.gyro_data3 = imu->getIMUData().gyro.z() * radian_to_degree;

				xv_gyro.quaternion[0] = -imuData.fusionQPose.x();
				xv_gyro.quaternion[1] = imuData.fusionQPose.z();
				xv_gyro.quaternion[2] = imuData.fusionQPose.scalar();
				xv_gyro.quaternion[3] = imuData.fusionQPose.y();

				double pitch_abs = 180.0 - abs(imuData.fusionPose.x()*RTMATH_RAD_TO_DEGREE);
				if(imuData.fusionPose.x() < 0)
					pitch_abs = -pitch_abs;
				xv_gyro.gyro_roll = xv_gyro.gyro_roll2 = imuData.fusionPose.y()*RTMATH_RAD_TO_DEGREE;
				xv_gyro.gyro_pitch = xv_gyro.gyro_pitch2 = pitch_abs;
				xv_gyro.gyro_yaw = xv_gyro.gyro_yaw2 = -(imuData.fusionPose.z()*RTMATH_RAD_TO_DEGREE);
			} else {
				FILE *fp = fopen("/var/tmp/error.txt","a");
				if (fp != NULL){
					fprintf(fp, "IMU Read Error\r\n");
					fclose(fp);
				}
				continueous_error_count ++;
				if (continueous_error_count > 10) shutdown_flag = 1;
			}
		}
//#endif //!defined VREP_SIMULATOR


		if (!shutdown_flag) cntr();			//重要ゾーン

/*#if 0
		{
			FILE *fp;
			static float old_angle[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			
			fp = fopen("data.csv","a");
			fprintf(fp, "%f, %f, %f, %f, %f, %f, ",
				xv_kine[0].x, xv_kine[0].y, xv_kine[0].z, xv_kine[1].x, xv_kine[1].y, xv_kine[1].z);
			for(int i = 0; i < 24; i ++) {
				fprintf(fp, "%f, ", xv_sv[i].pls_out / 10.0f);
			}
			for(int i = 0; i < 24; i ++) {
				fprintf(fp, "%f, ", (xv_sv[i].pls_out / 10.0f - old_angle[i]) / (FRAME_RATE / 1000.0f));
				old_angle[i] = xv_sv[i].pls_out / 10.0f;
			}			

			fprintf(fp, "\n");
			fclose(fp);
		}
#endif*/
		//倒れた時に頭の位置を正面に戻しているっぽい
		static unsigned long last_pan_update = 0;
		if ((fabs(xv_gyro.gyro_roll) > 30)||(fabs(xv_gyro.gyro_pitch) > 30)){
			if (abs((long)(count_time_l - last_pan_update)) > 10){
				set_xv_comm(&xv_comm, 'H', '0', '0', '0', '1', '0');
				convert_bin(&xv_comm_bin, &xv_comm);
				last_pan_update = count_time_l;
			}
		}
		//これは何ですか。多分エラー出力ですよね。何のエラー...。
		for(int j=0; j<SERV_NUM; j++ )
		{
			if( xv_sv[j].deg_sv	> xp_sv[j].deg_lim_h*100 || xv_sv[j].deg_sv < xp_sv[j].deg_lim_l*100 )
					printf("*******ERROR**** xv_sv[%d].deg_sv=%f\n", j, xv_sv[j].deg_sv/100.0f);
		}

/*#ifdef VREP_SIMULATOR
		{
			static int cnt = 0;
			std::vector<int> angles(24, 0);
			for(int i=0; i<24; i++)
				angles[i] = xv_sv[i].pls_out + servo_offset[i];
			float ptime = client.getSimulationTime();
			hc::SensorData sd;
			do {
				sd = client.setJointAngles(id, angles, 10);
				boost::this_thread::sleep(boost::posix_time::microseconds(1000));
				// empty sensor data indicate that simulation didn't progress 
			} while(sd.accel.size() == 0);

			while (ptime == client.getSimulationTime()) {
				boost::this_thread::sleep(boost::posix_time::microseconds(5000));
			}
			if (cnt > 5) {
				xv_gyro.gyro_data1 = -sd.gyro[0] * 1;	// roll		�������I�I	�\�z�ł�	�ő�d��(�I�t�Z�b�g�ς�):1[V], �ő匟�o:500[deg/sec] 1/500=0.002 ��xp_gyro.gyro_k1,2(1000)�������� 0.002*1000=2
				xv_gyro.gyro_data2 = -sd.gyro[1] * 1;	// pitch	�������I�I
				xv_gyro.gyro_data3 =  sd.gyro[2] * 1;	// yaw	�ő�d��(�I�t�Z�b�g�ς�):2[V], �ő匟�o:200[deg/sec] 2/200=0.01 ��xp_gyro.gyro_k3(100)�������� 0.01*100=1
				xv_acc.acc_data1 = sd.accel[0] / 9.8f * 0.3f * 3.1f;	// x	/9.8�ŒP�ʂ�G����m/ss�ɂ���
				xv_acc.acc_data2 = sd.accel[1] / 9.8f * 0.3f * 3.1f;	// y	1.08/3.6 = 0.3 �ŃX�P�[�������킹��	�Z���T�̍ő�d���i�I�t�Z�b�g�ς݁j:1.08[V], �ő匟�o:3.6[G]
				xv_acc.acc_data3 = sd.accel[2] / 9.8f * 0.3f * 3.1f;	// z	�Ō��xp_acc.acc_k(3.1)��������
//				printf("R:%lf\tP:%lf\tY:%lf\n",xv_gyro.gyro_data1, xv_gyro.gyro_data2, xv_gyro.gyro_data3);
//				printf("X:%f\tY:%f\tZ:%f\n",xv_acc.acc_data1, xv_acc.acc_data2, xv_acc.acc_data3);
				
				if ((xv_acc.acc_data1 != 0.0)||(xv_acc.acc_data2 != 0.0)||(xv_acc.acc_data3 != 0.0)){
					orientationEst.update(xv_gyro.gyro_data1*M_PI/180.0, xv_gyro.gyro_data2*M_PI/180.0, xv_gyro.gyro_data3*M_PI/180.0,
						xv_acc.acc_data1*9.8, xv_acc.acc_data2*9.8, xv_acc.acc_data3*9.8);
		
					xv_gyro.gyro_roll   =
					xv_gyro.gyro_roll2  = orientationEst.getRoll()   * 180.0 / M_PI;
					xv_gyro.gyro_pitch  =
					xv_gyro.gyro_pitch2 = orientationEst.getPitch()  * 180.0 / M_PI;
					xv_gyro.gyro_yaw    =
					xv_gyro.gyro_yaw2   = orientationEst.getYaw()    * 180.0 / M_PI;
					orientationEst.getQuaternion(&xv_gyro.quaternion[0], &xv_gyro.quaternion[1], &xv_gyro.quaternion[2], &xv_gyro.quaternion[3]);
//					printf("Roll:%f\tPitch:%f\tYaw:%f\n",xv_gyro.gyro_roll, xv_gyro.gyro_pitch, xv_gyro.gyro_yaw);
				}
			}
			cnt ++;
		}
#endif // VREP_SIMULATOR*/




//この辺は普通にros2の時間管理でなんとかなりそうゾーンだな。spinOnceがあるのか知らんけどそれ系でどうにかなりそう。
//#if !defined VREP_SIMULATOR
		//rtm_main();//���[�V�������[�_�𓮂����̂ɕK�v�����A�K�؂ȃ^�C�~���O��������K�v����
		// �������̂��߂�wait
		boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time(); 
		boost::posix_time::time_duration diff = now - time_of_previous_loop;
		if (diff.total_milliseconds() > (FRAME_RATE))
		{
			FILE *fp = fopen("/var/tmp/error.txt","a");
			if (fp != NULL){
				time_t now = time(NULL);
				struct tm *pnow = localtime(&now);
				fprintf(fp, "%d:%d:%d WALKING CONTROL OVERTIME: %dms\r\n", 
				pnow->tm_hour, pnow->tm_min, pnow->tm_sec,
				(int)diff.total_milliseconds());
				fclose(fp);
			}
		}

		while(diff.total_milliseconds() < FRAME_RATE) {
//			boost::this_thread::sleep(boost::posix_time::microseconds(10));
			boost::this_thread::sleep(boost::posix_time::microseconds(FRAME_RATE*1000 - diff.total_microseconds() - 100));
			now = boost::posix_time::microsec_clock::local_time(); 
			diff = now - time_of_previous_loop;
		}
//		printf("%dus\n", diff.total_microseconds());
		time_of_previous_loop = now;
//#endif
//		printf("cnt:%05d mode:%d%d%d%d%d\n", count_time_l, sq_flag.start, sq_flag.straight, sq_flag.ready, sq_flag.walk, sq_flag.motion);
	}
}