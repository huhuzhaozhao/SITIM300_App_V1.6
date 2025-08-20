
/***************************************************************************************************
****************************************************************************************************
@file		   : calibration.h
@brief         : This file contains all the object and function prototypes for
				 the calibration.c file.
@creation time : 2025-03-27-YY
****************************************************************************************************
***************************************************************************************************/


/* Define to prevent recursive inclusion ------------------------------------------------------------------------- */


#ifndef __CALIBRATION__
#define __CALIBRATION__


/* Includes ------------------------------------------------------------------------------------------------------ */





/* Private Defines ----------------------------------------------------------------------------------------------- */


#define IS_DOUBLE	0		// 解算数据类型 float=0 double=1

#define D2			2		// 维数 2
#define D3			3		// 维数 3
#define D4			4		// 维数 4
#define D5			5		// 维数 5
#define IMU_COUNT	8		// IMU 8个

#define ONLY_T		1		// 仅温度融合
#define ONLY_IMU	0		// IMU融合
#define IN			1		// 嵌入式给算法
#define OUT			0		// 算法给嵌入式

#define TCUT		5		// 温度分段点 5=1+3+1 3点分4段
#define TCUT_ALL	24		// 陀螺温补固定分段点 [-45:5:70] 24点(算首尾) 分23段
#define GCUT		4		// 速率分段点个数 4点分5段
#define K3210		4		// 三阶多项式系数
#define KB			2		// 一阶多项式系数

#define G_LIMIT		550		// 陀螺 量程 +-500°/s
#define A_LIMIT		11		// 加计 量程 +-10g

#define HZ_100		100		// 输出率100HZ 输出率不同低通参数不同 可能会平均
#define HZ_200		200		// 输出率200HZ
#define HZ_500		500		// 输出率500HZ
#define HZ_1k		1000	// 输出率1000HZ
#define HZ_2k		2000	// 输出率2000HZ

#define FILTER_SIZE	5		// 滑动平均滤波窗口大小
#define LOW_PASS_N	3		// 低通滤波阶数 3阶 f=2000HZ wc=0.2*(2000/2)=200HZ phi=97°

#define MANUAL_PARA_ON 		0		// 1启动手动参数赋值 0关闭
#define PRODUCT_NUMBER 2501002	// 参数产品号 2501001~2501003


/* Private Types ------------------------------------------------------------------------------------------------- */


#if IS_DOUBLE	// IS_DOUBLE
typedef double MYTYPE;	// double
#else
typedef float MYTYPE;	// float
#endif			// IS_DOUBLE

typedef struct			// 融合后IMU数据 (3+3+1)*4 = 7*4 = 28 Byte
{
	MYTYPE gyro[D3];	// +3 陀螺XYZ轴 °/s
	MYTYPE acc[D3];	    // +3 加计XYZ轴 g
	MYTYPE T;			// +1 温度 ℃
} SENSOR_DATA;

typedef struct			// IMU阵列数据 n组
{
	SENSOR_DATA IMU[IMU_COUNT];
} IMU_ARRAY;

typedef struct			// IMU陀螺温补系数
{
	MYTYPE para[TCUT_ALL - 1][KB * D3 * 2];		// 23*(2+2+2)*2*4 = 276*4 = 1104 Byte
} GYRO_T_COMP;

typedef struct					// 陀螺加计 温补及标定参数 (1+12+24+60+12+276*8+24+48+15+2+32+1)*4 = 2439*4 = 9756 Byte
{
	// #OSN_IMU16488,0#
	MYTYPE Version;				// 例202500301
	// #陀螺加计轴向极性#
	MYTYPE G_pola[D3];
	MYTYPE A_pola[D3];
	MYTYPE G_axis[D3];
	MYTYPE A_axis[D3];
	// #陀螺、加计交叉耦合及零偏#
	MYTYPE G_cross[D3][D3];
	MYTYPE G_bias[D3];
	MYTYPE A_cross[D3][D3];
	MYTYPE A_bias[D3];
	// #陀螺标度因素非线性补偿参数#
	MYTYPE Gx_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
	MYTYPE Gy_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
	MYTYPE Gz_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
	MYTYPE Gxyz_cut4[D3][GCUT];				// xyz 4分段点
	// #陀螺阵列温度补偿参数，每5℃分段 23*[kb kb kb;]*8组#
	GYRO_T_COMP T_d23_kb_xyz[IMU_COUNT];		// 8个IMU温补参数 
	// #陀螺阵列温度分段点，每5℃分段#
	MYTYPE T_cut[TCUT_ALL];						// [-45:5:70]
	// #加计温度补偿参数#
	MYTYPE AT_p3210_d4[D3][K3210 * D4];			// xyz 3阶4段
	// #加计温度分段点参数#
	MYTYPE AT_cut5[D3][TCUT];					// xyz 1+3+1分段点
	// #温度修正参数#
	MYTYPE T_kb[KB];							// 温度自身系数 kb
	// #IMU阵列选择，8组*[Gx;Gy;Gz;T]#
	MYTYPE Array_xyzT_8[D3 * 2 + 1][IMU_COUNT];	// 对应位置 1为选择 0为弃用
	// #参数生成时间#
	MYTYPE Time;								// 例250327
	// $$
} CALIB_PARA;

typedef struct				  // 低通滤波器缓存
{
	MYTYPE X[LOW_PASS_N + 1]; // 输入 x(k) x(k-1) x(k-2) x(k-3)
	MYTYPE Y[LOW_PASS_N + 1]; // 输出 y(k) y(k-1) y(k-2) y(k-3)
} LP_FILTER;				  // 低通滤波器缓存


/* (Extern) vars ------------------------------------------------------------------------------------------------- */





/* (Extern) functions -------------------------------------------------------------------------------------------- */


extern void imuProcess(void);				 // 标定主函数
extern void imuParaInit(CALIB_PARA* p_para); // 参数结构体初始化

extern void changeAxialAndPolarity(IMU_ARRAY* p_IMUarray, CALIB_PARA* p_para);	// 变换轴向和极性
extern void gyroTempCompensate(SENSOR_DATA* p_sensor, CALIB_PARA* p_para, IMU_ARRAY* p_IMUarray);// 仅陀螺 不分段温度补偿
extern void accTempCompensate(SENSOR_DATA* p_sensor, CALIB_PARA* p_para);						 // 仅加计 分段温度补偿
extern void gyroPartCompensate(SENSOR_DATA* p_sensor, CALIB_PARA* p_para);		// 陀螺分段补偿
extern void calibrateAllSensor(SENSOR_DATA* p_sensor, CALIB_PARA* p_para);		// 标定 零偏+耦合

extern char isOverRange(MYTYPE data, char GorA);									// 判断超量程
extern char whichPart(MYTYPE data, MYTYPE* cut, char n);							// 判定温度或转速属于哪个区间
extern char changeTPart(char part);													// 温度区间转变
extern char changeTPart2(char part);												// 陀螺温度区间转变
extern void matrixMultiply(MYTYPE* A, MYTYPE* B, MYTYPE* C, int m, int n, int p);	// 矩阵乘法

extern void imuArrayFusion(char D);							 // IMU阵列融合
extern void meanFusion(char D, char only_T);				 // 取平均值融合
extern MYTYPE lowPassFilter(LP_FILTER* p_filter, MYTYPE input);// 低通滤波

extern void assignmentStruct(char IN_or_OUT);				 // 传数给算法


/* Define to prevent recursive inclusion ------------------------------------------------------------------------- */


#endif /* __CALIBRATION__ */

