#include "main.h"
#include "calibration.h"

//#define D2			2		// 维数 2
//#define D3			3		// 维数 3
//#define D4			4		// 维数 4
//#define D5			5		// 维数 5

//#define TCUT		5		// 温度分段点 5=1+3+1 3点分4段
//#define GCUT		4		// 速率分段点个数 4点分5段
//#define K3210		4		// 三阶多项式系数
//#define KB			2		// 一阶多项式系数

//#define G_LIMIT		500		// 芯能联科陀螺 量程 +-500°/s
//#define A_LIMIT		10		// 加计		  量程 +-10g

//#define HZ_100		100		// 输出率100HZ 输出率不同低通参数不同
//#define HZ_200		200		// 输出率100HZ
//#define HZ_500		500		// 输出率100HZ
//#define HZ_1k		1000	// 输出率100HZ
//#define HZ_2k		2000	// 输出率100HZ

//#define FILTER_SIZE	5		// 滑动平均滤波窗口大小
//#define LOW_PASS_N	2		// 低通滤波阶数 2阶 f=1000HZ wc=0.2*10=2HZ
//#define MANUAL_PARA_ON 1	// 1启动手动赋值 0关闭

//typedef struct// 陀螺加计 温补及标定参数 (1+12+24+72+48+48+30+8+1)*8 = 244*8 = 1792 Byte
//{
//	// #OSN_IMU16488,0#
//	double Version;				// 例202500301
//	// #陀螺加计轴向极性#
//	double G_axis[D3];
//	double A_axis[D3];	
//	double G_pola[D3];
//	double A_pola[D3];
//	// #陀螺、加计交叉耦合及零偏#
//	double G_cross[D3][D3];
//	double G_bias[D3];
//	double A_cross[D3][D3];
//	double A_bias[D3];
//	// #陀螺标度因素非线性补偿参数#
//	double Gx_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
//	double Gy_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
//	double Gz_p3210_d5[K3210][GCUT + 1];	// 3阶 5段
//	double Gx_cut4[GCUT];			// 4分段点
//	double Gy_cut4[GCUT];			// 4分段点
//	double Gz_cut4[GCUT];			// 4分段点
//	// #陀螺温度补偿参数#
//	double GT_p3210_d4[D3][K3210 * D4];	// xyz 3阶4段
//	// #加计温度补偿参数#
//	double AT_p3210_d4[D3][K3210 * D4];	// xyz 3阶4段
//	// #陀螺、加计温度分段点参数#
//	double GT_cut5[D3][TCUT];				// xyz 1+3+1分段点
//	double AT_cut5[D3][TCUT];				// xyz 1+3+1分段点
//	// #温度修正参数#
//	double GTxyz_kb[D3][KB];	// xyz 温度自身系数 kb
//	double AT_kb[KB];			// 加计 温度自身系数 kb
//	// #参数生成时间#
//	double Time;				// 例250327
//	// $$
//} CALIB_PARA;
//extern CALIB_PARA  calib_papr;//头文件声明，需要源文件中定义


uint32_t read_cs_data(char *pbuf,uint8_t type);
uint8_t str_compare(char *s1, char *s2,uint32_t len);



