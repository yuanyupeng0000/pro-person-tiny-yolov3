#ifndef __NEW_ARITH_H__
#define __NEW_ARITH_H__
#include "DSPARMProto.h"
#include <sys/time.h>
#include "Python.h"
#include <opencv2/opencv.hpp>
using namespace cv;
//BY DAVID 20130322 FOR xDM
//#define		TRUE	1
//#define  	FALSE	0
/*
#ifdef WX_IVD_EXPORTS
#define CIVD_SDK_API extern "C" __declspec(dllexport)
#else
#define CIVD_SDK_API extern "C" __declspec(dllimport)
#endif
*/

#define CIVD_SDK_API

typedef  unsigned int 	Uint32;
typedef  int 			Int32;
typedef  unsigned short Uint16;
typedef  short 			Int16;
//typedef  unsigned short bool;
//typedef  unsigned short bool;
typedef  unsigned char	Uint8;
extern int frame;
//typedef  signed char	Int8;

//#define DETECTRECT_WIDTH_MAX		720//500
#define DETECTRECT_WIDTH_MAX		704//500
#define DETECTRECT_HEIGHT_MAX		576//240

#define DETECTRECT_WIDTH_DEFAULT	160
#define DETECTRECT_HEIGHT_DEFAULT	40
#define LANE_AMOUNT_DEFAULT			3
#define DETECT_WIDTH			200
#define DETECT_HEIGHT			120
#define DETECT_ALGORITHM			3

#ifndef __UPLOAD_DETECT__
#define __UPLOAD_DETECT__

#define TRUE 1
#define FALSE 0
#define	VISIB_LENGTH		10//txl,20160105
#define MAX_CLASSES 80
#define MAX_TARGET_NUM 100
#define MAX_DIRECTION_NUM 2
#define FRAME_FPS  10
#define MAX_LANE_TARGET_NUM 20
#define MAX_CAMERA_NUM 6
typedef struct LaneInitial 
{
	Uint16 		uTransFactor;
	Uint32		uGraySubThreshold;
	Uint32 		uSpeedCounterChangedThreshold;
	Uint32 		uSpeedCounterChangedThreshold1;
	Uint32 		uSpeedCounterChangedThreshold2;


	Uint16		uDayNightJudgeMinContiuFrame;//持续帧数
	Uint16		uComprehensiveSens;//综合灵敏度
	Uint16		uDetectSens1;//检测灵敏度1
	Uint16		uDetectSens2;
	Uint16		uStatisticsSens1;
	Uint16 		uStatisticsSens2;	//by david 20130910 from tagCfgs
	Uint16		uSobelThreshold;//sobel锟斤拷值
	Uint16		uEnvironmentStatus;//时间，黄昏、晚上、黎明、白天1,2,3,4
	Uint16      uEnvironment;           //时间 白天和晚上 执行算法类别
    float       base_length;//基准线长度
	float       near_point_length;//近距点距离
}LANEINISTRUCT;//702

typedef void            XDAS_Void;
typedef Uint8           XDAS_bool;


typedef unsigned char           XDAS_Int8;      /**< Actual size chip dependent. */
typedef Uint8                   XDAS_UInt8;     /**< Actual size chip dependent. */
typedef Int16                   XDAS_Int16;     /**< Actual size of type is 16 bits. */
typedef Uint16                  XDAS_UInt16;    /**< Actual size of type is 16 bits. */
typedef Int32                   XDAS_Int32;     /**< Actual size of type is 32 bits. */
typedef Uint32                  XDAS_UInt32;    /**< Actual size of type is 32 bits. */
typedef struct XDM1_SingleBufDesc {
    XDAS_Int8   *buf;       /**< Pointer to a buffer address. */
    XDAS_Int32  bufSize;    /**< Size of @c buf in 8-bit bytes. */
    XDAS_Int32  accessMask; /**< Mask filled by the algorithm, declaring
                             *   how the buffer was accessed <b>by the
                             *   algorithm processor</b>.
                             *
                             *   @remarks  If the buffer was <b>not</b>
                             *             accessed by the algorithm
                             *             processor (e.g., it was filled
                             *             via DMA or other hardware
                             *             accelerator that <i>doesn't</i>
                             *             write through the algorithm's
                             *             CPU), then no bits in this mask
                             *             should be set.
                             *
                             *   @remarks  It is acceptible (and
                             *             appropriate!)to set several
                             *             bits in this mask if the
                             *             algorithm accessed the buffer
                             *             in several ways.
                             *
                             *   @remarks  This mask is often used by the
                             *             application and/or framework
                             *             to appropriately manage cache
                             *             on cache-based systems.
                             *
                             *   @sa XDM_AccessMode
                             */
} XDM1_SingleBufDesc;
typedef struct IVIDEO1_BufDescIn {
    XDAS_Int32  numBufs;        /**< Number of buffers in bufDesc[]. */
    XDAS_Int32  frameWidth;     /**< Width of the video frame. */
    XDAS_Int32  frameHeight;    /**< Height of the video frame. */
    XDAS_Int32  framePitch;     /**< Frame pitch used to store the frame.
                                 *
                                 *   @remarks   This field can also be used to
                                 *              indicate the padded width.
                                 */
    XDM1_SingleBufDesc bufDesc[8]; /**< Picture buffers. */
} IVIDEO1_BufDescIn;

/*typedef struct tagDetectEvent
{
	int	 EventCode;
	bool EventFlag;

}DETECTEVENT;*/

typedef struct tagOutBuf
{

	int DetectInSum[8];
	int DetectOutSum[8];
	//DETECTEVENT DetectEvent[8];
	unsigned int thresholdValue;
	unsigned int frame;
	unsigned int calarflag[8];
	unsigned int AlarmLineflag[8][5];
	CPoint LineUp[8][2];
	bool IsCarInTail[8];//尾部占有状态
	//unsigned int LineUp[8][2];
	Uint16	uVehicleSpeed[8];			//车辆速度
	Uint16  uVehicleType[8];            //车辆类型
	Uint16	uVehicleLength[8];			//车辆长度
	Uint16	uVehicleHeight[8];			//车辆高度
	Uint16 uVehicleQueueLength[8];//排队长度
	bool fuzzyflag;//视频异常状态
	bool visibility;//能见度状态
    Uint16 uActualDetectLength[8];
    Uint16 uActualTailLength[8];
    Uint16 uDegreePoint[20][2];
	Uint16	uEnvironmentStatus;
	bool getQueback_flag[8];//txl,20160104
	int DetectRegionVehiSum[8];//new  区域车辆数
	CPoint QueLine[8][2];//new   排队长度线点
	Uint16 uVehicleQueueLength1[8];//车排队长度
	CRect udetBox[100];
	Uint16 udetNum;//检测框
	CRect udetPersonBox[100];
	Uint16 udetPersonNum;//行人检测框
	Uint16 udetStatPersonNum;//统计行人检测框
}OUTBUF;
#endif


/*锟斤拷锟斤拷锟侥结构*/
#define	CAMERA_STRUCT_LENGTH	64

typedef struct tagCAMERASTRUCT
{
	Uint16	CameraId;
	Uint16	DetectAmount;				
	/*			                                              
    Uint16	MaxBrightDeviationLimit;                                                    
    Uint16	MinBrightDeviationLimit;                                                    
	Uint16	MaxBrightDiffLimit;          				                                                                                          
    Uint16	MinBrightDiffLimit;                                                                                          
    Uint16	Bt835Bright;                                        
    Uint16	Bt835Contrast;                                          
    Uint16	Bt835Sat_U;                                       
    Uint16	Bt835Sat_V;                                       
    Uint16	Bt835Hue;                                     
    Uint16	Resv[15];                                     
    bool			bCameraEventHasSent;	                                           
    bool			bCameraEventHappened;                                           
    Uint16	bCounterPole;*/                                         
    Uint16			bCameraWorkInCross; //txl,20150709,bool->Uint16                                 
    Uint16	LaneAmount;                                       
    Uint16	LanesId[8];                                        

	//added by david 20131011
	Uint16		uEnvironmentStatus;		//added by david 20131011

	Uint16		uDayNightJudgeMinContiuFrame;//取锟狡癸拷时锟斤拷为锟角灯癸拷幕叶锟斤拷锟斤拷锟斤拷锟斤拷拥锟斤拷锟斤拷锟街★拷锟�
	Uint16		uGetBackMinContiuFrame;//取锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷帧锟斤拷
	Uint16		uVehicleHeadMinRow;//锟叫讹拷锟角筹拷头锟斤拷锟斤拷小锟斤拷锟斤拷
	Uint16		uInThreshold;
	Uint16		uSquareThreshold;
	Uint16 		guSubImageThreshold;	//by david 20130910 from tagCfgs
	Uint16		uSobelThreshold;//sobel锟斤拷值
	
	Uint16		Resv2[1];
}CAMERA_STRUCT;                                          
                	            				                   

//#ifndef __ALG_CPOINT__
//#define __ALG_CPOINT__
//typedef struct 
//{
//	Uint16 x;
//	Uint16 y;
//}CPoint; 
//#endif
////20140102
#ifndef __ALG_CRECT__
#define __ALG_CRECT__   

/*typedef struct RECT_Obj
{
//	struct RECT_Fxns *rectfxns;
	
    
    Int16    left;
    Int16    top;
    Int16    right;
    Int16    bottom;
} CRect;*/
/*
typedef struct RECT_FXNS
{
	void (*Init)(CRect handle);
	void (*Free)(CRect handle);
	int (*Width)(CRect handle);
	int (*Height)(CRect handle);
}RECT_Fxns;
*/
#endif

typedef struct tagSPEED_DETECTOR_WORKMODE
{
	Uint16			uReserve			:13;
	bool			bDay				:1;
	bool			bNight				:1;
	bool			bAuto				:1;
}SPEED_DETECTOR_WORKMODE;

typedef struct tagDOWNLOAD_DETECT
{
	Uint16	uID;
	CPoint			ptFourCorner[8];
	Uint16		uDayNightJudgeMinContiuFrame;
	Uint16		uGetBackMinContiuFrame;
	Uint16		uVehicleHeadMinRow;
	Uint16		uVehicleHeadMinCol;
	Uint16		uSobelThreshold;
	SPEED_DETECTOR_WORKMODE SpeedDetectorWorkMode;
}DOWNLOAD_DETECT;
                                 
typedef struct tagSPEED_COUNTER_STATUS						/*锟劫讹拷探锟斤拷锟斤拷募锟斤拷锟阶达拷*/
{
	Uint16			uReserve			:11;
	bool			bBeginCount			:1;					/*锟斤拷头一锟斤拷末尾锟斤拷锟斤拷锟皆匡拷始锟斤拷锟斤拷*/
	bool			bCanBeginCount		:1;			/*锟斤拷锟街★拷压锟斤拷锟皆匡拷始锟斤拷锟姐车头锟角否到达拷撞锟�*/
	bool			bLeftDestroyed		:1;
	bool			bRightDestroyed		:1;
	bool			bIsCounting			:1;
}SPEED_COUNTER_STATUS;

#define		SPEEDINFO_MAXSIZE	7
#define		LENGTHINFO_MAXSIZE	4

typedef struct cSpeedDetectStruct
{
	Uint16		DetectId;								
	Uint16		DetectDots;
	Uint16	    QueDetectDots;
	Uint16		DetectLines;
	Uint16		DetectColumns;
	Uint16		QueDetectLines;
	Uint16		QueDetectColumns;
	Uint32		*CoordinatePointer;
	Uint32		*CoordinatePointerQ;
	Uint16		*CurrentImagePointer;
	Uint16		*BackImagePointer; 
	Uint16		*puTempImage;

	Uint16		*CurrQueueImage;
	Uint16		*PreQueueImage; 
	Uint16		*BackQueueImage;
	Uint16      *PrePreQueueImage;
	Uint16      *PrePrePreQueueImage;
					
	SPEED_COUNTER_STATUS	SpeedCounterStatus;	
	SPEED_DETECTOR_WORKMODE SpeedDetectorWorkMode;	

	bool getQueback_flag;//txl,20160104
	Uint16		uReserve[37];
}SPEED_DETECT_STRUCT;		/* total struct occupy 128 words */

struct	cSpeedStructIndex
{
	Uint16	SpeedId;
	struct cSpeedDetectStruct	*ExtStorageAddr;
};


struct	cCountStructIndex
{
	Uint16	CountId;
	struct cCountDetectStruct	*ExtStorageAddr;								
};

typedef	struct cPresenceDetectStruct
{
	Uint16	DetectId;
	Uint16	DetectDots;
	Uint16	DetectLines;
	Uint16	DetectColumns;
	Uint32	*CoordinatePointer;
	Uint16	*CurrentImagePointer;
	Uint16	*BackImagePointer; 
}PRESENCE_DETECT_STRUCT;		/*All words are 120+16=136*/

struct	cPresenceStructIndex
{
	Uint16	PresenceId;
	struct cPresenceDetectStruct	*ExtStorageAddr;
};

typedef	struct	tagCameraParamaters
{
	struct	cSpeedStructIndex		dSpeedIdIndex[8];		
	struct	cCountStructIndex		dCountIdIndex[8];		
	struct	cPresenceStructIndex	dPresenceIdIndex[32];			
	Uint8			*DetectCfgEntry[32];   		
	unsigned int			DetectStateFlag[4]; 		                                  
	unsigned int			nPresenceDetectAmount;		                                    
	SPEED_COUNTER_STATUS 	guSpeedCounterStatus[8];	  

	//extended by david 20130422:
	Uint8		 	*CameraCfgEntry;
	Uint8		 	*ImageStorageEntry;
	Uint16			frameWidth;
	Uint16			frameHeight;
	Uint32			bNormalDetectEnable;		//bit0:锟斤拷锟节硷拷锟斤拷锟绞癸拷锟�; bit1:锟斤拷锟接筹拷锟饺硷拷锟斤拷锟绞癸拷锟�; 锟斤拷锟斤拷锟斤拷
}CAMERA_PARAMETERS;

union	NormalDetect
{       
	struct	cSpeedDetectStruct	cSpeedDesc;
	//struct	cCountDetectStruct	cCountDesc;	
	struct  cPresenceDetectStruct cPresenceDesc;
}; 

typedef struct{
	cv::Rect box;	
	float prob;
	int class_id;
	char names[50];
	bool detected;
	int  lost_detected;
	int target_id;
	cv::Point trajectory[3000];
	int trajectory_num;
	int vx;//速度
	int vy;
	int continue_num;
	float start_time;
	float end_time;//用于计算车速
	int lane_id;
	bool cal_speed;
	bool cal_flow;
}CTarget;

typedef struct{
	int class_id;
	float prob[100];
	cv::Rect box[100];
	char names[50];
	int classes_num;
	int lane_id[100];//框所在的车道
}CDetBox;

typedef struct tagCfgs
{
	CAMERA_STRUCT 			CameraCfg;			//锟斤拷锟斤拷全锟斤拷锟斤拷锟斤拷
	CAMERA_PARAMETERS		CameraLocalPara;	//
	RESULTMSG				ResultMsg;			//
	ZENITH_SPEEDDETECTOR	DownSpeedCfg;		

	Rect  detectROI;//检测区域
	char** names;
	int classes;//检测类别数
	CTarget targets[MAX_TARGET_NUM];//目标
	CDetBox detClasses[MAX_CLASSES];//检测类别
	int target_id;
	int targets_size;
	CTarget detTargets[MAX_TARGET_NUM];//用于车道内车辆数量统计
	int detTarget_id;
	int detTargets_size;
	Uint16 current_target_id[MAX_LANE];
	Uint16 headtime[MAX_LANE];//用于计算车头时距
	Uint16 jgtime[MAX_LANE];
	Uint16 Headposition[MAX_LANE];
	Uint16 Tailposition[MAX_LANE];
	Rect detBoxes[MAX_LANE][MAX_LANE_TARGET_NUM];
	Uint16 detNum[MAX_LANE];
	Uint16 uStatPersonNum[4];//统计行人数，采用四帧平均值
	int uDetectRegionNum;//行人检测区域数
	Uint16 uRegionPersonNum[MAX_REGION_NUM];//每个区域的行人数
	double currIime;//用于计算车辆速度
	struct timeval time_start;
	struct timeval time_end;
	Uint16	m_iWidth, m_iHeight;
	Uint16	team_width;
	Uint16  team_height;
	bool    IsCarInTail[MAX_LANE];//txl,1126

	bool    bNight;
	Uint16  bAuto;

	Uint32 	gThisFrameTime;
	Uint32  abnormal_time;

	float 	gdAverGrey;
	float 	gdWrap;
	float	gdSum;

	//added by david 20130422
	Uint32	uDetectInSum[MAX_LANE];		//进入流量线圈的车流量
	Uint32	uDetectOutSum[MAX_LANE];	//出流量线圈的车流量
	Uint32  uDetectVehicleSum[MAX_LANE];//车道内的车辆总数
	Uint16  uStatVehicleSum[MAX_LANE][4];//用于统计车道内的车辆数
	Uint16  uStatQuePos[MAX_LANE][6];//用于统计排队长度
	Uint16  uVehicleQueueLength[MAX_LANE];
	CPoint  m_ptend[MAX_LANE][12];
	float mapping_matrix[9];
	//图像标定
	float image_actual[FULL_ROWS][FULL_COLS][2];
	float actual_point[CALIBRATION_POINT_NUM][2];
	float image_point[CALIBRATION_POINT_NUM][2];
	int calibration_point[4][2];//标定点
	int base_line[2][2];//基准线端点
	float base_length;//基准线长
	float near_point_length;//近距点距离
	float actual_distance[8][FULL_ROWS];
	Uint16 uActualDetectLength[8];//实际流量线圈长度
	Uint16 uActualTailLength[8];//实际尾部占有线圈长度
	float actual_degree_length[FULL_ROWS];//实际刻度线对应的距离
    Uint16 degreepoint[20][2];//刻度点

	Uint16 fuzzydegree;//624xyx
	Uint16 fuzzytimes;//630xyx
	bool fuzzyflag;
	Uint16 visitimes;
	bool visibility;//能见度状态
    int up_visib_value;//txl,20160105
	Uint16 visib_value[VISIB_LENGTH];//txl,20160105
	
	PyObject *pModule, *pFunc; //调用Python
	PyObject *g_graph;
	int NCS_ID;//计算棒标识
}ALGCFGS;

typedef struct tagParams
{
	union	NormalDetect	NormalDetectCfg;

	Uint8	*puPointNewImage;   ///锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷谋锟斤拷锟�
	Uint8 	*puBackImagePlus;	//用于隔两帧数据
	//txl,0630
	Uint8	*CurrQueueImage;  //锟脚队硷拷锟侥憋拷锟斤拷
	Uint8	*PreQueueImage;
	Uint8	*BackQueueImage;
	Uint8   *PrePreQueueImage;
	Uint8   *PrePrePreQueueImage;
	Uint8   *MaskDetectImage;//检测区域掩模图像	
}ALGPARAMS;

typedef struct tagVIDENC1COPY_TI_OBJ{
    
	ALGCFGS		*pCfgs;		
	ALGPARAMS	*pParams; 
	
} VIDENC1COPY_TI_Obj;

extern  SPEEDCFGSEG    pDetectCfgSeg;
extern  ALGCFGS        pCfgs;
extern  ALGPARAMS      pParams;
extern  CFGINFOHEADER  pCfgHeader; 
extern  unsigned char	Image[7][1600*1200];
extern  Uint16	ImageStorageEntry[8][8][1600*1200];
extern  bool  flag;
////////////////////////////////////////////////////////////
extern Uint16 ArithProc(Uint16 ChNum, IVIDEO1_BufDescIn * inBuf, RESULTMSG* outBuf, Int32 outSize, ALGCFGS *pCfgs, ALGPARAMS *pParams,int* result, int nboxes);//,CPoint LineUp1[],CPoint m_ptend1[]
extern bool Color_deviate(Uint8 *uImage,Uint8 *vImage,int width,int height);////txl,20150714
extern bool ArithInit(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams);//,CPoint m_ptend[],CPoint LineUp[]

extern bool ArithVariableInit(Uint16 ChNum, ALGCFGS *pCfgs, ALGPARAMS *pParams);
static bool CfgStructParse(Uint16 ChNum, CFGINFOHEADER *pCfgHeader, SPEEDCFGSEG *pDetectCfgSeg, ALGCFGS *pCfgs, ALGPARAMS *pParams);//,CPoint m_ptend[],CPoint LineUp[]
static CPoint ptGetDot(CPoint* ptUpLeft, CPoint* ptUpRight, CPoint* ptDownRight, CPoint* ptDownLeft, Int16 nColNum, Uint32 * ptStorePlace);
static float GetLenOfTwoPoint(CPoint* Cmp_Dot1, CPoint* Cmp_Dot2);
static void StoreCurImage(Uint8 *inBuf, ALGCFGS *pCfgs, ALGPARAMS	*pParams);
static void SpeedCaculate(Uint16 LaneID, ALGCFGS *pCfgs, ALGPARAMS	*pParams, CPoint m_ptend[]);
static Uint16 GetImagePointValue(Int16 iCol,Int16 iRow,Uint16 Lines,Uint16 Columns,Uint8 *ImagePtr);
static void iSubStractImage(Uint8 *puSourceImage,Uint8 *puTargetImage, Uint32 nThreshold, Int16 nFromLine, Int16 nToLine,Int16 width,Int16 height);

typedef struct args{
	ALGCFGS *pCfgs;
	ALGPARAMS *pParams;
	OUTBUF *p_outbuf;
	SPEEDCFGSEG    pDetectCfgSeg;
	CFGINFOHEADER  pCfgHeader;

    CPoint m_ptEnd[48];
    CPoint ptimage[8];
	int NCS_ID;

}m_args;

bool transform_init_DSP_VC(bool iniflag, Uint16 lanecount,
		LANEINISTRUCT LaneIn,RESULTMSG *p_outbuf,m_args *p_arg);
CIVD_SDK_API unsigned int transform_Proc_DSP_VC(unsigned char  *pInFrameBuf,
		unsigned char *pInuBuf,unsigned char *pInvBuf,\
		int nWidth, int nHeight, int hWrite,RESULTMSG *p_outbuf,m_args *p_arg);
int transform_arg_ctrl_DSP_VC(m_args *);
int transform_release_DSP_VC(m_args *);
extern unsigned int transform_Proc_0_DSP_VC(unsigned int* count,float* speed,float* length);

//static void camera_calibration(float actual_point[][2],float img_point[][2],float mapping_matrix[],int calibration_num,ALGCFGS *pCfgs);
//static void img_to_actual(float mapping_matrix[],int start_row,int end_row,int overlap_row1,int overlap_row2,int flag,ALGCFGS *pCfgs);
static void camera_calibration(int base_line[][2],float base_length,int calibration_point[][2],float near_point_length,int laneNum,ALGCFGS *pCfgs);
static void get_actual_point(float actual_point[2],int row,int col,int limit_line,ALGCFGS *pCfgs);
static float distance_two(float actual_point1[2],float actual_point2[2]);

static float fuzzy(unsigned char* puNewImage,int nWidth,int nHight);//624xyx
static bool Visibility(unsigned char* puNewImage,int nWidth,int nHight,ALGCFGS *pCfgs);//702xyx
static bool visible_judge(Uint16 *a,int visib_length,int threshold);//txl,20160105

int alg_mem_malloc(m_args *);
int alg_mem_free(m_args *arg_arg);
extern Uint16 ArithDetect(ALGCFGS* pCfgs, unsigned char* pInFrameBuf, unsigned char* pInuBuf, unsigned char* pInvBuf, int width, int height, int* result);
#endif



