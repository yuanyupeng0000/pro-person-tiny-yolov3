/*
 * sig_service.h
 *
 *  Created on: 2016年6月22日
 *      Author: Administrator
 */

#ifndef INCLUDE_SIG_SERVICE_H_
#define INCLUDE_SIG_SERVICE_H_
#include "g_define.h"


#include <pthread.h>
#pragma pack(push)
#pragma pack(1)
//
//#define  VERSION    0x01     //版本类型
//#define  PROTTYPE    0x02         //协议版本
//
////网络协议类型
//#define  SETCAMPARAM    0x0001      //设置相机参数命名
//#define  GETCAMPARAM    0x0002		  //获取相机参数命名
//#define  REPCAMPARAM    0x0003	  //获取相机参数命名回应
//
//#define  SETDETECTDEVICE    0x0004    //设置检测设备参数命名
//#define  GETDETECTDEVICE    0x0005    //获取检测设备参数命名
//#define  REPDETECTDEVICE    0x0006    //获取检测设备参数命名回应
//
//#define  SETCHECHPARAM    0x0007    //设置视频画线参数
//#define  GETCHECHPARAM    0x0008    //获取视频画线参数
//#define  REPCHECHPARAM    0x0009    //获取视频画线参数回应
//
//#define  HEART    0x1000    //心跳包数据协议
//#define  SHUTDOWN    0x1001    //关闭命令
//#define  REPHEART    0x1002    //心跳包数据协议回应
//
//#define  REBOOTZEN   0x2001   //重启命令
//
//#define  FORKEXIT    0x3001   //程序异常退出
//
//#define   HEARTTIME     60
//
//#define  USERNAMEMAX   20
//#define  IPADDRMAX     16
//#define  DEVNAMEMAX    50
//
////相机方向代号
//#define Z_NONE 0x0
//#define Z_NORTH 0x1
//#define Z_NORTHEAST 0x2
//#define Z_EAST 0x3
//#define Z_SOUTHEAST 0x4
//#define Z_SOUTH 0x5
//#define Z_SOUTHWEST 0x6
//#define Z_WEST 0x7
//#define Z_NORTHWEST 0x8
//
//#define  COILPOINTMAX       4      //线圈四个顶点
//#define  DETECTLANENUMMAX   4      //检测通道个数
//#define  LANELINEMAX         8         //车道线个数
//#define  STANDPOINT          8        //标定定8个顶点
//#define  STANDARDVAULEMAX    4
//#define  ALGMAX               2    //算法参数个数
//#define  MAXSUBSYSTEM         4
//
////#define  CAM_NUM           4    //连接相机最大个数
//
//typedef struct Command{
//	unsigned char version;       //版本类型VERSION
//	unsigned char prottype;      //协议版本 PROTTYPE
//	unsigned short objnumber;      //对象类型. 下标index
//	unsigned short objtype;      //数据类型   例如: SETCAMPARAM
//	unsigned int objlen;         //数据体的长度
//}mCommand; //网络协议头
//
//typedef struct{
//    unsigned short x;
//    unsigned short y;
//}IVDPoint;
///*-----------------实时检测数据-----------------*/
//typedef struct{
//    unsigned char   state;                    //车道状态   //0出车  1入车
//    unsigned char   isCarInTail;              //前置线圈是否有车, 0出车, 1入车
//    unsigned short  queueLength;              //车队的长度
//    unsigned int    vehnum;                   //车辆总数
//    //unsigned int    speed; 					  //车辆的速度
//    unsigned short  uActualDetectLength;       //虚拟线圈的长度  //近线圈
//    unsigned short  uActualTailLength;			//虚拟线圈的长度  //远线圈
//    IVDPoint        LineUp[2];                //当前车的实际坐标 起始点和终点
//}mRealLaneInfo;
//
//typedef struct{
//    unsigned char   flag;          //数据标志0xFF
//    unsigned char   laneNum;       //实际车道个数
//    unsigned char   curstatus;      //  1 是白天, 2 是夜晚
//    unsigned char   fuzzyflag;                //视频异常状态
//    unsigned char   visibility;		           //能见度状态
//    unsigned short 	uDegreePoint[20][2];      //表定点的坐标. 0:x 1:y
//    //unsigned short 	uDegreePoint[4][2];      //表定点的坐标. 0:x 1:y
//    mRealLaneInfo   lane[DETECTLANENUMMAX];  //16
//}mRealStaticInfo;
//
//#if 0
////-----------------检测器全局配置------------------
//typedef struct CammerControl{
//	unsigned char start;                    //相机连接启用标志   启用0x1 不启用0x0
//	unsigned char camdirect;              //相机所属方位
//	unsigned char cammerIp[IPADDRMAX];    //相机IP地址
//}mCammerControl;
//
//typedef struct DetectDeviceConfig{
//	unsigned int  detectport;
//	unsigned char camnum;
//	unsigned char detectip[IPADDRMAX];
//	unsigned char detectname[DEVNAMEMAX];
//	mCammerControl  camcontrol[CAM_NUM];
//}mDetectDeviceConfig;   //检测器设备参数,包含4个检测相机,
//#endif
//typedef struct DetectDeviceConfig{
//	unsigned int  deviceID;   //检测器ID
//	unsigned int  detectport;
//	unsigned char camnum;
//	unsigned char mode;       //是否录像模式标志   1 录像, 0非录像模式，也就相机模式
//	//unsigned char camdirect1;              //相机所属方位
//	//unsigned char camdirect2;              //相机所属方位
//	//unsigned char camdirect3;              //相机所属方位
//	//unsigned char camdirect4;              //相机所属方位
//	//unsigned char cammerIp1[IPADDRMAX];    //相机IP地址
//	//unsigned char cammerIp2[IPADDRMAX];    //相机IP地址
//	//unsigned char cammerIp3[IPADDRMAX];    //相机IP地址
//	//unsigned char cammerIp4[IPADDRMAX];    //相机IP地址
//
//	unsigned char camstatus[CAM_NUM];
//	unsigned char camdirect[CAM_NUM];
//	unsigned char cammerIp[CAM_NUM][IPADDRMAX];
//
//	unsigned char detectip[IPADDRMAX];
//	unsigned char detectname[DEVNAMEMAX];
//}mDetectDeviceConfig;   //检测器设备参数,包含4个检测相机,
//
////-----------------界面配置----单个相机相关参数配置-------------
//typedef struct CamAttributes{
//	unsigned char direction;
//	unsigned int  camID;      //相机的ID
//	unsigned int  cammerport;
//	unsigned int  adjustport;
//	unsigned int  signalport;
//	unsigned char urlname[USERNAMEMAX];  //为访问的流文件名字
//	unsigned char username[USERNAMEMAX];
//	unsigned char passwd[USERNAMEMAX];
//	unsigned char cammerIp[IPADDRMAX];
//	unsigned char adjustIp[IPADDRMAX];
//	unsigned char signalIp[IPADDRMAX];
//}mCamAttributes; //相机的属性
//
//typedef struct CamDemarcateParam{
//	unsigned short cam2stop;
//	unsigned short camheight;
//	unsigned short lannum;    //车道数
//	unsigned short number;    //通道编号
//	unsigned short baselinelen;
//	unsigned short farth2stop;
//	unsigned short recent2stop;
//}mCamDemarcateParam; //相机标定参数
//
//typedef struct ChannelVirtualcoil{
//	unsigned short number;      //通道编号
//	unsigned short farthCoillen;
//	unsigned short recentCoillen;
//}mChannelVirtualcoil; //通道虚拟线圈的参数
//
//typedef struct CamParam{
//	unsigned char coilnum;     //通道数
//	mCamAttributes camattr;
//	mCamDemarcateParam camdem;
//	mChannelVirtualcoil channelcoil[DETECTLANENUMMAX];
//}mCamParam;

//---------------线圈配置------单个相机检测参数配置---------
//typedef struct Point{
//    unsigned short x;
//    unsigned short y;
//}mPoint;//点坐标
//
//typedef struct Line{
//    unsigned short startx;
//    unsigned short starty;
//    unsigned short endx;
//    unsigned short endy;
//}mLine; //线坐标
//
//typedef struct RearCoil{
//	mPoint RearCoil[COILPOINTMAX];  //占位检测线圈
//	mPoint FrontCoil[COILPOINTMAX]; //前置线圈
//}mChannelCoil;  //单通道虚拟线圈的位置
//
//typedef struct CamDetectLane{
//	unsigned char lanenum;                 //车道数
//	mChannelCoil virtuallane[DETECTLANENUMMAX];
//}mCamDetectLane;   //单个相机相关的车道数据, 相机检测检测通道虚拟线圈 //每一个车道包含两个线圈
//
//typedef struct VirtualLaneLine{
//	unsigned char lanelinenum;         //
//	mLine         laneline[LANELINEMAX];
//}mVirtualLaneLine;    //虚拟车道线 最大支持4个车道5根线
//
//typedef struct StandardPoint{
//	mPoint  coordinate;
//	unsigned short value;
//}mStandardPoint; //标定点的坐标和值
//
//typedef struct DemDetectArea{
//	mPoint  vircoordinate[STANDPOINT];
//	mPoint  realcoordinate[STANDPOINT];
//}mDemDetectArea;  // 标定坐标系8个点,虚拟点和实际坐标点
//
//typedef struct DetectParam{
//	unsigned short uTransFactor;
//	unsigned int   uGraySubThreshold;
//	unsigned int   uSpeedCounterChangedThreshold;
//	unsigned int   uSpeedCounterChangedThreshold1;
//	unsigned int   uSpeedCounterChangedThreshold2;
//	unsigned short  uDayNightJudgeMinContiuFrame;//切换时二值化阈值
//	unsigned short  uComprehensiveSens;//取背景的连续帧数
//	unsigned short  uDetectSens1;//判断是车头的最小行数
//	unsigned short  uDetectSens2;
//	unsigned short  uStatisticsSens1;
//	unsigned short  uStatisticsSens2;	//by david 20130910 from tagCfgs
//	unsigned short  uSobelThreshold;//sobel阈值
//	unsigned short  shutterMax;        // 1 2 3 4 5 6 7 8
//	unsigned short  shutterMin;        // 1 2 3 4 5 6 7 8
//}mDetectParam;
//
//typedef struct CamDetectParam{
//	unsigned int   timep[4]; //0 1代表凌晨 点1至点2   2 3代表 黄昏 点1至点2
//	mCamDetectLane detectlane;        //检测车道数
//	mVirtualLaneLine  laneline;      //用到的车道线
//	mStandardPoint standpoint[STANDARDVAULEMAX];       //标定点和坐标
//	mDemDetectArea area;              //标定的区域
//	mDetectParam detectparam[ALGMAX];   // 0  白天的参数,  1代表晚上参数
//}mCamDetectParam;  //相机的所有检测有关参数

//通用帧定义
enum LINKID {
    LINKID_ETH1=1,
    LINKID_ETH2,
    LINKID_UART1,
    LINKID_UART2,
    LINKID_UART3,
    LINKID_UART4,
    LINKID_UART5,
    LINKID_CAN,
    LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD,
    LINKID_BROARDCAST=0xFF
};
//协议类型
enum PROTOCOLTYPE {
    PROTOCOTYPE_VIDEO_DETECTOR=0x01,
    PROTOCOTYPE_MAX
};
//设备类
enum DEVICECLASS {
    DEVICECLASS_VIDEO_SEPERATED_MACHINE=0x01,
    DEVICECLASS_IPCAMERAL,
    DEVICECLASS_VIDEO_INTEGRATED_MACHINE,
    DEVICECLASS_MAIN_CONTROL_BOARD,
    DEVICECLASS_COMMUNICATION_BOARD,
    DEVICECLASS_BROADCAST=0x0F,
};
//通用帧头定义
typedef struct {
    unsigned char mHeaderTag;
    unsigned char mSenderLinkId;
    unsigned char mRecieverLinkId;
    unsigned char mProtocolType;
    unsigned char mProtocolVersion;
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mSenderDeviceID,mRecieverDeviceId;
    unsigned char mSessionID;
    unsigned char mDataType;
    //unsigned char pContent[1];
} FrameHeader;
//通用帧尾定义
typedef struct {
    unsigned char mXor;
    unsigned char mTailTag;
}FrameTail;
///////////////////////////////////////////////// //实时交通数据/////////////
typedef struct EachChannelPack {
    unsigned char mDetectChannelIndex;
    unsigned char mQueueLength;
    unsigned char mRealTimeSingleSpeed;
    struct {
        unsigned char bOccupyStatus0:1;
        unsigned char bOccupyStatus1:1;
        unsigned char bFix0:2;
        unsigned char flow:2;
        unsigned char bVehicleType:2;
    }mDetectDataOfHeaderVirtualCoil;
    unsigned char mHeadwayTimeOfHeaderVirtualCoil;
    unsigned char mOccupancyOfHeaderVirtualCoil;
    struct {
        unsigned char bOccupyStatus0:1;
        unsigned char bOccupyStatus1:1;
        unsigned char bFix0:6;
    }mDetectDataOfTailVirtualCoil;
    unsigned char mHeadwayTimeOfTailVirtualCoil;
    unsigned char mOccupancyOfTailVirtualCoil;
    struct {
        unsigned char bFix0:7;
        unsigned char bDataIsValid:1;
    }mWorkStatusOfDetectChannle;
}EachChannelPackm;

typedef struct {
    unsigned char mDetectChannelCount;
    EachChannelPackm EachChannelPack[1];
}RealTimeTrafficData;

/////// //设备工作状态查询命令 ////////////直接使用通用帧头 ///////////////////////////////////////////
//设备工作状态查询响应 /////////////////////////////////////////

typedef	struct{
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mCameralDeviceId;
    struct {
        unsigned char bNorth:1;
        unsigned char bEastNorth:1;
        unsigned char bEast:1;
        unsigned char bEastSouth:1;
        unsigned char bSouth:1;
        unsigned char bWestSouth:1;
        unsigned char bWest:1;
        unsigned char bWestNorth:1;
    }mCameralPosition;
    struct {
        unsigned char bWorkMode:2;
        unsigned char bBackgroundRefreshed:1;
        unsigned char bH264DecodeStatus:1;
        unsigned char bCameralOnLine:1;
        unsigned char bPictureStable:1;
        unsigned char bFix0:2;
    }mCameralStatus;
}EachCameralStatus;

typedef struct {
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mDetectMainMachineDeviceId;
    union {
        struct {
            unsigned char bRegisted:1;
            unsigned char bTimeCorrected:1;
            unsigned char bFix0:6;
        }mStatusWhenSeperatedMachine;
        unsigned char mFix0WhenIntegratedMachine;
    }uDetectMainMachineStatus;
    unsigned char mCameralCount;
    EachCameralStatus mEachCameralStatus[1];
}DeviceWorkStatusQueryResponse;

//////////////////////////////////////// //对时广播命令 ////////
typedef struct {
    unsigned int mUTCTime;
}TimeBroadcastCommand;

///////////////////// //高峰广播命令 /////////
typedef struct {
    unsigned char mIsRushHour;
}RushHourBroadcastCommand;

////////////////////// //通讯板参数查询命令 /////////////////// //直接使用通用帧头
/////////// //通讯板参数查询响应 ///
typedef struct {
    struct {
        unsigned short mListenPort; unsigned char mIp[4];
        unsigned char mNetmask[4]; unsigned char mGateway[4];
    }mEthernet[2];
    struct {
        unsigned char mWorkElectricLevel;
        unsigned char mBaudRate;
        unsigned char mDataBits;
        unsigned char mStopBits;
        unsigned char mParityBit;
    }mUart[5];
    unsigned char mCanBaudRate;
}CommunicationBoardArgumentQueryResponse;

///////////////////// //通讯板参数设置命令 ////////////
typedef struct {
    struct {
        unsigned short mListenPort;
        unsigned char mIp[4];
        unsigned char mNetmask[4];
        unsigned char mGateway[4];
    }mEthernet[2];
    struct {
        unsigned char mWorkElectricLevel;
        unsigned char mBaudRate;
        unsigned char mDataBits;
        unsigned char mStopBits;
        unsigned char mParityBit;
    }mUart[5];
    unsigned char mCanBaudRate;
}CommunicationBoardArgumentSetUpCommand;

///////////////////////////////// ////通讯板参数设置响应 ///////////////////////////
typedef struct {
    unsigned char mResult;
}CommunicationBoardArgumentSetUpResponse;
//////////////////////////////////////////////////////////////
//设备运行事件主动上报
typedef struct{
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mEventDeviceId;
    union {
        struct { unsigned char bNorth:1;
                 unsigned char bEastNorth:1;
                      unsigned char bEast:1;
                           unsigned char bEastSouth:1;
                                unsigned char bSouth:1;
                                     unsigned char bWestSouth:1;
                                          unsigned char bWest:1;
                                               unsigned char bWestNorth:1;
               }mEventOccourPositionWhenCameralOrIntegratedCameral;
        unsigned char mFix0WhenSeperatedMachine;
    }uEventOccourPosition;
    unsigned char mEvent;
    unsigned char mReserverFix0;
    unsigned int mEventTime;
}DeviceEventsAutoReporte;

#define CHANNELMAXNUM 4
typedef struct car_info {
    int g_flow[4] ={0};
    int g_50frame1[4]={0};
    int g_50frame2[4]={0};
    int g_50frametail1[4]={0};
    int g_50frametail2[4]={0};
    int g_occupancyframe[4]={0};
    int g_occupancyframetail[4]={0};
} m_car_info;
typedef struct staticchannel{
    //unsigned char index;
    unsigned char status;   //表示数据的有效性, 0 初始状态, 不发送状态, 1为在线状态, 2前面数据发送完毕.
    //unsigned char mode;
    //unsigned char black;
    //unsigned char algswitch;
    EachCameralStatus EachStatus;//Camera status
    EachChannelPackm Eachchannel[CHANNELMAXNUM];//This is what signal machine wants
    int camera_state_change=0;
    int lane_num;
    m_car_info car_info;
    //unsigned int  frameNum;
}m_sig_data;
extern m_sig_data * get_sig_data(int index);
/*
 * v1.2
 上报状态的条件：
1.响应信号机查询
2.相机数秒没数据给信号机（相当于判断断线）
3.信号机连接事件
4,。任意状态改变（背景刷新，图片不稳定，解码错误，工作模式，相机掉线）；
 *
 *

上报事件条件：
1.程序启动（工控机开机）
2.信号机连接成功
3.工控机重启
4.信号机断开
 *
 * */


void reset_sig_machine();
void init_sig_service();void init_sig_service1();
//void submit_sig_data(m_sig_data *p_channel_rst,int index);

extern m_sig_data * get_locked_sig_data(int index);
extern void submit_unlock_sig_data(int index);
void reboot_cmd();
void reset_sig_machine();

#define NANJING_CAM_NUM 4
typedef struct queue_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char detect_time[6];
    unsigned char detect_status;//whether device good or not
    unsigned char dir_no;//camera direction
    unsigned char lane_dir_type;//income or out come
    unsigned char queue_len[8];//---------
    unsigned char queue_start_pos[8];
    unsigned char queue_veh_num[8];//---------
    unsigned char veh_speed[8];//
    unsigned char crc;
}queue_info_t;



typedef struct flow_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char detect_time[20];
    unsigned char dir_no;
    unsigned char section_no;
    unsigned char flow[8];
    unsigned char average_speed[8];
    unsigned char ocuppy_percent[8];
    unsigned char crc;
}flow_info_t;

typedef struct outcar_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char pass_time[6];
    unsigned char dir_no;
    unsigned char lane_dir_type;
    unsigned char section_number;
    unsigned char lane_number;
    unsigned char veh_type;
    unsigned char veh_speed;
    unsigned int occupy_time;
    unsigned char crc;
}ourcar_info_t;
static int out_car_flag;
extern pthread_mutex_t out_car_lock;


typedef struct data_1s{
}data_1s_t;

#pragma pack(pop)

#endif /* INCLUDE_SIG_SERVICE_H_ */
