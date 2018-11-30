///////////////////////////////////////////////////////
//		DSPARMProto.h
//
//		dsp��armͨѶЭ��
//  	BY DAVID 
//      20130512
//		VER: 1.01.00.00
///////////////////////////////////////////////////////

#ifndef __DSP_PROTO_H__
#define __DSP_PROTO_H__

#define MAX_DETECTOR_TYPES		2			//���֧�����ּ����
#define MAX_DETECTOR_ONETYPE	8			//ÿ�ּ�������֧��8��
////20131222
#define MAX_LANE		8	//	8			//ÿ����������֧��8������

//#define FULL_COLS  					(720)
#define FULL_COLS  					(640)
#define FULL_ROWS  					(480)
#define  CALIBRATION_POINT_NUM   8  //�궨����2015102
//////////////////////////////////////////////////
//		�ṹ����: 	�����������Ϣ�ṹ
/////////////////////////////////////////////////

#ifndef __ALG_POINT__
#define __ALG_POINT__

typedef  unsigned int 	Uint32;
typedef  int 			Int32;
typedef  unsigned short Uint16;
typedef  short 			Int16;
typedef  int             BOOL;
typedef  unsigned char	Uint8;

typedef struct 
{
	Uint16 x;
	Uint16 y;
}CPoint;
#endif

#ifndef __ALG_RECT__
#define __ALG_RECT__
typedef struct 
{
	Uint16 x;
	Uint16 y;
	Uint16 width;
	Uint16 height;
}CRect;
#endif


typedef struct tagSPEEDLANE
{
	Uint16				uLaneID; //������
	//detect region
	CPoint				ptFourCorner[6];//�ĸ��������
	CPoint				ptCornerQ[2];//�Ŷ�ǰ����
	CPoint				ptCornerQA[2];//������
	CPoint				ptCornerLB[2];//20150918
	CPoint				ptCornerRB[2];//20150918
	Uint16				uDetectDerection;//����
	Uint16				ptFrontLine;//Ǯֱ��
	Uint16				ptBackLine;//��ֱ��
	Uint16				uReserved0[30];//����

	//vehicle length and speed transformation params
	Uint16				uTransFactor;//ת��ϵ��
	
	//extended params by david 20130904
	Uint32 			uSpeedCounterChangedThreshold;
	Uint32 			uSpeedCounterChangedThreshold1;
	Uint32 			uSpeedCounterChangedThreshold2;
	Uint32				uGraySubThreshold;//�ҶȲ��ֵ��
	
	
	Uint16				uTrackParams[20];		//Ԥ��
	Uint16				uReserved1[20];			//Ԥ��
}SPEEDLANE;

/*�������ڼ��������,start*/
typedef struct tagZENITH_SPEEDDETECTOR {
	
	Uint16				uLaneTotalNum;//��������
	SPEEDLANE 				SpeedEachLane[MAX_LANE];
	Uint16				uEnvironmentStatus;			//����״̬, 1������  2����������(����) 3������·�� 4��������·�� 0������  //20130930 by david
	//alg params
	Uint16				uDayNightJudgeMinContiuFrame;	//����ת�������� 15
	Uint16				uComprehensiveSens;		//�ۺ������� 60
	Uint16				uDetectSens1;			//���������1	 20	
	Uint16				uDetectSens2;			//���������2    10
	Uint16				uStatisticsSens1;		//ͳ��������1   15
	Uint16				uStatisticsSens2;		//ͳ��������2   3
	Uint16				uSobelThreshold;		//��ֵ������    3
	Uint16             uEnvironment;           //20140320,���� ���ϲ���������ת��
	CPoint					ptactual[8];//�궨��
	CPoint					ptimage[8];//�궨��	
        //CPoint calibration_point[4];//�궨�����
	//CPoint base_line[2];//�궨��׼�ߵ�
	float base_length;//��׼�߳�
	float near_point_length;//��������
	Uint16				uReserved1[10];			//Ԥ��   
}ZENITH_SPEEDDETECTOR;
/*�������ڼ��������,end*/


/*���ӳ��ȼ����,start*/
typedef struct tagPRESENCELANE {
	Uint16 uReserved[256];
}PRESENCELANE;

typedef struct tagZENITH_PRESENCEDETECTOR {
	
	Uint16			uLaneTotalNum;
	PRESENCELANE		PresenseEachLane[MAX_LANE];
}ZENITH_PRESENCEDETECTOR;
/*���ӳ��ȼ����,end*/

//////////////////////////////////////////////////
//		�ṹ����: 	��Ϣ���� arm ---> dsp
/////////////////////////////////////////////////

typedef struct tagSpeedCfgSeg {
	Uint16				uType;		//���������:1���������ڼ����,2�����ӳ��ȼ����,3��256 ����,0���޼����
	Uint16				uNum; 		//���������
	ZENITH_SPEEDDETECTOR	uSegData[1];//ͬ�������м��������������
} SPEEDCFGSEG;

typedef struct tagPresenceCfgSeg {
	Uint16				uType;		//���������:1���������ڼ����,2�����ӳ��ȼ����,3��256 ����,0���޼����
	Uint16				uNum; 		//���������
	ZENITH_PRESENCEDETECTOR	uSegData[1];//ͬ�������м��������������
} PRESENCECFGSEG;

//union NormalCfgSeg {
//	SPEEDCFGSEG  		cSpeedCfgSeg;
//	PRESENCECFGSEG		cPresenceCfgSeg;
//};

typedef struct tagMsgHeader {
    Uint16   	uFlag;
    Uint16  	uCmd;
    Uint16  	uMsgLength;		
} MSGHEADER;

typedef struct tagConfigInfoHeader {
    Uint16		uDetectPosition;
    Uint16		uDetectFuncs[2];	//
} CFGINFOHEADER;

typedef struct tagCfgMsg {
	MSGHEADER		uMsgHeader;
	CFGINFOHEADER	uCfgHeader;			
} CFGMSG;
//////////////////////////////////////////////////
//		�ṹ����: 	��Ϣ����  dsp ---> arm
//////////////////////////////////////////////////
typedef struct tagSpeedDetectInfo {
	BOOL		bInfoValid;				//����������Ч
	Uint16	bVehicleSta;			//���복��״̬
	CPoint		ptVehicleCoordinate;	//����λ��
	Uint16	uVehicleSpeed;			//����
	Uint16	uVehicleLength;			//����
	Uint16	uVehicleType;			//����
	Uint16  uVehicleDirection;       //�������з���
	Uint16	uVehicleHeight;			//����
	Uint16	uVehicleHeadtime;		//��ͷʱ��
	Uint16 uVehicleQueueLength;    //�Ŷӳ���
	unsigned int calarflag;
	unsigned int car_in;
	unsigned int car_out;
	CPoint	LineUp[2];
	int AlarmLineflag;
	bool     IsCarInTailFlag;    //β������ռ�б�־
	bool     getQueback_flag;	//txl,20160104
	Uint16 uDetectRegionVehiSum; //��������
	Uint16 uVehicleQueueLength1; //�Ŷӳ���
	CPoint QueLine[2]; //�Ŷӳ�����
	Uint16	uReserved[20];			//Ԥ��
}SpeedDetectInfo_t;

typedef struct tagPresenceDetectInfo {
	BOOL		bInfoValid;				//����������Ч
	Uint16	uMotorCadeLength;		//���ӳ���
	CPoint		ptLisnceCoordinate[4];	//��������
	Uint16	uLisnceID[10];			//���ƺ���
	Uint16	uLisnceColor;			//������ɫ
	Uint16	uVehicleColor;			//������ɫ
	Uint16	uVehicleBrand;			//��������(����Ʒ��)
	Uint16	uSignalLightSta;		//�źŵ�״̬			
	Uint16	uEnvirenmentSta;		//����״̬			
	Uint16	uReserved[100];			//Ԥ��
}PresenceDetectInfo_t;

typedef struct tagResultData {
	Uint16			uLaneID;				//����ID
	Uint16			uReserved0;				//
	SpeedDetectInfo_t		SpeedDetectInfo1;
	PresenceDetectInfo_t PresenceDetectInfo;
	Uint16			uReserved[95];			//Ԥ�� 
}RESULTDATA;

typedef struct tagResultInfo {
	Uint16 		LaneSum;						//
	Uint16			uEnvironmentStatus;		//����״̬, 1������  2����������(����) 3������·�� 4��������·�� 0������  //20130930 by david
	CRect udetBox[100];
	Uint16 udetNum;//����
    RESULTDATA			uEachLaneData[8];				//����8���������еļ����Ϣ
} RESULTINFO;

typedef struct tagResultMsg {
    MSGHEADER		uMsgHeader;
    RESULTINFO 		uResultInfo;
} RESULTMSG;


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#endif