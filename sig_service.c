
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "client_net.h"
#include "sig_service.h"
#include "csocket.h"
#include "common.h"
#include "g_define.h"


#include <time.h>
#include <sys/time.h>
//#include "cam_alg.h"
unsigned char get_crc(unsigned char *buf,int sz)
{
    unsigned char crc=0;
    for(int i=0;i<sz;i++){
        crc^=*(buf+i);
    }
    return crc;
}
void get_outcar_info()
{

}
void get_time_string(char *buf , int len)
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);

    sprintf(buf,"%d-%d-%d %d:%d:%d",t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

}
int get_year()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_year+1900;
}
int get_year_tail()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return (t->tm_year+1900)%2000;
}
int get_month()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_mon+1;
}
int get_day()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_mday;
}
int get_hour()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_hour;
}
int get_min()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_min;
}
int get_sec()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_sec;
}

//pthread_mutex_t out_car_lock;
//pthread_mutex_t queue_lock;
//pthread_mutex_t flow_lock;

pthread_mutex_t mutex_lock;
typedef struct sig_holder{
    m_sig_data traffic_rst;
    pthread_mutex_t sig_data_lock;
}m_holder;
m_holder holder[ACTIVE_CAM_NUM];
int sig_state;
static pthread_mutex_t sig_state_lock;
enum{
    SIG_PRE_CONNECT,
    SIG_CONNECTED,
    SIG_NULL
};
int sig_fd;
char sig_ip[IP_LEN];
int sig_port;
//m_sig_data channel_rst[CAM_NUM];
void sig_set_state(int state)
{
    //	prt(stack,"set to %d",state);
    sig_state= state;
}
extern void submit_unlock_sig_data(int index)
{
    //	int ori_state=0;
    //	pthread_mutex_lock(&holder[index].sig_data_lock);
    ////	ori_state=holder[index].traffic_rst.camera_state_change;
    //	memcpy(&holder[index].traffic_rst,p_channel_rst,sizeof(m_sig_data));
    //	if(p_channel_rst->camera_state_change==1){
    //		prt(info,"state change ");
    //		p_channel_rst->camera_state_change=0;
    //	}
    pthread_mutex_unlock(&holder[index].sig_data_lock);
    //	prt(info,"index %d,src num %d,dst num %d",index,p_channel_rst->lane_num,holder[index].traffic_rst.lane_num);

}
extern m_sig_data * get_locked_sig_data(int index)
{
    pthread_mutex_lock(&holder[index].sig_data_lock);
    return &holder[index].traffic_rst;
}

int externalProtocolAddHeader(unsigned char *buff,int *size)
{
    buff[0]=0xC0;
    buff[*size+1]=0xC0;
    *size=*size+2;
    return 0;
}

void externalProtocolEncode(unsigned char *buff,int *size)
{
#if 0
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0XC0)
        {
            buff[i]=0XDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0XDC;

        }
        else if(buff[i]==0XDB)
        {
            buff[i]=0XDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0XDD;
        }
    }
#else
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0xC0)
        {
            buff[i]=0xDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0xDC;

        }
        else if(buff[i]==0xDB)
        {
            buff[i]=0xDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0xDD;
        }
    }
#endif
}

void externalProtocolDecode(unsigned char *buff,int *size)
{
#if 0
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0x7D&&buff[i+1]==0x5E){
            buff[i]=0x7E;
            //闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸犳宕愰幇鐗堢厸鐎癸拷鐎ｎ剛鐦堥悗瑙勬礃鐢帟鐏掗梺缁樿壘閻°劎锟芥艾缍婇弻鈥愁吋鎼粹�插闂佺懓鍢查崲鏌ワ綖濠靛鏁嗛柛灞剧敖閵娾晜鈷戦柛婵嗗椤箓鏌涢弮锟介崹鍧楃嵁閸愵喖顫呴柕鍫濇噹缁愭稒绻濋悽闈浶㈤悗姘间簽濡叉劙寮撮姀鈾�鎷绘繛杈剧悼閸庛倝宕甸敓浠嬫⒑閹肩偛锟芥牠鎮ч悩鑽ゅ祦闊洦绋掗弲鎼佹煥閻曞倹瀚�?闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷???闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷??闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }else if(buff[i]==0x7D&&buff[i+1]==0x5D){
            buff[i]=0x7D;
            //闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸犳宕愰幇鐗堢厸鐎癸拷鐎ｎ剛鐦堥悗瑙勬礃鐢帟鐏掗梺缁樿壘閻°劎锟芥艾缍婇弻鈥愁吋鎼粹�插闂佺懓鍢查崲鏌ワ綖濠靛鏁嗛柛灞剧敖閵娾晜鈷戦柛婵嗗椤箓鏌涢弮锟介崹鍧楃嵁閸愵喖顫呴柕鍫濇噹缁愭稒绻濋悽闈浶㈤悗姘间簽濡叉劙寮撮姀鈾�鎷绘繛杈剧悼閸庛倝宕甸敓浠嬫⒑閹肩偛锟芥牠鎮ч悩鑽ゅ祦闊洦绋掗弲鎼佹煥閻曞倹瀚�?闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷???闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷??闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
    }
    return;
#else
    int i=0,j=0;
    for(i=0; i<*size; i++)
    {
        if(buff[i]==0xDB&&buff[i+1]==0xDC)
        {
            buff[i]=0xC0;
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
        else if(buff[i]==0xDB&&buff[i+1]==0xDD)
        {
            buff[i]=0xDB;
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
    }
    //return TRUE;
#endif
}

int externalProtocolAddCrcCode(unsigned char *buff,int *size)
{
#if 0
    unsigned char xor_crc=0;
    int i=0;
    for(i=0;i<*size;i++)
        xor_crc^=buff[i];

    *size=*size+1;
    buff[*size-1]=xor_crc;

    if(buff[*size-1]==0xC0){
        buff[*size-1]=0xDB;
        *size=*size+1;
        buff[*size-1]=0xDC;
    }else if(buff[*size-1]==0xDB){
        buff[*size-1]=0xDB;
        *size=*size+1;
        buff[*size-1]=0xDD;
    }
#else
    unsigned char xor_crc=0;
    int i=0;
    for(i=0;i<*size;i++)
        xor_crc^=buff[i];
    *size=*size+1;
    buff[*size-1]=xor_crc;
#endif
    return 0;
}

int externalProtocolCheckCrc(unsigned char *buff,int size)
{
#if 0
    unsigned char xor_crc=0;
    unsigned char my_xor=0;
    int i=0;
    int len = 0;

    if(buff[size-1]==0xDB&&buff[size-2]==0xDC){
        my_xor = 0xC0;
        len = size-2;
    }else if(buff[size-1]==0xDB&&buff[size-2]==0xDD){
        my_xor = 0xDB;
        len = size-2;
    }else{
        len = size-1;
        my_xor = buff[size-1];
    }

    for(i=0;i<len;i++)
        xor_crc^=buff[i];

    if(xor_crc==my_xor){
        return 0;
    }else{
        return -1;
    }
#else
    char xor_crc=0;
    int i=0;
    for(i=0;i<size;i++)
    {
        xor_crc^=buff[i];
    }
    if(xor_crc==buff[size])
    {
        return 1;
    }
    else
    {
        return 0;
    }
#endif
}

int ReportedCammerStatus(int sock, int mSessionID)
{
    int size = 0;
    unsigned char  sendbuff[100];
    m_sig_data *p_fvdstaticchannel;//=&p_detect_ctx->cam_ctx[i].thread_param.channel_rst;
    //	unsigned char *sendbuff=NULL;
    //	mSystemConfig * fsys = g_pSyscfg;

    ///	sendbuff = (unsigned char *)malloc(100);
    if(sendbuff==NULL)
        return 0;
    size = 0;
    memset(sendbuff, 0, sizeof(sendbuff));
    FrameHeader * fnewhead = (FrameHeader*)sendbuff;
    fnewhead->mSenderLinkId = 0x0;
    fnewhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fnewhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fnewhead->mProtocolVersion = 0x12;
    fnewhead->mSenderDeviceID.mDeviceIndex = 0x1;
    fnewhead->mSenderDeviceID.mDeviceClass = 0x1;
    fnewhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fnewhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fnewhead->mSessionID = mSessionID;
    fnewhead->mDataType = 0x04;

    size = sizeof(FrameHeader)-1;
    DeviceWorkStatusQueryResponse *res=(DeviceWorkStatusQueryResponse *)(sendbuff+sizeof(FrameHeader));

    res->mDetectMainMachineDeviceId.mDeviceClass = 0x1;
    res->mDetectMainMachineDeviceId.mDeviceIndex = get_dev_id();
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bRegisted=0x1;
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bTimeCorrected=0x1;
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bFix0=0x0;
    res->mCameralCount = 0x0;
    size +=3;

    int i =0;
    for(i=0; i<ACTIVE_CAM_NUM; i++){
        p_fvdstaticchannel=&holder[i].traffic_rst;
        if(get_cam_status(i)){
            EachCameralStatus * cstatus=(EachCameralStatus*)(sendbuff+sizeof(FrameHeader)+3+res->mCameralCount*sizeof(EachCameralStatus));

            //	cstatus->mCameralDeviceId.mDeviceIndex = p_detect_ctx->cam_ctx[i].thread_param.cam_cfg.camattr.camID;

            cstatus->mCameralDeviceId.mDeviceIndex =get_cam_id(i);

            cstatus->mCameralDeviceId.mDeviceClass = DEVICECLASS_IPCAMERAL;
            //			prt(info,"get direction %d",get_cam_direction(i));
            switch(get_cam_direction(i)){

            case 0x1: cstatus->mCameralPosition.bNorth=0x1;break;

            case 0x2: cstatus->mCameralPosition.bEastNorth=0x1;break;

            case 0x3: cstatus->mCameralPosition.bEast=0x1;break;

            case 0x4: cstatus->mCameralPosition.bEastSouth=0x1;break;

            case 0x5: cstatus->mCameralPosition.bSouth=0x1;break;

            case 0x6: cstatus->mCameralPosition.bWestSouth=0x1;break;

            case 0x7: cstatus->mCameralPosition.bWest=0x1;break;

            case 0x8: cstatus->mCameralPosition.bWestNorth=0x1;break;

            default:  cstatus->mCameralPosition.bNorth=0x1;break;

            }
            cstatus->mCameralStatus.bWorkMode

                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bWorkMode;

            cstatus->mCameralStatus.bBackgroundRefreshed

                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bBackgroundRefreshed;

            cstatus->mCameralStatus.bH264DecodeStatus

                    =p_fvdstaticchannel->EachStatus.mCameralStatus.bH264DecodeStatus;

            if(0x1 == p_fvdstaticchannel->status || 0x2 == p_fvdstaticchannel->status)
                cstatus->mCameralStatus.bCameralOnLine=0x1;
            else
                cstatus->mCameralStatus.bCameralOnLine=0x0;
            cstatus->mCameralStatus.bPictureStable
                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bPictureStable;
            cstatus->mCameralStatus.bFix0
                    =p_fvdstaticchannel->EachStatus.mCameralStatus.bFix0;
            res->mCameralCount++;
            size+=sizeof(EachCameralStatus);
        }
    }


#if 0
    externalProtocolEncode(sendbuff+1,&size);
    externalProtocolAddCrcCode(sendbuff+1,&size);
    externalProtocolAddHeader(sendbuff,&size);
#else
    externalProtocolAddCrcCode(sendbuff+1,&size);
    externalProtocolEncode(sendbuff+1,&size);
    externalProtocolAddHeader(sendbuff,&size);
#endif
    int tmp=0;
    //			prt(stack,"=========report cam status======");
    //			for (tmp = 0; tmp < size; ++tmp) {
    //				prt(info,"%x",sendbuff[tmp]);
    //			}
    return SendDataByTcp(sock, (char *)sendbuff, size);
}

int RequestPro(unsigned char *buffer, int len, int sock)
{
    printf("get request\n");
    unsigned char *sendbuff=NULL;
    int size = 0;
    int i=0;
    while(i<len){
        printf("%x\n",buffer[i]);
        i++;
    }
    if(buffer[0] != 0xC0 || buffer[len-1] != 0xC0){
        return 0;
    }

#if 0
    size=len-2;
    if(externalProtocolCheckCrc(buffer+1,size)){
        return 0;
    }
    size-=1;
    externalProtocolDecode(buffer+1,&size);
    if((size+1) != sizeof(FrameHeader) && (size+1) != (sizeof(FrameHeader)+4)){
        return 0;
    }
#else
    size=len-2;
    printf("check crc \n\n");
    externalProtocolDecode(buffer+1,&size);
    if(externalProtocolCheckCrc(buffer+1,size)){
        printf("check crc err \n\n");
        return 0;
    }
#endif

    FrameHeader * fhead = (FrameHeader*)buffer;
    if(fhead->mSenderLinkId != LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD
            || fhead->mRecieverLinkId != LINKID_BROARDCAST
            || fhead->mProtocolType != PROTOCOTYPE_VIDEO_DETECTOR
            || fhead->mProtocolVersion != 0x12){
        return 0;
    }

    if(fhead->mDataType == 0x05){
        char strTime[24];
        //time_t UTCTime = ntohl(*((unsigned int*)(buffer+sizeof(FrameHeader))))+28800;
        time_t UTCTime = *((unsigned int*)(buffer+sizeof(FrameHeader)))+28800;
        struct tm *ppltime = gmtime(&UTCTime);
        strftime(strTime,24,"%F %T",ppltime);
        char cmd[50]={0};
        sprintf(cmd, "date -s \"%s\";hwclock -w", strTime);
        system(cmd);
        printf("reply tm\n");
    }else if(fhead->mDataType == 0x03){
        printf("rply  \n");
        return ReportedCammerStatus(sock,fhead->mSessionID);
    }
    return 0;
}
#include "camera_service.h"
int ReportedEvent(int sock, char event)
{
    int size;
    unsigned char mysbuf[100] = {0};

    memset(mysbuf, 0, sizeof(mysbuf));
    FrameHeader * fhead= (FrameHeader *)mysbuf;
    fhead->mSenderLinkId = 0x00;
    fhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fhead->mProtocolVersion = 0x12;
    //fhead->mSenderDeviceID.mDeviceIndex = g_pSyscfg->devparam.deviceID;
    fhead->mSenderDeviceID.mDeviceClass = 0x1;
    fhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fhead->mSessionID = 0xFF;

    fhead->mDataType = 0x10;
    size = sizeof(FrameHeader)-1;
    DeviceEventsAutoReporte* report = (DeviceEventsAutoReporte*)(mysbuf+sizeof(FrameHeader));

    report->mEventDeviceId.mDeviceIndex = get_dev_id();
    report->mEventDeviceId.mDeviceClass = 0x1;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bNorth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEastNorth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEast=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEastSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWestSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWest=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWestNorth=0x0;
    size+= sizeof(DeviceEventsAutoReporte);
    report->mEvent = event;
    report->mEventTime =(int )time(NULL);
#if 0
    externalProtocolEncode(mysbuf+1,&size);
    externalProtocolAddCrcCode(mysbuf+1,&size);
    externalProtocolAddHeader(mysbuf,&size);
#else
    externalProtocolAddCrcCode(mysbuf+1,&size);
    externalProtocolEncode(mysbuf+1,&size);
    externalProtocolAddHeader(mysbuf,&size);
#endif
    int tmp=0;
    //				prt(info,"=========report cam event======");
    //				for (tmp = 0; tmp < size; ++tmp) {
    //					prt(info,"%x",mysbuf[tmp]);
    //				}

    return SendDataByTcp(sock, (char *)mysbuf, size);
}
int ReportedRealStatic(int sock )
{
    int len=-1;
    int size, i, j, ret;
    unsigned char mysbuf[100] = { 0 };
    memset(mysbuf, 0, sizeof(mysbuf));
    FrameHeader * fhead = (FrameHeader *) mysbuf;
    size = sizeof(FrameHeader) - 1;
    fhead->mSenderLinkId = 0x00;
    fhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fhead->mProtocolVersion = 0x12;
    fhead->mSenderDeviceID.mDeviceIndex =get_dev_id();
    fhead->mSenderDeviceID.mDeviceClass = 0x1;
    fhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fhead->mSessionID = 0xFF;
    fhead->mDataType = 0x01;

    for (i = 0; i < ACTIVE_CAM_NUM; i++) {
        if(get_cam_status(i)){
            size = sizeof(FrameHeader) - 1;
            //		if (p_detect_ctx->dev_cfg.camstatus[i] == 0)
            //			continue;
            //
            //		if(loops++%10==0){
            //			if(p_detect_ctx->cam_ctx[i].thread_param.algparam.framecount==frames_count_old[i]){
            //				 ReportedCammerStatus(p_detect_ctx->sig_fd, 0xFF);
            //			}
            //			frames_count_old[i]=p_detect_ctx->cam_ctx[i].thread_param.algparam.framecount;
            //		}
            //	traffic_rst
            RealTimeTrafficData*tmpReal = (RealTimeTrafficData*)(mysbuf+sizeof(FrameHeader));
            holder[i].traffic_rst.lane_num=get_lane_num(i);
            tmpReal->mDetectChannelCount = holder[i].traffic_rst.lane_num;
            //	prt(info,"%d",	tmpReal->mDetectChannelCount);
            size+=1;
            pthread_mutex_lock(&holder[i].sig_data_lock);
            //	prt(info,"lane num %d", tmpReal->mDetectChannelCount);
            for (j = 0;j< tmpReal->mDetectChannelCount;j++) {

                struct EachChannelPack *ptrChannel = (struct EachChannelPack *) (mysbuf + sizeof(FrameHeader) + 1
                                                                                 + j * sizeof(struct EachChannelPack));
                holder[i].traffic_rst.Eachchannel[j].mWorkStatusOfDetectChannle.bDataIsValid=get_cam_running_state(i);
                memcpy((void *) ptrChannel,
                       (void *) &holder[i].traffic_rst.Eachchannel[j],sizeof(EachChannelPackm));
                size += sizeof(EachChannelPackm);
                //	prt(info,"dealing cam %d , lane %d",i,j);
                //	dump_pack(i,j,ptrChannel);
            }
            pthread_mutex_unlock(&holder[i].sig_data_lock);
            externalProtocolAddCrcCode(mysbuf + 1, &size);
            externalProtocolEncode(mysbuf + 1, &size);
            externalProtocolAddHeader(mysbuf, &size);


            len = SendDataByTcp(sock, (char *) mysbuf, size);
            if (len <= 0) {
                prt(info,"sentd tcp sig machine err");
                return len;
            } else {
            }
        }
    }
    return len;
}
void reset_sig_machine()
{

    sig_set_state(SIG_NULL);
    close_socket(&sig_fd);
    get_sig_ip(sig_ip);
    prt(info,"reset sig ip to %s",sig_ip);
    sig_port=get_sig_port();
    sig_set_state(SIG_PRE_CONNECT);
}

void *report_data_callback_fun(void *data)
{
    if (sig_state == SIG_CONNECTED) {
        if (ReportedRealStatic(sig_fd) <= 0) {
            close_socket(&sig_fd);
            sig_set_state(SIG_PRE_CONNECT);
        }
    }

}
void *check_event_callback_fun(void *data)
{
    int i;
    for (i = 0; i < ACTIVE_CAM_NUM; i++) {
        if (holder[i].traffic_rst.camera_state_change&&sig_state==SIG_CONNECTED) {
            if(ReportedCammerStatus(sig_fd, 0xFF)>0)
            {
                holder[i].traffic_rst.camera_state_change = 0;
            }else{
                prt(info,"fail to send");
            }
        }
    }
}

extern void reboot_cmd()
{
    int retlen = ReportedEvent(sig_fd, 0x1);
    if (retlen < 0) {
        close_socket(&sig_fd);
        //		return ;
    }
    retlen = ReportedEvent(sig_fd, 0x4);
    if (retlen < 0) {
        close_socket(&sig_fd);
        //		continue;
    }
    usleep(200000);
    prt(info,"rebooting ");
    system("reboot");
}
int connect_sig()
{
    int retlen;
    if (sig_fd > 0) {
        usleep(1000000);
        prt(debug_sig, "try to connect signal machine,%s",sig_ip);
        retlen =ConnectTcpClient(sig_fd,
                                 (char *) sig_ip,
                                 sig_port);
        prt(debug_sig,"ret %d",retlen);
        if (-1 == retlen) {
            close_socket(&sig_fd);
        } else {
            prt(net, "ok to connect signal machine");
        }
    }
    return retlen;
}
void *sig_service_thread(void *data)
{
    int retlen;
    while (1) {
        usleep(100000);
        switch (sig_state) {
        case SIG_PRE_CONNECT:
            prt(debug_sig,"connecting ");
            if(sig_fd<=0){
                //sig_fd=CreateTcpClientSock(SIGNALPORTL, 0);
                sig_fd=CreateTcpClientSock(0, 0);// random port
                if(sig_fd<0)
                    continue;
                if(connect_sig()<=0)
                    continue;
            }
            // retlen = ReportedEvent(sig_fd, 0x2);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send 2");
                break;
            }
            //  retlen = ReportedEvent(sig_fd, 0x3);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send 3");
                break;
            }
            //  retlen = ReportedCammerStatus(sig_fd, 0xFF);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send ff");
                break;
            }
            sig_set_state( SIG_CONNECTED);
            prt(debug_sig,"sig connected ");
            break;
        case SIG_CONNECTED:
            break;
        case SIG_NULL:
            break;
        default:
            break;
        }
    }
}
static void init_data()
{
    int i;

    sig_set_state(SIG_NULL);
    sig_fd=-1;
    get_sig_ip(sig_ip);
    sig_port=get_sig_port();
    //	pthread_mutex_init(&sig_state_lock);
    for(i=0;i<ACTIVE_CAM_NUM;i++){
        pthread_mutex_init(&holder[i].sig_data_lock,NULL);
    }
    pthread_mutex_init(&mutex_lock,NULL);
    //    pthread_mutex_init(&out_car_lock,NULL);
    //    pthread_mutex_init(&queue_lock,NULL);
    //    pthread_mutex_init(&flow_lock,NULL);
}
m_timed_func_data callback_data;
m_timed_func_data callback_data1;

void init_sig_service()
{
    init_data();
    create_detach_thread(sig_service_thread,1,NULL);
    callback_data.time=500000;
    callback_data.func=report_data_callback_fun;
    regist_timed_callback(&callback_data);

    callback_data1.time=100000;
    callback_data1.func=check_event_callback_fun;
    regist_timed_callback(&callback_data1);
    if(sig_ip[0]!='0'){
        sig_set_state(SIG_PRE_CONNECT);
    }
}


//####################nan jing #####################
#include "cam_alg.h"
data_60s_t d60[NANJING_CAM_NUM];



//---------queue start----------------



flow_info_t flow_data[NANJING_CAM_NUM];
queue_info_t queue_data[NANJING_CAM_NUM];
void get_queue_info()
{
    int i,j;
    for(i=0;i<NANJING_CAM_NUM;i++){
        for(j=0;j<NANJING_LANE_MAX;j++){
            queue_data[i].queue_len[j]=info.cams[i].lanes[j].queue_len;
            queue_data[i].crc=  get_crc((unsigned char *)&queue_data[i],sizeof(queue_info_t)-1);




            queue_data[i].detect_time[0]=get_year_tail();
            queue_data[i].detect_time[1]=get_month();
            queue_data[i].detect_time[2]=get_day();
            queue_data[i].detect_time[3]=get_hour();
            queue_data[i].detect_time[4]=get_min();
            queue_data[i].detect_time[5]=get_sec();

            queue_data[i].dir_no=get_direction(i);
            queue_data[i].lane_dir_type=0;
            queue_data[i].queue_start_pos[j]=0;
            queue_data[i].queue_veh_num[j]=info.cams[i].lanes[j].veh_no;

            for(int t=0;t<5;t++){
                queue_data[i].table_head[t]=0xfe;
            }
            queue_data[i].table_no=0x0e;
            queue_data[i].detect_status=(unsigned char)info.cams[i].lanes[j].det_status;
            queue_data[i].table_length=43;
            queue_data[i].veh_speed[j]=info.cams[i].lanes[j].speed;
        }
    }

}

int send_queue_info(int fd)
{
    printf("--->");fflush(stdout);
    int i=0;
    for(i=0;i<NANJING_CAM_NUM;i++){
        //   pthread_mutex_lock(&holder[i].sig_data_lock);
        int len = SendDataByTcp(fd, (char *) &queue_data[i], sizeof(queue_info_t));
        //  printf("---> %d",  info.cams[1].lanes[1].in_car);
        // pthread_mutex_unlock(&holder[i].sig_data_lock);
    }
    return 1;
}



m_timed_func_data camera_queue_info;// 1 s ---> focus on all camera
void *callback_camera_queue_info(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    if (sig_state == SIG_CONNECTED) {
        get_queue_info();
        if (send_queue_info(sig_fd) <= 0) {
            close_socket(&sig_fd);
            sig_set_state(SIG_PRE_CONNECT);
        }
    }
    pthread_mutex_unlock(&mutex_lock);

}
//---------queue end----------------



//----------60s flow start----------------
#include <sys/time.h>
#include <time.h>
int send_flow_info(int fd)
{
    int i=0;
    int j=0;
    for(i=0;i<NANJING_CAM_NUM;i++){
        int len = SendDataByTcp(fd, (char *) &flow_data[i], sizeof(flow_info_t));
    }
    prt(info,"sent %x\n",flow_data[0].table_length);
    prt(info,"sent1 %x\n",flow_data[1].table_length);
    return 1;
}

void calculate_60s()
{
    //cal
    //lock
    int i=0;
    int j=0;
    for( i=0;i<NANJING_CAM_NUM;i++){
        get_time_string((char *)flow_data[i].detect_time,sizeof(flow_data[i].detect_time));
        flow_data[i].crc=  get_crc((unsigned char *)&flow_data[i],sizeof(flow_info_t)-1);
        flow_data[i].dir_no=get_direction(i);
        flow_data[i].section_no=1;
        flow_data[i].table_length=48;
        flow_data[i].table_no=0x0c;
        for(int t=0;t<5;t++){
            flow_data[i].table_head[t]=0xfe;
        }

     //   prt(info,"%x\n",flow_data[i].table_length);
        //    if(d60[i].data_valid){//time up, stop acu,start cal and send
        for( j=0;j<NANJING_LANE_MAX;j++){
            flow_data[i].ocuppy_percent[j]=(unsigned char)d60[i].lane_data[j].exist_duration*100/60/1000;
            d60[i].lane_data[j].exist_duration=0;
            if(flow_data[i].flow[j])
                flow_data[i].average_speed[j]=(unsigned char)d60[i].lane_data[j].speed_sum/flow_data[i].flow[j];
            d60[i].lane_data[j].speed_sum=0;
            flow_data[i].flow[j]=(unsigned char)d60[i].lane_data[j].pass_number;
            d60[i].lane_data[j].pass_number=0;


            //            flow_data[i].detect_time[0]=get_year();
            //            flow_data[i].detect_time[1]=get_month();
            //            flow_data[i].detect_time[2]=get_day();
            //            flow_data[i].detect_time[3]=get_hour();
            //            flow_data[i].detect_time[4]=get_min();
            //            flow_data[i].detect_time[5]=get_sec();
      //  prt(info,"(%d %d )-> %x\n\n",i,j,flow_data[0].table_length);
        }
       // prt(info,"%x\n\n",flow_data[i].table_length);
  //  prt(info,"(%d %d )--> %x\n\n",i,j,flow_data[0].table_length);
    }
   // prt(info,"---> %x\n\n",flow_data[0].table_length);

  //  prt(info,"done\n");

}
m_timed_func_data camera_flow_info;// 60 s---> focus on all camera
void *callback_camera_flow_info(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    time_t tm;
    tm=time(NULL);
    if(tm%60==0){
        calculate_60s();
        send_flow_info(sig_fd);
    }
    pthread_mutex_unlock(&mutex_lock);
}
//----------60s flow end------------------










//--------- outcar start------------------
ourcar_info_t ourcar;
int send_outcar_info(int fd,int cam_index, int lane_index)
{
    ourcar.veh_speed=info.cams[cam_index].lanes[lane_index].speed;
    ourcar.lane_number=get_lane_index(cam_index,lane_index);


    int i=0;
    int j=0;
    for(i=0;i<5;i++){
        ourcar.table_head[i]=0xfe;
    }
    ourcar.table_no=0x0f;
    ourcar.table_length=18;
    ourcar.pass_time[0]=get_year_tail();
    ourcar.pass_time[1]=get_month();
    ourcar.pass_time[2]=get_day();
    ourcar.pass_time[3]=get_hour();
    ourcar.pass_time[4]=get_min();
    ourcar.pass_time[5]=get_sec();
    ourcar.dir_no=get_direction(cam_index);
    ourcar.lane_dir_type=0;
    ourcar.section_number=1;
    ourcar.veh_type=1;
    ourcar.occupy_time=info.cams[cam_index].lanes[lane_index].out_car_time-info.cams[cam_index].lanes[lane_index].in_car_time;
    ourcar.crc=get_crc((unsigned char *)&ourcar,sizeof(ourcar_info_t)-1);
    // for(i=0;i<NANJING_CAM_NUM;i++){
    //   int len =
    SendDataByTcp(fd, (char *) &ourcar, sizeof(ourcar_info_t));

    //    }
    return 1;
}

m_timed_func_data outcar_check_info;// 1ms   --> focus on 1 lane
void *callback_outcar_check_info(void *data)
{

    int i=0;
    int j=0;
    pthread_mutex_lock(&mutex_lock);
    for(i=0;i<NANJING_CAM_NUM;i++){
        for(j=0;j<NANJING_LANE_MAX;j++){
            if(info.cams[i].lanes[j].out_car){
                if (sig_state == SIG_CONNECTED) {
                    if (send_outcar_info(sig_fd,i,j) <= 0) {
                        close_socket(&sig_fd);
                        sig_set_state(SIG_PRE_CONNECT);
                    }
                }
                info.cams[i].lanes[j].out_car=0;
            }

        }
    }
    pthread_mutex_unlock(&mutex_lock);
}
//--------- outcar end------------------





typedef void *(func)(void*);
void regist_callback(m_timed_func_data *data,func fc,int ms)
{
    data->func=fc;
    data->time=ms*1000;
    regist_timed_callback(data);
}

void init_sig_service1()
{
    init_data();
    create_detach_thread(sig_service_thread,1,NULL);

    regist_callback(&camera_queue_info,callback_camera_queue_info,1000);
    regist_callback(&camera_flow_info,callback_camera_flow_info,1000);
    regist_callback(&outcar_check_info,callback_outcar_check_info,10);

    if(sig_ip[0]!='0'){
        sig_set_state(SIG_PRE_CONNECT);
    }



}
