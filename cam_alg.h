/*
 * cam_alg.h
 *
 *  Created on: 2016年6月30日
 *      Author: root
 */

#ifndef ALG_H_
#define ALG_H_

#include "m_arith.h"
//#include "param.h"
//#include "protocol.h"
#include "client_obj.h"
enum{
    TIME_SECTION_NULL,
    DUSK=1,
    NIGHT,
    MORNING,
    DAYTIME
};
enum{
    ALG_NULL,
    ALG_DAYTIME=1,
    ALG_NIGHT,
};
typedef struct AlgParam{
    m_args alg_arg;
    LANEINISTRUCT LaneIn;
    short algNum;
    int alg_index;   //默认情况是是1   1/0
    int time_section;//1-4   1=>huang hun 2=>wan shang  3=>ling cheng   4=> bai tian  在开始程序的时候，以及每隔10s获取一次，传递给算法。，
    int framecount;    //帧数计数器
    SPEEDCFGSEG    pDetectCfgSeg;
    CFGINFOHEADER  pCfgHeader;
    RESULTMSG outbuf;
    int tick;
}mAlgParam;

typedef struct alg_context{
}m_alg_context;
static int alloc_alg(mAlgParam *algparam,mCamDetectParam *p_camdetectparam,mCamParam *p_cammer,int index);
void release_alg(int index);
int run_alg(int index,unsigned char *y,unsigned char *u,unsigned char *v);
int reset_alg(int index);
int open_alg(int index);
void extern init_alg(int index);



#define NANJING_CAM_NUM 4
#define NANJING_LANE_MAX 4
typedef struct lane_rst_info{
    int no;// che dao bian hao
    int ms;// jian ce shi jian(ms)
    int queue_len;// pai dui chang du
    int start_pos;// pai dui kai shi wei zhi
    int veh_no;// che liang zong shu
    int speed;// shang yi liang che de che su
    int veh_type;//che liang lei xing
    int in_car;//dang qian jin che biao zhi
    int in_car_time;//dang qian jin che biao zhi
    int out_car;// dang qian chu che biao zhi
    int out_car_time;// dang qian chu che biao zhi
    int exist_flag;
    int last_exist_flag;
    int det_status;

}lane_rst_info_t;
typedef struct cam_rst_info{
    lane_rst_info_t lanes[NANJING_LANE_MAX];
}cam_rst_info_t;
typedef struct frame_info{
    cam_rst_info_t cams[NANJING_CAM_NUM];
}frame_info;
typedef struct l_d{
    int exist_duration;
    int pass_number;
    int speed_sum;
}l_d_t;
typedef struct data_60s{
    int data_valid;//when valid is false, accumulate start;when time is up , calculate and turn vaild true , and stop accumulate

    l_d_t lane_data[4];


}data_60s_t;
extern frame_info info;
extern data_60s_t d60[NANJING_CAM_NUM];

#endif /* INCLUDE_CAM_ALG_H_ */
