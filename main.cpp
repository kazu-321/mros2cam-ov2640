// include here
#include "ThisThread.h"
#include "logging.h"
#include "mbed.h"
#include "mros2.h"
#include "mros2-platform.h"
#include "sensor_msgs/msg/image.hpp"
#include <cstdint>
#include <cstdio>
#include <math.h>
#include <vector>
#include "arducam.h"
// using and define here
using namespace std;

ArduCAM cam(PC_12,PC_11,PC_10,PD_2,PB_9,PB_8);          // カメラ
void cam_fn();

mros2::Publisher pub_cam;


// プログラムhere
int main(){
    MROS2_INFO("start");
    if (mros2_platform::network_connect()) {            // connect to the network
        MROS2_ERROR("ネットつながってないにょ...");
        return -1;
    } else {
        MROS2_INFO("ネットつながた！---");
    }


    MROS2_INFO("カメラ起動");
    cam.InitCAM();
    MROS2_INFO("カメラﾖｼ！");

    MROS2_INFO("mini robot起動します");

    mros2::init(0, NULL);       // 初期化
    MROS2_DEBUG("mROS 2初期化完了");

    mros2::Node node = mros2::Node::create_node("mros2_node");  // node生成
    mros2::Publisher pub_cam = node.create_publisher<sensor_msgs::msg::Image>("image",10);

    osDelay(100);
    MROS2_INFO("送受信準備完了！");
    cam_fn();
    mros2::spin();
    return 0;
}

void cam_fn(){
    ThisThread::sleep_for(2s);
    while(true){
        cam.write_reg(0x00,0x56);
        if(cam.read_reg(0x00)==0x56){
            printf("ok!!\n");
            break;
        }else{
            printf("spi error...\n");
        }
        ThisThread::sleep_for(100ms);
    }
    while(true){
        cam.flush_fifo();
        cam.clear_fifo_flag();
        cam.start_capture();
        while(!cam.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
        uint32_t imageLength = cam.read_fifo_length();
        printf("data %d\n",imageLength);
        cam.CS_LOW();
        cam.set_fifo_burst();
        auto message=sensor_msgs::msg::Image();
        message.frame_id="camera_flame";
        message.nanosec=0;
        message.sec=time(NULL);
        message.height=240;
        message.width=320;
        message.encoding="rgb";
        message.is_bigendian=0;
        message.step=320*3;
        message.data.resize(imageLength);
        // vector<uint8_t> imageData(imageLength);
        for(int i=0;i<imageLength;i++){
            uint8_t byte=cam.read_fifo();
            // printf("%d,",byte);
            // imageData[i]=byte;
            message.data[i]=byte;
        }

        cam.CS_HIGH();
        cam.clear_fifo_flag();
        // message.data=imageData;
        pub_cam.publish(message);
    }
}