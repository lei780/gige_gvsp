
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>

#include <pthread.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <queue>
#include <atomic>
#include <string>
#include <functional>
#include <mutex>
#include <list>
#include <queue>
#include <condition_variable>
#include <cstring>
#include <cstdint>
#include <cstdint>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/objdetect.hpp>
//#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "yuv_output.h"

using namespace std;
using namespace cv;

/** Global variables */
uint32_t g_image_width = 1920;
uint32_t g_image_height = 1080;

uint32_t g_image_size;
uint32_t g_image_count;

uint32_t g_yuv_loop = 0;

std::mutex recv_lock;
std::queue<char *> recv_queue;
std::condition_variable recv_queue_cond;

void print_help() 
{
  printf("Usage: simple_cam <width> <height> <device>\n");
  printf("Example: simple_cam 640 480 /dev/video0\n");
}

void set_resolution(uint32_t width, uint32_t height)
{
    g_image_width = width;
    g_image_height = height;

    g_image_size = (g_image_width*g_image_height) + (g_image_width*g_image_height/2); 
    //g_image_count = f_info.st_size / g_image_size;
}

void feeding_data(char *pyuv, uint32_t length)
{
    std::cout << __FUNCTION__ << ": Pushed yuv data, length= " << length << std::endl;

    std::lock_guard<std::mutex> lk(recv_lock);
    /* packets are inserted to list's queue */
    recv_queue.push(pyuv);
    recv_queue_cond.notify_one();

}

void output_main_stop() 
{
    g_yuv_loop = 0; 
}

int output_main() 
{
    bool bret; 
    char *pbuf;
    cv::Mat picNV12;
    cv::Mat picBGR;
    cv::Mat picGRAY;

    //int g_count = 0; 
    //char fname[32];
    //unsigned int fsize = 1920*1080+(1920*1080/2); 

    g_yuv_loop = 1; 
    while(g_yuv_loop == 1) 
    {
        std::unique_lock<std::mutex> lk(recv_lock);
        bret = recv_queue_cond.wait_for( lk, std::chrono::milliseconds(100), []{ return !recv_queue.empty(); } );
        if(bret) {
            std::cout << __FUNCTION__ << ": Received yuv data" << std::endl;
            std::cout << __FUNCTION__ << ": width : " << g_image_width << std::endl;
            std::cout << __FUNCTION__ << ": height : " << g_image_height << std::endl;
            pbuf = recv_queue.front();
            recv_queue.pop();
            lk.unlock();
#if 0
            sprintf(fname, "yuv_sample_%d.yuv", g_count++); 
            int fd = open(fname, O_RDWR|O_CREAT, 0644);

            std::cout << "fsize: " << fsize << std::endl;
            write(fd, pbuf, fsize);
            close(fd); 
#endif
            picNV12 = cv::Mat(g_image_height*3/2, g_image_width, CV_8UC1, pbuf);
            cv::cvtColor(picNV12, picBGR, cv::COLOR_YUV2RGB_NV12);
            cv::cvtColor(picNV12, picGRAY, cv::COLOR_YUV2GRAY_NV12);

 
            std::string greyArrWindow = "Grey Array Image";
            //cv::namedWindow(greyArrWindow, cv::WINDOW_AUTOSIZE);
            cv::namedWindow(greyArrWindow, cv::WINDOW_NORMAL);
            cv::resizeWindow(greyArrWindow, 1280, 720);
            //cv::namedWindow(greyArrWindow);
 
            //cv::imshow(greyArrWindow, picBGR);
            cv::imshow(greyArrWindow, picGRAY);
            delete[] pbuf;
        }
        cv::waitKey(1);
    }

    std::cout << "Window Close.. " << std::endl;
    cv::destroyAllWindows();
    return 0;
}


