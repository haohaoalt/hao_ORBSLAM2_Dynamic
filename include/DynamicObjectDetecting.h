/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-09-08 12:12:55
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-11-09 10:07:07
 * @FilePath: /hao_ORBSLAM2_Dynamic/include/DynamicObjectDetecting.h
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */

#ifndef DYNAMIC_FOBJECT_DETECTING_H
#define DYNAMIC_FOBJECT_DETECTING_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>

#include "yolo_v2_class.hpp" 


namespace ORB_SLAM2
{


class DynamicObjectDetecting
{
public:
    DynamicObjectDetecting();
    void init(std::vector<std::string> specifiedThings, std::string cfgFile, std::string weightFile, std::string labelPath, double DetectTh);
    void run();

    void insertImage(const cv::Mat& im);
    std::vector<bbox_t> getDetectResult();
    
    void setFinish();
    bool isFinished();
    void requestFinish();

    // hayden: getDynamicObjectBbox实现
    std::vector<bbox_t> getDynamicObjectBbox(const cv::Mat& mImGray);
    // hayden: getSpecifiedObjectBbox实现
    std::vector<bbox_t> getSpecifiedObjectBbox(const cv::Mat& mImGray);
    void saveDistance();
    void setImGrayPre(const cv::Mat &imGrayPre);

    std::vector<std::string> id2name;
    std::unique_ptr<Detector> detector;

private:
    void loadLabel();
    void calcDynamicPoints(const cv::Mat &imGray);
    const std::vector<cv::Point2f>& getDynamicPoints();
    const std::vector<cv::Point2f>& getTrackedPoints();

    // config parameters
    std::string mCfgFile, mWeightFile;
    std::string mLabelPath;
    double mDetectTh;
    std::vector<std::string> mSpecifiedThings;

    // obtain the dynamic points
    cv::Mat mImGrayPre;
    std::vector<cv::Point2f> mDynamicPoints;
    std::vector<cv::Point2f> mTrackedPoints;
    const double limit_dis_epi = 1; 
    const double limit_of_check = 2120;
    const int limit_edge_corner = 5; 

};


}  // namespace ORB_SLAM2


#endif


