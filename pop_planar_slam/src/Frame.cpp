# include "Frame.h"
using namespace std;


void tracking_frame::compute_frame_BoW(const ORB_SLAM::ORBVocabulary* mpORBvocabulary, ORB_SLAM::ORBextractor* mpORBextractor)
{
    // ORB descriptor, each row associated to a keypoint
    cv::Mat mDescriptors;
    cv::Mat gray_img;
    if ( frame_img.channels()==3 )
	cv::cvtColor(frame_img, gray_img, CV_RGB2GRAY);
    else
	gray_img=frame_img;
    
    (*mpORBextractor)(gray_img,cv::Mat(),mvKeys,mDescriptors);
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = ORB_SLAM::Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}