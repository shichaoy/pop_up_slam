/*
 * line_detection interface
 * Copyright Shichao Yang,2016, Carnegie Mellon University
 * Email: shichaoy@andrew.cmu.edu
 *
 */

#include <line_lbd/line_descriptor.hpp>

// #include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <ctime>
#include <line_lbd/line_lbd_allclass.h>

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

static const char* keys =
{ "{@image_path | | Image path }" };

static void help()
{
  cout << "\nThis example shows the functionalities of lines extraction " << "furnished by BinaryDescriptor class\n"
       << "Please, run this sample using a command in the form\n" << "./example_line_descriptor_lines_extraction <path_to_input_image>" << endl;
}


int main( int argc, char** argv )
{
      /* get parameters from comand line */
      if(argc<2){
	help();
	return -1;
      }
      
      std::string image_path(argv[1]);
      
      /* load image */
      cv::Mat imageMat = imread( image_path, 1 );
      if( imageMat.data == NULL )
      {
	std::cout << "Error, image could not be loaded. Please, check its path \n"<<image_path << std::endl;
	return -1;
      }
            
      /* create a random binary mask */
      cv::Mat mask = Mat::ones( imageMat.size(), CV_8UC1 );

      BinaryDescriptor::Params line_params;
      line_params.numOfOctave_ = 1;
      line_params.Octave_ratio = 2.0;  
      
      std::clock_t start = std::clock();
      
      // using my line detector class, could select LSD or edline.
      cv::Mat output4 = imageMat.clone();
      line_params.numOfOctave_ = 1;
      line_params.Octave_ratio = 2.0;
      line_lbd_detect* line_lbd_ptr = new line_lbd_detect(line_params.numOfOctave_,line_params.Octave_ratio); 
      line_lbd_ptr->use_LSD=false;
      line_lbd_ptr->line_length_thres=10;
      std::vector<std::vector< KeyLine>> keylines_out;
      std::vector<cv::Mat> line_descrips;
      line_lbd_ptr->detect_descrip_lines_octaves(imageMat,keylines_out,line_descrips);
//       std::cout<<"my keylines_out size "<<keylines_out[0].size()<<std::endl;
    //   std::cout<<"line_descrips "<<line_descrips[0]<<std::endl;
      
      
      /* draw lines extracted from octave 0 */
      if( output4.channels() == 1 )
	cvtColor( output4, output4, COLOR_GRAY2BGR );  
      drawKeylines(imageMat, keylines_out[0], output4, cv::Scalar( 0, 0, 255 ),2); // B G R paper cv::Scalar( 0, 150, 0 ) 2   Scalar::all( -1 ),3
      imshow( "My Line detector", output4 );
      waitKey();
      char charbuf[500];
      snprintf(charbuf,500,"/home/yamahas/ysc_space/mavscout/src/line_lbd/data/paper/frame_edge_%05d.png",10); //TODO change file directory.
      cv::imwrite(charbuf,output4);  
      
    //   , cv::Scalar( 120, 0, 0 ),2
  
}
