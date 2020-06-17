#include <opencv2/core/core.hpp>
#include <System.h>

int main (){

    /*cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
*/ /*cv::VideoCapture cap(1);
    NR_SLAM NR_slam(cap);
*/ /*/ string arg = "/home/lamarca/ORB-SDTAM/calibration_files/logitechc922.yaml" ;
    string arg2= "/home/lamarca/ORB-SDTAM/Vocabulary/ORBvoc.txt";*/
 //   string arg = "/home/jose/ORB-SDTAM/calibration_files/logitechc922.yaml" ;
    string arg = "../../calibration_files/stereo.yaml" ;

    string arg2= "../../Vocabulary/ORBvoc.txt";
    ORB_SLAM2::System SLAM(arg2,arg,ORB_SLAM2::System::MONOCULAR,true);
   //cv::VideoCapture cap(0);
    cv::VideoCapture cap("../../videos/DUOCapture1.avi" );

    // cv::VideoCapture cap("/home/jose/ORB-SDTAM/videos/blabla.webm" );
  //     cv::VideoCapture cap("/home/jose/ORB-SDTAM/videos/keyframeschanges.webm" );
    // cv::VideoCapture cap("/home/jose/ORB_SLAM2/videomanta.webm" );
///     cv::VideoCapture cap("/home/jose/ORB_SLAM2/video3.webm" );

   // cv::VideoCapture cap("/home/jose/ORB_SLAM2/2017-10-04-111941.webm" );
   // cv::VideoCapture cap("/home/jose/ORB_SLAM2/video2.webm" );

    using namespace cv;
    int ndisparities = 16*5;   /**< Range of disparity */
    int SADWindowSize = 21; /**< Size of the block window. Must be odd */
    uint i(0);

    Ptr<StereoSGBM> sgbm = new StereoSGBM(0,16,5);
       int blockSize = 15;
       sgbm->SADWindowSize =blockSize;
       sgbm->numberOfDisparities = 160;
       sgbm->minDisparity = 0;
    while (cap.isOpened()){
        double u(0.0);
        cv::Mat a;
        cap >> a;

        int height = a.rows;
        int width = a.cols;
        cv :: Rect rect1 =   cv :: Rect ( 0 , 0 ,  width/2 , height);
        cv :: Rect rect2 =   cv :: Rect ( width/2-1 , 0 ,  width/2 , height );
        cv::Mat iml = cv :: Mat ( a , rect1 );
        cv::Mat imr = cv :: Mat ( a, rect2 );

        if( iml.empty() || imr.empty() )
        { std::cout<< " --(!) Error reading images " << std::endl; return -1; }
        Mat imgDisparity16S = Mat( iml.rows, iml.cols, CV_16S );
        Mat imgDisparity8U = Mat( iml.rows, iml.cols, CV_8UC1 );
        SLAM.TrackMonocular(iml,u);
        u=u+0.01;
        cv::waitKey(30);
        /*if (!cap.isOpened()){
            cap.release();
            cv::VideoCapture cap("/home/jose/ORB_SLAM2/blabla.webm" );
            i=-1;
        }*/
        cv::Mat imlnew;
        cv::cvtColor(iml,imlnew, COLOR_BGR2GRAY);
        cv::Mat imrnew;
        cv::cvtColor(imr,imrnew, COLOR_BGR2GRAY);
        sgbm->operator ()(imlnew,imrnew,imgDisparity16S);//disparity private?

        double minVal; double maxVal;

        minMaxLoc( imgDisparity16S, &minVal, &maxVal );

        printf("Min disp: %f Max value: %f \n", minVal, maxVal);

        //-- 4. Display it as a CV_8UC1 image
        normalize(imgDisparity16S, imgDisparity8U, 0, 255, CV_MINMAX, CV_8U);


        namedWindow( "Disparity", WINDOW_NORMAL );
        imshow( "Disparity", imgDisparity8U );
        waitKey(10);

        namedWindow( "rightimage", WINDOW_NORMAL );
        imshow( "rightimage", imrnew );
        waitKey(10);

         std::vector<std::pair<std::pair<uint,uint>,double>> achis = SLAM.getKeypointsandDistance();
         std::cout << achis.size() << std::endl;
         for (std::vector<std::pair<std::pair<uint,uint>,double>>::iterator it = achis.begin(); it != achis.end();it++){
                double scale = it->second / (imgDisparity16S.at<short>(it->first.first,it->first.second))*1000;
                std::cout << scale << std::endl;
         }
        i++;
    }

    SLAM.Shutdown();

    return 0;
}
