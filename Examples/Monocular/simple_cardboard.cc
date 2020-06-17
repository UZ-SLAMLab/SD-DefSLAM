#include <opencv2/core.hpp>
#include <System.h>

int main (){

    /*cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
*/ /*cv::VideoCapture cap(1);
    NR_SLAM NR_slam(cap);
*/ /*/ string arg = "/home/lamarca/ORB-SDTAM/calibration_files/logitechc922.yaml" ;
    string arg2= "/home/lamarca/ORB-SDTAM/Vocabulary/ORBvoc.txt";*/
    string arg = "../../calibration_files/logitechc922.yaml" ;
    string arg2= "../../Vocabulary/ORBvoc.txt";
    ORB_SLAM2::System SLAM(arg2,arg,ORB_SLAM2::System::MONOCULAR,true);

    uint i(15);

    while (true){

        double u(0.0);

        //std::cout <<"/home/jose/ORB-SDTAM/Examples/Monocular/Color"+to_string(i)+".jpeg" << std::endl;
        cv::Mat a  = cv::imread("../../videos/cardboard/seq/Movie"+to_string(i)+".png") ;
        SLAM.TrackMonocular(a,u);
        u=u+0.01;
        usleep(50000);

        cv::waitKey(10);
        i++;
    }

    SLAM.Shutdown();

    return 0;
}
