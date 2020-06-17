#include <opencv2/core/core.hpp>
#include <System.h>
#include <math.h>       /* log */
#include <Timer.h>

enum stage{
    init=0,
    experiment=1,
    end=2
} e;

int main (){
    string arg = "../../calibration_files/asus.yaml" ;
    string arg2= "../../Vocabulary/ORBvoc.txt";

    ORB_SLAM2::System SLAM(arg2,arg,ORB_SLAM2::System::MONOCULAR,true);
    ofstream times;

    ofstream myfile2Doport;
    ofstream myfile2DoportNode;

    ofstream myfile2Dreal;
    ofstream myfile2DrealNode;

    times.open("time.txt");

    myfile2Doport.open("oport.txt");
    myfile2Dreal.open("oportreal.txt");
    myfile2DoportNode.open("oportnode.txt");
    myfile2DrealNode.open("oportrealnode.txt");

    uint i(57);
    e=stage::init;
    uint countas(0);
    while (true){
        std::cout << i << std::endl;
        if ((i==1197)and(e==stage::init))
            SLAM.ActivateLocalizationMode();
        double u(0.0);

     //   std::cout <<"/home/jose/ORB-SDTAM/Examples/Monocular/Color"+to_string(i)+".jpeg" << std::endl;
         cv::Mat a,a2;
         cv::Mat adepth;

        if (e==stage::init){
            a = cv::imread("../../videos/initializationsheet3/Color"+to_string(i)+".jpeg") ;
            adepth = cv::imread("../../videos/initializationsheet3/Depth"+to_string(i)+".png",cv::IMREAD_ANYDEPTH) ;
        }
        else if (e==stage::experiment){
            if (i%10==1){
            a2 = cv::imread("../../videos/dataset/Color"+to_string(i)+".jpeg") ;
            if (!a2.empty()){
                a=a2.clone();
            }
            adepth = cv::imread("../../videos/dataset/Depth"+to_string(i)+".png",cv::IMREAD_ANYDEPTH) ;
            }else{
                cv::Mat BlackImage(a.cols, a.rows, CV_8UC3, cv::Scalar(0,0,0));
                a = BlackImage;
            }
        }else{
            break;
        }
       // cap >> a;
        Timer timer;
        if (((i<1200)and(e==stage::init))or((e==stage::experiment)and(i<10200))){
        if (!a.empty()){
            if (e==stage::init){
               timer.start();
               SLAM.TrackMonocular(a,i);
               timer.stop();
               std::cout<< "Runtime frame: " << timer.getElapsedTimeInMilliSec() << " ms" <<  std::endl;
               times << i << " " << timer.getElapsedTimeInMilliSec() << std::endl;
            }
            if (e==stage::experiment)
            {
               timer.start();
               SLAM.RelocateMonocular(a,i+1200);
               timer.stop();
               times << i+1200 << " " << timer.getElapsedTimeInMilliSec() << std::endl;

               std::cout<< "Runtime frame: " << timer.getElapsedTimeInMilliSec() << " ms" <<  std::endl;
            }
        }
            u=u+0.03;
        }
        else
        {
            if (e==stage::init){
               e = stage::experiment;
               i=147;
            }
            else if (e==stage::experiment){
                e = stage::end;
                i=0;
            }
        }


        if (true)
        if ((!adepth.empty())){
        std::vector<std::pair<std::pair<uint,uint>,double>> KPdistance = SLAM.getKeypointsandDistance();

        std::vector<std::pair<double,double>> dk;

        for (std::vector<std::pair<std::pair<uint,uint>,double>>::iterator itb = KPdistance.begin();itb!=KPdistance.end();itb++){
              short rdmm = adepth.at<short>(itb->first.second,itb->first.first);
              double  rd =double(rdmm)/1000;
              double  cd = itb->second;
              std::pair<double,double> d(rd,cd);
              dk.push_back(d);
        }

        std::vector<std::pair<std::pair<uint,uint>,double>> Nodedistance = SLAM.getNodesandDistance();

        std::vector<std::pair<double,double>> dn;

     /*   for (std::vector<std::pair<std::pair<uint,uint>,double>>::iterator itb = Nodedistance.begin();itb!=Nodedistance.end();itb++){
            if ((itb->first.second<adepth.cols)and(itb->first.first<adepth.rows)){
                double  rd(0.0);
                try {
                    unsigned short rdmm = 0;
                    rdmm = (unsigned short)(adepth.at<unsigned short>(itb->first.second,itb->first.first));
                   /* std::cout << itb->first.second << " " << itb->first.first << std::endl;

                    std::cout << rdmm << std::endl;
                    rd = double(rdmm);
                } catch (...){
                    std::cout << "Fail" << std::endl;
                }
                rd = rd*0.001;
                double  cd = itb->second;
                if ((rd>0)&&(cd>0)){
                std::pair<double,double> d(rd,cd);
                dn.push_back(d);}
              }
        }
*/
        if(KPdistance.size()!=0){
            myfile2Doport << i << " ";
            myfile2Dreal  << i << " ";

            myfile2DoportNode<< i <<" ";
            myfile2DrealNode << i << " ";
            for (std::vector<std::pair<double,double>>::iterator it = dk.begin();it!=dk.end();it++){
                double a = (it->first-it->second);
                if ((!std::isnan(a))and(!std::isinf(a))and(abs(it->first<2))and((it->first)>0.5)){
                    myfile2Doport << it->second << " ";
                    myfile2Dreal << it->first <<" ";
                }
            }
            myfile2Doport << std::endl;
            myfile2Dreal << std::endl;

            for (std::vector<std::pair<double,double>>::iterator it = dn.begin();it!=dn.end();it++){
                double a = (it->first-it->second);
                if ((!std::isnan(a))and(!std::isinf(a))and(abs(it->first<2))and((it->first)>0.5)){
                    myfile2DoportNode << it->second << " ";
                    myfile2DrealNode << it->first <<" ";
                }
            }
            myfile2DoportNode << std::endl;
            myfile2DrealNode << std::endl;
        }
        }else{

        }
        usleep(10000);
       // std::cout << "countas " << countas << std::endl;

        i++;
    }
    SLAM.Shutdown();
    myfile2Doport.close();
    return 0;
}
