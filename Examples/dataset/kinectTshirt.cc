#include <opencv2/core/core.hpp>
#include <Examples/dataset/SystemTest.h>

class GrabberKinect
{
public:
    GrabberKinect(const string & ,const string & ,uint ,const string & vertex,const string & index,const string & ,const string & );

    cv::Mat GetRGB(uint i);
    uint size();
    ORB_SLAM2::System* pSys;
    std::vector<std::vector<double>> & get_vertices();
    std::vector<std::vector<int> > &get_indexes();
    std::vector<std::vector<double>> &get_matches(uint i);
    std::vector<std::vector<double>> &get_matchesGT(uint i);

    std::vector<std::vector<std::vector<double>>> matches;
    std::vector<std::vector<std::vector<double>>> matchesGT;

private:
    cv::Mat Colorimages[1000];
    std::vector<std::vector<double>> vertices;
    std::vector<std::vector<int>> indexes;
    cv::Mat dephtimages[18000];
    double u;

    uint size_of_dataset;
    std::mutex mMutex;
};

GrabberKinect::GrabberKinect(const string & place, const string & format, uint size_data_set, const string & vertex, const string & index, const string & SIFT2D, const string & SIFT3D)
    : size_of_dataset(size_of_dataset)
{
    //////////
    /// Charge images
    uint i(0);
    while (i<size_data_set){
      //  std::cout << i  <<" %" <<std::endl;
        /// Read image
    if (i<9){
       // std::cout << place+to_string(0)+to_string(0)+to_string(i+1)+format << std::endl;
       Colorimages[i] = cv::imread(place+to_string(0)+to_string(0)+to_string(i+1)+format);
    }else if (i<99){
     //  std::cout << place+to_string(0)+to_string(i+1)+format << std::endl;
       Colorimages[i] = cv::imread(place+to_string(0)+to_string(i+1)+format);
    }else if (i<999){
       //std::cout << place+to_string(i+1)+format << std::endl;
       Colorimages[i] = cv::imread(place+to_string(i+1)+format);
    }
      i++;
    }

 i = 2;
    while (i<size_data_set){
    //    std::cout << i  <<" %" <<std::endl;
        /// Read image
//std::cout << SIFT2D+to_string(i)+".txt" << std::endl;
    /// Read matches
     ifstream mySIFT2DFile;
     mySIFT2DFile.open(SIFT2D+to_string(i)+".txt");
     std::string output;
     std::vector<std::vector<double>> matchesimagei;
     if (mySIFT2DFile.is_open()) {
     while (!mySIFT2DFile.eof()) {
        getline(mySIFT2DFile,output);
        stringstream ss;
        ss << output;
        uint j(0);
        string temp;
        double found;
        std::vector<double> matchi;
        while (!ss.eof()) {
                /* extracting word by word from stream */
                ss >> temp;
                (stringstream(temp) >> found);
                matchi.push_back(found);
        }
        //std::cout<< matchi[0] << " " << matchi[1] << " " <<matchi[2] << std::endl;
        matchesimagei.push_back(matchi);
     }
    }
     matchesimagei.pop_back();

     mySIFT2DFile.close();
     matches.push_back(matchesimagei);


     /// Read GroundTruth matches
      ifstream mySIFTGTFile;
      std::vector<std::vector<double>> matchesGTi;
      mySIFTGTFile.open(SIFT3D+to_string(i)+".txt");
      if (mySIFTGTFile.is_open()) {
      while (!mySIFTGTFile.eof()) {
         getline(mySIFTGTFile,output);
         stringstream ss;
         ss << output;
         uint j(0);
         string temp;
         double found;
         std::vector<double> GT;
         while (!ss.eof()) {
                 /* extracting word by word from stream */
                 ss >> temp;
                 (stringstream(temp) >> found);
                 GT.push_back(found);
               //  Index.push_back(found);
                 j++;
         }
         //std::cout << GT[0] << " "<< GT[1] << " "<< GT[2] << " " << std::endl;
         matchesGTi.push_back(GT);
      }
     }
      matchesGTi.pop_back();
      matchesGT.push_back(matchesGTi);
      mySIFTGTFile.close();
      i++;
    }
    std::cout << matches[0].size() << " "
                                   << matchesGT[0].size()<<std::endl;
    //////////
    /// Read vertex of the mesh
    ifstream myVertexFile;
    myVertexFile.open(vertex);
    std::string output;
    if (myVertexFile.is_open()) {
    while (!myVertexFile.eof()) {
       getline(myVertexFile,output);
       stringstream ss;
       ss << output;
       uint i(0);
       string temp;
       double found;
       std::vector<double> Vertex;
       while (!ss.eof()) {
               /* extracting word by word from stream */
               ss >> temp;
               (stringstream(temp) >> found);
               Vertex.push_back(found);
               i++;
           }
     //  std::cout<< Vertex[0] << " " << Vertex[1] << " " <<Vertex[2] << std::endl;
        vertices.push_back(Vertex);
    }
   }
   myVertexFile.close();
   //////////
   /// Read index of the mesh

   ifstream myIndexFile;
   myIndexFile.open(index);
   if (myIndexFile.is_open()) {
   while (!myIndexFile.eof()) {
      getline(myIndexFile,output);
      stringstream ss;
      ss << output;
      uint j(0);
      string temp;
      int found;
      std::vector<int> Index;
      while (!ss.eof()) {
              /* extracting word by word from stream */
              ss >> temp;
              (stringstream(temp) >> found);
              Index.push_back(found);
            //  Index.push_back(found);
              j++;
      }
      indexes.push_back(Index);
   }
  }
   myIndexFile.close();
   indexes.pop_back();

}


cv::Mat GrabberKinect::GetRGB(uint i){
    return (Colorimages[i]);
}



std::vector<std::vector<int>> & GrabberKinect::get_indexes(){

    //
    return indexes;
}


std::vector<std::vector<double>> & GrabberKinect::get_vertices(){
    return vertices;
}

std::vector<std::vector<double>> & GrabberKinect::get_matches(uint i){
   return matches[i];
}

std::vector<std::vector<double>> & GrabberKinect::get_matchesGT(uint i){
   return matchesGT[i];
}

uint GrabberKinect::size(){
    return (size_of_dataset);
}

int main (){

    string arg = "../../calibration_files/asusKinectPaper.yaml" ;
    string arg2= "../../Vocabulary/ORBvoc.txt";

    GrabberKinect grabberKinect("../../videos/kinect_tshirt/seq/frame_",".png",313,"../../videos/kinect_tshirt/mesh_9x9.pts","../../videos/kinect_tshirt/mesh_9x9.tri",
                                "../../videos/kinect_tshirt/matches_9x9/sift_matches_1_",
                                "../../videos/kinect_tshirt/matches_9x9/sift_matches_3d_");
    ORB_SLAM2::SystemTest SLAM(arg2,arg,grabberKinect.get_vertices(),grabberKinect.get_indexes(),true);

    /// Kinect_tshirt;
     cv::Mat TcwO =(cv::Mat_<float>(4,4) <<       1.0000000e+00,   0.0000000e+00,   0.0000000e+00,   0.0000000e+00,
           0.0000000e+00,   1.0000000e+00,   0.0000000e+00,   0.0000000e+00,
           0.0000000e+00,   0.0000000e+00,   1.0000000e+00,   0.0000000e+00,
                   0,0,0,1);
     std::ofstream myfilewrite;
     myfilewrite.open ("errorTshirt.txt");
    for (uint i(0);i<312;i++){
        std::vector<double> Error;
        cv::Mat a = grabberKinect.GetRGB(i);
        std::cout << i << std::endl;
        SLAM.TrackMonocular(a,grabberKinect.get_matches(i),i,TcwO);
        std::vector<std::vector<double>> Matches = SLAM.get_Matches3D();
        std::vector<std::vector<double>> GT = grabberKinect.get_matchesGT(i);
        for (uint k(0);k<Matches.size();k++)
            Error.push_back(pow(Matches[k][0]-GT[k][0],2)+pow(Matches[k][1]-GT[k][1],2)+pow(Matches[k][2]-GT[k][2],2));
        double RMS(.0);
        for (auto e:Error){
            RMS = RMS + e/Error.size();
        }
        myfilewrite << i << " "<<sqrt(RMS) <<std::endl;
        //cv::waitKey(0);
    }
    myfilewrite.close();
    SLAM.Shutdown();

    return 0;
}
