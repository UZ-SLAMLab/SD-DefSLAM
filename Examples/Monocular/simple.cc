#include <System.h>
#include <Viewer.h>
#include <opencv2/core/core.hpp>
#include <unistd.h>
int main()
{
  cv::VideoCapture cap(
      "/media/jose/NuevoVol/Dataset/hamlyn.doc.ic.ac.uk/vision/data/Dataset1/stereo.avi");
  if (!cap.isOpened()) // check if we succeeded
    return -1;

  // string arg2 = "../../Vocabulary/ORBvoc.txt";
  uint i(0);
  Timer timer;
  double u(0.0);
  while (true)
  {
    cv::Mat a;
    cap >> a;
    if (a.empty())
      break;
    cv::imshow("a", a);

    char order = cv::waitKey(20) & 0xFF;

    if (a.channels() == 1)
    {
      cv::cvtColor(a, a, cv::COLOR_GRAY2RGB);
    }
    u = u + 1;
    // SLAM.TrackMonocular(a, i);
    i++;
  }

  //  SLAM.Shutdown();

  return 0;
}
