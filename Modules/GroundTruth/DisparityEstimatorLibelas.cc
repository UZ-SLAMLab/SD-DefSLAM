#include <DisparityEstimatorLibelas.h>
#include <elas.h>
#include "image.h"

namespace stereodisparity
{
    using namespace std;
    using namespace cv;

    // Process the disparity of an image
    cv::Mat DisparityEstimatorLibelas::process(const cv::Mat &imLeft, const cv::Mat &imRight)
    {
        // load images
        image<uchar> *I1, *I2;
        I1 = new image<uchar>(imLeft.cols, imLeft.rows);
        I1->data = imLeft.data;
        I2 = new image<uchar>(imRight.cols, imRight.rows);
        I2->data = imRight.data;
        // check for correct size
        if (I1->width() <= 0 || I1->height() <= 0 || I2->width() <= 0 || I2->height() <= 0 ||
            I1->width() != I2->width() || I1->height() != I2->height())
        {
            cout << "ERROR: Images must be of same size, but" << endl;
            cout << "       I1: " << I1->width() << " x " << I1->height() << ", I2: " << I2->width() << " x " << I2->height() << endl;
            return cv::Mat();
        }

        // get image width and height
        int32_t width = I1->width();
        int32_t height = I1->height();

        // allocate memory for disparity images
        const int32_t dims[3] = {width, height, width}; // bytes per line = width
        float *D1_data = (float *)malloc(width * height * sizeof(float));
        float *D2_data = (float *)malloc(width * height * sizeof(float));

        // process
        Elas::parameters param(Elas::setting::MEDICAL);
        Elas elas(param);
        elas.process(I1->data, I2->data, D1_data, D2_data, dims);

        // find maximum disparity for scaling output disparity images to [0..255]
        float disp_max = 15;
        for (int32_t i = 0; i < width * height; i++)
        {
            if (D1_data[i] > disp_max)
                disp_max = D1_data[i];
        }

        // copy float to uchar
        image<uchar> *D1 = new image<uchar>(width, height);
        image<uchar> *D2 = new image<uchar>(width, height);

        for (int32_t i = 0; i < width * height; i++)
        {
            D1->data[i] = uint8_t(D1_data[i]);
            D2->data[i] = uint8_t(D2_data[i]);
        }

        cv::Mat img_color;
        cv::Mat img_in(height, width, CV_8UC1, D1->data, 0);
        // cv::imshow("MyDisp", img_in);
        cv::Mat img_norm;
        cv::normalize(img_in, img_norm, 0, 255, NORM_MINMAX, CV_8UC1);

        applyColorMap(img_norm, img_color, COLORMAP_JET);
        //cv::imshow("DispInColor", img_color);

        // free memory
        I1 = nullptr;
        I2 = nullptr;
        // delete D1;
        delete D2;
        free(D1_data);
        free(D2_data);
        return img_in;
    }

} // namespace stereodisparity