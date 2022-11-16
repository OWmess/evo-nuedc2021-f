#include "opencv2/opencv.hpp"
class ThreshRed{
public:
    cv::Mat threshold_red(cv::Mat src);
private:
	cv::Mat srcs[3], src_red, src_green, src_gray,src_max;
    cv::Mat roi_up,roi_down,roi_left,roi_right;
    int src_max_data = 0;
    int threshold_data = 0;
    int cont = 1;
    int avr_data = 0;
    int mix_data = 0;
    std::vector<int> threshold_vector;
    void remove_light(cv::Mat& src_gray);
};
