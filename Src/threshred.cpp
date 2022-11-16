#include "threshred.h"
void ThreshRed::remove_light(cv::Mat &src_gray)
{

    src_max_data = src_gray.at<uchar>(0, 0);
    for (int i = 0; i < src_gray.cols; i++)
        for (int j = 0; j < src_gray.rows; j++)
        {

            if (src_gray.at<uchar>(j, i) > src_max_data)
                src_max_data = src_gray.at<uchar>(j, i);
        }
    threshold_data = src_max_data * 0.9;
    cv::threshold(src_gray, src_gray, threshold_data, 255, cv::THRESH_BINARY);
    cv::bitwise_not(src_gray, src_gray);

}

cv::Mat ThreshRed::threshold_red(cv::Mat src)
{

    cv::split(src, srcs);

    cv::cvtColor(src, src_gray, cv::COLOR_BGR2GRAY);
    remove_light(src_gray);

    cv::subtract(srcs[2], srcs[0], src_red);
    cv::subtract(srcs[2], srcs[1], src_green);

    //cv::imshow("src_red", src_channels[0]);

    //src_max = src_red & src_green;
    src_max = src_red & src_green & src_gray;
    src_max_data = src_max.at<uchar>(0, 0);
    for (int i = 0; i < src_max.cols; i++)
        for (int j = 0; j < src_max.rows; j++)
        {
//            if (src_red.at<uchar>(j, i) * 1.2 <= 255)
//                src_red.at<uchar>(j, i) = src_red.at<uchar>(j, i) * 2;

            if (src_max.at<uchar>(j, i) > src_max_data)
                src_max_data = src_max.at<uchar>(j, i);

        }
    threshold_data = (int)(src_max_data * 0.3);

    if (threshold_vector.size() < 20)
    {
        mix_data = mix_data + threshold_data;

        if (threshold_vector.size() != 0)
            avr_data = mix_data / threshold_vector.size();

        threshold_vector.push_back(threshold_data);



    }

    else if(threshold_vector.size() >= 20)
    {

        if (threshold_data > avr_data)
            threshold_vector.push_back(25);
        else
            threshold_vector.push_back(threshold_data);

        threshold_vector.erase(std::begin(threshold_vector));

        mix_data = mix_data + threshold_vector[threshold_vector.size()] - threshold_vector[0];

        avr_data = mix_data / threshold_vector.size();

        //std::cout<<threshold_vector.size()<<std::endl;
    }
    //std::cout << "mix_data  " << mix_data << "avr_data  " << avr_data << std::endl;
    cv::threshold(src_max, src_max, avr_data ,255, cv::THRESH_BINARY);
    //std::cout << threshold_data<<"  "<< avr_data << std::endl;
    cv::morphologyEx(src_max, src_max, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::dilate(src_max, src_max, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    return src_max;
}
