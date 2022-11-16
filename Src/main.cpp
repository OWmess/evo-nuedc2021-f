
#include <opencv2/opencv.hpp>
#include <iostream>
#include <serial.h>
#include <threshred.h>


auto erodeKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
auto rectKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
auto sqKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
auto dilateKernel= cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
auto SAVE_NAME = "../NUMBER_1000.xml";
std::vector <cv::Mat> digits;
auto svm = cv::Algorithm::load<cv::ml::SVM>(SAVE_NAME);
    std::vector <std::vector<cv::Point2i>> cnt;
    std::vector <cv::Vec4i> hierarchy;
    cv::Mat img_gray, img_temp;
    cv::Mat roi_up, roi_down, roi_left, roi_right;
    Serial st;
    ThreshRed tr;
int findNumber(cv::VideoCapture cap);
cv::Point getCenterPoint(cv::Rect rect);
void get_roi(cv::Mat src);
int main() {
    cv::VideoCapture cap;
    
    cap.open(0);
    cap.set(3, 320);
    cap.set(4, 320);
    if (!cap.isOpened())
        return -1;
    st.openPort();
    
    int num[10] = { 0 };
    bool flag[10] = { false };
    std::cout << "start after 1 sec!" << std::endl;
    cv::waitKey(1000);
    auto resultNumber = findNumber(cap);
    std::cout << "resultNumber is " << resultNumber << std::endl;
    std::cout << "start find result number after 1 sec!" << std::endl;
    cv::waitKey(1000);
    auto key = 0;
    int64 findNumber = 0;
    std::vector <cv::Rect>digit_rect;
    cv::Mat frame;
    cv::Mat line_img;
    cv::Mat img_temp_line;
    cv::Point2f lineCenter;
    std::vector<cv::Vec4i> line;
    std::cout<<"Start!"<<std::endl;
    int64_t lineNum=0;
    while (key != 27) {
        double time=cv::getTickCount();
        unsigned char sendData=0;
        unsigned char lineFlag=0;
        lineCenter=cv::Point2f(frame.cols,frame.cols);
        cap >> frame;
        if (frame.empty())
            break;
       //std::cout << "FRAME SIZE IS" << frame.rows << "x" << frame.cols << std::endl;
        
        cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);  
        cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0, 0);
        //threshold(img_gray, img_temp, 120, 255, THRESH_BINARY_INV);
        cv::threshold(img_gray, img_temp, 0, 255, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
        img_temp_line=tr.threshold_red(frame);
        cv::imshow("line_gray",img_temp_line);
        cv::erode(img_temp, img_temp, erodeKernel);
        
        cv::HoughLinesP(img_temp_line,line,1,CV_PI/180,180,30,0);
        
        //cv::drawContours(frame, line, -1, (0, 255, 255), 3);
        
        bool fline1=false;
        bool fline2=false;
        for(int i=0;i<line.size();i++){
            auto p1=cv::Point(line[i][0],line[i][1]);
            auto p2=cv::Point(line[i][2],line[i][3]);
            cv::line(frame,p1,p2,cv::Scalar(255,255,0));
            if(abs(p2.x-p1.x)>frame.cols/3){
                fline1=true;
            }
            if(abs(p2.y-p1.y)>frame.cols/3){
                fline2=true;
                lineCenter=cv::Point2f((line[i][0]+line[i][2])/2,(line[i][1]+line[i][3])/2);
            }
        }
        if(fline1&&fline2){
            lineNum++;
            std::cout<<"founded"<<std::endl;
        }
        if(lineNum>=5){
            lineFlag=1;
            lineNum=0;
        }    
        //lineCenter=cv::Point2f((line[0][0]+line[0][2])/2,(line[0][1]+line[0][3])/2);
        cv::findContours(img_temp, cnt, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(frame, cnt, -1, (0, 255, 0), 3);
        for (const auto& i : cnt) {
            auto tmp = boundingRect(i);
            if ((tmp.area() > 1000 && tmp.area() < 6000) && (tmp.width * 2 > tmp.height)) {//TODO 
                auto digit_roi = img_gray(tmp);
                cv::threshold(digit_roi, digit_roi, 120, 255, cv::THRESH_BINARY_INV);//TODO
                cv::resize(digit_roi, digit_roi, cv::Size(28, 28));
                digits.push_back(digit_roi);
                digit_rect.push_back(tmp);
                
                //std::cout<<"digits_size="<<digits.size()<<std::endl;
            }
        }
        int lc=0;
        int x=0;
        for (const auto& img : digits) {
            cv::Mat p = img.reshape(1, 1);
            p.convertTo(p, CV_32FC1);
            normalize(p, p);
            int response = (int)svm->predict(p);
            if (response == resultNumber){
                findNumber++;
                lc=x;
            }
            if (response != 0)num[response]++;
            x++;
        }
        if (findNumber >= 5) {//TODO
            //std::cout << "[INFO]foud result number!" << std::endl;
            cv::imshow("number", frame(digit_rect[lc]));
            //std::cout << digit_rect[lc] << std::endl;
            auto center = getCenterPoint(digit_rect[lc]);
            if (center.x <= lineCenter.x) {
                sendData = 1;
            }else { 
                sendData = 2;
            }
            printf("send data is %c\n",sendData);
            findNumber = 0;
        }
        digits.clear();
        digit_rect.clear();
        cv::imshow("contours", frame);
        cv::imshow("thresh", img_temp);
        st.sendCmd(sendData,lineFlag);
        key=cv::waitKey(1);
        double bias=(double)(cv::getTickCount()-time)/(double)cv::getTickFrequency();
        std::cout<<"fps is "<<bias<<std::endl;
    }
    digit_rect.clear();
    digits.clear();
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

int findNumber(cv::VideoCapture cap) {
    int num[10] = { 0 };
    double lastTime = cv::getTickCount();
    double biasTime = 0;
    while (biasTime < 2.0) {

        cv::Mat frame;
        cap >> frame;
        if (frame.empty())
            break;
        cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY); 
        cv::GaussianBlur(img_gray, img_gray, cv::Size(5, 5), 0, 0);
        cv::threshold(img_gray, img_temp, 120, 255, cv::THRESH_BINARY_INV);
        cv::erode(img_temp, img_temp, erodeKernel);
        cv::GaussianBlur(img_temp, img_temp, cv::Size(5, 5), 0, 0);
        cv::findContours(img_temp, cnt, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::drawContours(frame, cnt, -1, (0, 255, 0), 3);
        for (const auto& i : cnt) {
            auto tmp = cv::boundingRect(i);
            if ((tmp.area() > 1000 && tmp.area() < 6000) && (tmp.width * 2 > tmp.height)) {//TODO
                auto digit_roi = img_temp(tmp);
                cv::resize(digit_roi, digit_roi, cv::Size(28, 28));
                digits.push_back(digit_roi);
                //std::cout<<"digits_size="<<digits.size()<<std::endl;
            }
        }
        for (const auto& img : digits) {
            cv::Mat p = img.reshape(1, 1);
            p.convertTo(p, CV_32FC1);
            normalize(p, p);
            int response = (int)svm->predict(p);
            if (response != 0)num[response]++;
        }
        cv::imshow("contours", frame);
        cv::imshow("thresh", img_temp);
        digits.clear();
        cv::waitKey(1);
        biasTime = (cv::getTickCount() - lastTime) / cv::getTickFrequency();
    }
    int maxNum = num[1];
    int finalNumber = 1;
    for (int i = 2; i < 9; i++) {
        if (num[i] > maxNum) {
            finalNumber = i;
            maxNum = num[i];
        }
    }
    return finalNumber;
}

cv::Point getCenterPoint(cv::Rect rect)

{
    cv::Point cpt;

    cpt.x = rect.x + cvRound(rect.width / 2.0);

    cpt.y = rect.y + cvRound(rect.height / 2.0);

    return cpt;

}
