
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

typedef struct armor_point {
    cv::Point point;
    double dis;
}armor_point;

void drawRotatedRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness)
{
    cv::Point2f Vertex[4];
    rect.points(Vertex);
    for (int i = 0; i < 4; i++)
    {
        cv::line(img, Vertex[i], Vertex[(i + 1) % 4], color, thickness);
    }
}
double distance(cv::Point a, cv::Point b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

int main()
{
    string video_path = "./test_blue.mp4";
    cv::VideoCapture cap;
    cap.open(video_path);


    // cv::VideoWriter writer;
    // int coder = cv::VideoWriter::fourcc('M','J','P','G');//选择编码格式
    // double fps=25.0;//设置视频帧率
    // string filename="live.avi";//保存的视频文件名称
    // writer.open(filename,coder,fps,cv::Size(1350,1080),true);//创建保存视频文件的视频流
    // if(!writer.isOpened()){
    //     cout<<"打开视频文件失败，请确认是否为合法输入"<<endl;
    //     return -1;
    // }

    while (true) {
        cv::Mat src_frame;
        cap >> src_frame;
        cv::resize(src_frame, src_frame, cv::Size(640, 480));

        vector<cv::Mat> bgr_images;
        cv::split(src_frame, bgr_images);
        cv::Mat b_src_img = bgr_images[0];
        cv::blur(b_src_img, b_src_img, cv::Size(3, 3));

        cv::Mat threshold_img;
        cv::threshold(b_src_img, threshold_img, 130, 255, cv::THRESH_BINARY);

        vector<vector<cv::Point>> contours;
        cv::findContours(threshold_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // cv::drawContours(src_frame,contours,-1,cv::Scalar(0,0,255));

        cv::Point r_center; // R 的中心点
        vector<cv::RotatedRect> contours_min_rects;//所有轮廓的最小外接矩形
        vector<cv::RotatedRect> armor_min_rects;
        vector<cv::RotatedRect> target_min_rects;
        vector<cv::RotatedRect> r_min_rects; //R

        int r_index = -1;
        // int cnt = 0;
        for (unsigned int contour_index = 0; contour_index < contours.size(); contour_index++) {
            //     //寻找最小旋转矩形,放进容器，顺便将面积过大的剔除,长宽比悬殊的剔除
            cv::RotatedRect minrect = minAreaRect(contours[contour_index]);
            cv::Rect rect = boundingRect(contours[contour_index]);

            if (minrect.size.area() <= 6000.0 && minrect.size.area() > 150) {
                float width;
                float height;
                //判断长宽比
                if (minrect.size.width > minrect.size.height) {
                    width = minrect.size.width;
                    height = minrect.size.height;
                }
                else {
                    width = minrect.size.height;
                    height = minrect.size.width;
                }
                if (width / height < 5) {
                    contours_min_rects.push_back(minrect);
                    if (minrect.size.area() > 200 && minrect.size.area() < 350 && minrect.center.y > 100) { // find R
                        if (height / width > 0.85) {
                            // R_minRects.push_back(minrect);
                            r_center = minrect.center;
                            cv::circle(src_frame, minrect.center, 15, cv::Scalar(5, 255, 100));
                            r_index = contour_index;
                            // std::cout<<cnt++<<std::endl;    
                        }
                    }
                    else {
                        if (minrect.size.area() > 300 && minrect.size.area() < 4350 && (height / width) < 0.7) {
                            armor_min_rects.push_back(minrect);
                        }
                    }
                }
            }
        }
        bool find_ok = false;
        for (int i = 0; i < armor_min_rects.size() - 1; i++) {
            for (int j = i + 1; j < armor_min_rects.size(); j++) {
                double dis = distance(armor_min_rects[i].center, armor_min_rects[j].center);
                if (dis < 100) {
                    target_min_rects.push_back(armor_min_rects[i]);
                    target_min_rects.push_back(armor_min_rects[j]);
                    find_ok = true;
                    break;
                }
                // std::cout<<dis<<std::endl;
            }
            if (find_ok) {
                break;
            }
        }
        if (target_min_rects.size() != 2) {
            continue;
        }
        else {
            cv::RotatedRect rrect_in; //= target_minRects[0];
            cv::RotatedRect rrect_out;// = target_minRects[1];
            double dis1 = distance(r_center, target_min_rects[0].center);
            double dis2 = distance(r_center, target_min_rects[1].center);
            if (dis1 > dis2) {
                rrect_in = target_min_rects[1];
                rrect_out = target_min_rects[0];
            }
            else {
                rrect_in = target_min_rects[0];
                rrect_out = target_min_rects[1];
            }
            drawRotatedRect(src_frame, rrect_in, cv::Scalar(0, 250, 0), 1);
            drawRotatedRect(src_frame, rrect_out, cv::Scalar(0, 0, 255), 1);

            cv::Point target_center = cv::Point((int)((rrect_in.center.x + rrect_out.center.x) / 2), (int)((rrect_in.center.y + rrect_out.center.y) / 2));
            cv::circle(src_frame, target_center, 5, cv::Scalar(0, 0, 255), -1);
            cv::Point2f in_vet_points[4];
            cv::Point2f out_vet_points[4];
            rrect_in.points(in_vet_points);
            rrect_out.points(out_vet_points);
            vector<armor_point> armor_points;
            for (int i = 0; i < 4; i++) {
                armor_point point;
                point.point = in_vet_points[i];
                point.dis = distance(target_center, in_vet_points[i]);
                armor_points.push_back(point);
            }
            for (int i = 0; i < 4; i++) {
                armor_point point;
                point.point = out_vet_points[i];
                point.dis = distance(target_center, out_vet_points[i]);
                armor_points.push_back(point);
            }
            sort(armor_points.begin(), armor_points.end(), [](armor_point a, armor_point b) {return a.dis < b.dis; });
            for (int i = 0; i < 4; i++)
            {
                cv::circle(src_frame, armor_points[i].point, 3, cv::Scalar(255, 0, 255), -1);
            }
            int buff_run_radius = (int)distance(r_center, target_center);
            cv::circle(src_frame, r_center, buff_run_radius, cv::Scalar(55, 110, 255), 1);
        }
        // cv::resize(src_frame,src_frame,cv::Size(1350,1080));
        // writer.write(src_frame);//把图像写入视频流
        cv::imshow("src_img", src_frame);
        // cv::imshow("threshold_img",threshold_img);
        cv::waitKey(0);
        // writer.release();
        cap.release();
        return 0;
    }
}