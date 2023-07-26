
#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>
#include <ctime> 

using namespace std;
using namespace cv;

#define multiple  1.5/*���ʣ�����Ŀ�������*/
#define min -1/*��С��*/
#define max 10000 /*������*/
#define pointSetnumeber 5/*�㼯��С*/
#define speedSetnumber 2/*�ٶȼ��ϴ�С*/
#define blue 0/*��ɫ*/
#define red 1/*��ɫ*/
#define change 1/*ƫ��*/
#define retain 0/*����*/

/*________________������_________________*/
Point2f pointSet[pointSetnumeber];/*����㼯,���Ŀ���*/
int pointNumber = 0;/*���pointSet[pointSetnumeber]����ŵ�*/
int runNumber = 0;/*���Ŀ���Ĵ�����������꿪ʼԤ��*/
float speed;
float acceleratedSpeed;/*�ٶȣ����ٶ�*/
float speedSet[speedSetnumber];/*�ٶȼ���*/
int speedNumber = 0;/*���speedSet[speedSetnumber]������ٶ�*/
float predictdistance;
Point2f predictPoint;/*����Ԥ������Ԥ���*/
float lastDistance = 0;/*��ʼ������*/
int frame;/*֡��*/
int color;/*����ʶ����ɫ��0������ɫ��1�����ɫ*/

int minId;
double minArea = max;/*������Ĵ����*/
Point2f center;  /*�������Բ��������*/
float radius;  /*�������Բ�뾶*/
Point2f oldcenter;   /*��������ԲԲ������*/
Point2f newcenter;  /*���������Բ��������*/
float newradius;  /*���������Բ�뾶*/

int maxId;
double maxArea = min;/*���ɸѡ������������*/
float referenceR;/*�뾶�ο�����*/
Point2f rectMid;/*�뾶�ο���������������������*/
Point2f target;/*Ŀ���*/
bool state;
/*-------------------------------------------------------------------------*/
float distance(Point2f lastPoint, Point2f presentPoint);/*���������ľ���*/
float distance(Point2f lastPoint, Point2f presentPoint) {
	float distance;
	distance = sqrt((presentPoint.x - lastPoint.x) * (presentPoint.x - lastPoint.x) + (presentPoint.y - lastPoint.y) * (presentPoint.y - lastPoint.y));
	return distance;
}


int main()
{
	VideoCapture cap("./test_blue.mp4");
	Mat image;
	color = blue;/*��ɫ*/
	for (;;) {
		cap.read(image);
		resize(image, image, Size(640, 512));
		/*����Ч��չʾ*/
		Mat test;
		image.copyTo(test);

		vector<Mat> imgChannels;
		split(image, imgChannels);
		/*��ɫ*/
		Mat blueImage = imgChannels.at(0) - imgChannels.at(2);
		Mat binaryImage;
		Mat binaryImagecricle;
		/*��ֵ��*/
		threshold(blueImage, binaryImagecricle, 170, 255, THRESH_BINARY);
		threshold(blueImage, binaryImage, 90, 255, THRESH_BINARY);
		/*��ʴ����*/
		Mat element = getStructuringElement(MORPH_RECT, Size(1, 1));
		Mat dstImage;
		erode(binaryImagecricle, dstImage, element);
		/*�ҵ�Բ���˶���Բ�ġ���R*/
		vector<vector<Point>> outlines;
		vector<Vec4i> hierarchies;
		findContours(dstImage, outlines, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
		for (int i = 0; i < outlines.size(); i++) {

			vector<Point>points;
			double area = contourArea(outlines[i]);
			/*����ų�����*/
			if (area < 10 || area>10000)
				continue;
			/*�ҵ�û�и�����������*/
			if (hierarchies[i][3] >= 0 && hierarchies[i][3] < outlines.size())
				continue;
			/*������������*/
			if (hierarchies[i][2] < 0 || hierarchies[i][2] >= outlines.size())
				continue;
			/*������Χ*/
			if (area <= minArea + 10 && area >= minArea - 20) {
				minArea = area;
				minId = i;
				continue;
			}
			/*�����С������*/
			if (minArea >= area)
			{
				minArea = area;
				minId = i;
			}
		}
		/*��ֹminId���ڷ�Χ�ڱ���*/
		if (minId >= 0 && minId < outlines.size())
		{
			/*�����Բ���ҵ�Բ��*/

			minEnclosingCircle(Mat(outlines[minId]), newcenter, newradius);
			/*��С���������*/
			if (distance(newcenter, center) < 2) {
			}
			else {
				oldcenter = center;
				center = newcenter;
			}
			if (fabs(newradius - radius) < 2) {
			}
			else {
				radius = newradius;
			}
			circle(test, center, radius, Scalar(0, 0, 255), 1, 8, 0);
		}
		else {
			continue;
		}


		/*���Ͳ���*/
		element = getStructuringElement(0, Size(3, 3));
		Mat dilateImage;
		/*dilate���һ�����������ʹ���*/
		dilate(binaryImage, dilateImage, element, Point(-1, -1), 2);
		/*��������*/
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(dilateImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
		for (int i = 0; i < contours.size(); i++) {
			vector<Point>points;
			double area = contourArea(contours[i]);
			/*����ų�����*/
			if (area < 20 || area>10000)
				continue;
			/*�ҵ�û�и�����������*/
			if (hierarchy[i][3] >= 0 && hierarchy[i][3] < contours.size())
				continue;
			/*��û��������*/
			if (hierarchy[i][2] >= 0 && hierarchy[i][2] < contours.size())
				continue;
			/*�������������*/
			if (maxArea <= area)
			{
				maxArea = area;
				maxId = i;
			}
			/*������Χ*/
			if (area <= maxArea + 50 && area >= maxArea - 50) {
				maxArea = area;
				maxId = i;
			}
			cout << maxArea << endl;

		}
		if (maxId >= 0 && maxId < contours.size()) {
			/*�����*/
			Moments rect;
			rect = moments(contours[maxId], false);
			/*�������ľ�:*/
			Point2f rectmid;
			rectmid = Point2f(rect.m10 / rect.m00, rect.m01 / rect.m00);
			/*�������λ����*/
			drawContours(test, contours, maxId, Scalar(0, 255, 255), 1, 8);

				/*��С����*/
				if (runNumber < 2) {
					referenceR = distance(rectmid, center);
					rectMid = rectmid;
				}
				else if (distance(rectmid, center) <= referenceR + 2 && distance(rectmid, center) >= referenceR - 2 && distance(rectmid, rectMid) < 0.5) {
				}
				else {
					referenceR = distance(rectmid, center);
					rectMid = rectmid;
				}

				/*����������λ���ĵ�*/
				circle(test, rectMid, 1, Scalar(0, 255, 255), -1, 8, 0);
				/*2��1����������λ,���*/
				/*��һ����*/
				if (rectMid.x >= center.x && rectMid.y <= center.y) {
					target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y - (center.y - rectMid.y) * multiple);

				}
				/*�ڶ�����*/
				if (rectMid.x <= center.x && rectMid.y <= center.y) {
					target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y - (center.y - rectMid.y) * multiple);

				}
				/*��������*/
				if (rectMid.x <= center.x && rectMid.y >= center.y) {
					target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y + (rectMid.y - center.y) * multiple);

				}
				/*��������*/
				if (rectMid.x >= center.x && rectMid.y >= center.y) {
					target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y + (rectMid.y - center.y) * multiple);

				}
				circle(test, target, 1, Scalar(0, 255, 255), -1, 8, 0);

				/*���������ĵ����㼯*/
				pointSet[pointNumber] = target;
				pointNumber++;
				/*ʵ���µ��滻�ɵ�*/
				if (pointNumber == pointSetnumeber) {
					pointNumber = 0;
				}

			}
			else {
				continue;
			}
			/*��ƫ��*/
			if (state == change) {
				float xchange;
				float ychange;
				xchange = center.x - oldcenter.x;
				ychange = center.y - oldcenter.y;
				/*�ı�㼯*/
				if (pointNumber == 0) {
					pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
					pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
					pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
					pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
				}
				if (pointNumber == 1) {
					pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
					pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
					pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
					pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
				}
				if (pointNumber == 2) {
					pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
					pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
					pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
					pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
				}
				if (pointNumber == 3) {
					pointSet[3] = Point2f(pointSet[3].x + xchange, pointSet[3].y + ychange);
					pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
					pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
					pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
				}
				if (pointNumber == 4) {
					pointSet[4] = Point2f(pointSet[4].x + xchange, pointSet[4].y + ychange);
					pointSet[0] = Point2f(pointSet[0].x + xchange, pointSet[0].y + ychange);
					pointSet[1] = Point2f(pointSet[1].x + xchange, pointSet[1].y + ychange);
					pointSet[2] = Point2f(pointSet[2].x + xchange, pointSet[2].y + ychange);
				}
			}


			/*Ԥ��*/
			if (runNumber > pointSetnumeber) {

				int i = pointNumber - 1;//ȡ���µĵ����ٶ�
				int number1 = i;
				if (number1 < 0) {
					number1 += pointSetnumeber;
				}
				int number2 = i - 1;
				if (number2 < 0) {
					number2 += pointSetnumeber;
				}
				int number3 = i - 3;
				if (number3 < 0) {
					number3 += pointSetnumeber;
				}
				int number4 = i - 4;
				if (number4 < 0) {
					number4 += pointSetnumeber;
				}
				/*ȡ����ĵ㣬���ٶȣ�����ٶ�*/
				speed = distance(pointSet[number1], pointSet[number2]) * frame;
				speedSet[0] = speed;
				speed = distance(pointSet[number3], pointSet[number4]) * frame;
				speedSet[1] = speed;
				acceleratedSpeed = fabs((speedSet[0] - speedSet[1]) * frame);

				/* X = V0T + 1 / 2AT'2��ͨ�����빫ʽ����Ԥ��Ĵ������� */
				predictdistance = 4.5 * speedSet[0] / frame + 1 / 2 * acceleratedSpeed / frame / frame * 18;


				/*���Ԥ��ʱx, y������ֵ�ı�ֵ*/

				float xRatio, yRatio;
				xRatio = fabs(pointSet[number1].x - pointSet[number2].x) / distance(pointSet[number1], pointSet[number2]);
				yRatio = fabs(pointSet[number1].y - pointSet[number2].y) / distance(pointSet[number1], pointSet[number2]);
				/*��һ������  ˳  ����*/
				if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y) {
					predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
				}
				/*�ڶ�������  ˳  ����*/
				if (pointSet[number1].x >= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y) {
					predictPoint = Point2f(pointSet[number1].x + predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
				}
				/*����������  ˳  һ��*/
				if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y <= pointSet[number2].y) {
					predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y - predictdistance * yRatio);
				}
				/*����������  ˳  ����*/
				if (pointSet[number1].x <= pointSet[number2].x && pointSet[number1].y >= pointSet[number2].y) {
					predictPoint = Point2f(pointSet[number1].x - predictdistance * xRatio, pointSet[number1].y + predictdistance * yRatio);
				}



				/*����Ԥ�ⶶ��*/
				lastDistance = predictdistance;

				/*��켣���·��*/
				/*��Բ*/
				circle(test, center, distance(center, target) + 3, Scalar(0, 255, 255), 1, 8, 0);
				/*Ԥ�����Բ����£*/
			/*��һ����*/
				if (predictPoint.x >= center.x && predictPoint.y <= center.y) {
					predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target) / distance(center, predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));

				}
				/*�ڶ�����*/
				if (predictPoint.x <= center.x && predictPoint.y <= center.y) {
					predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y - (center.y - predictPoint.y) * distance(center, target) / distance(center, predictPoint));

				}
				/*��������*/
				if (predictPoint.x <= center.x && predictPoint.y >= center.y) {
					predictPoint = Point2f(center.x - (center.x - predictPoint.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));

				}
				/*��������*/
				if (predictPoint.x >= center.x && predictPoint.y >= center.y) {
					predictPoint = Point2f(center.x + (predictPoint.x - center.x) * distance(center, target) / distance(center, predictPoint), center.y + (predictPoint.y - center.y) * distance(center, target) / distance(center, predictPoint));

				}
				/*����Ԥ���*/
				circle(test, predictPoint, 2, Scalar(0, 0, 255), -1, 8, 0);

			}


			imshow("1", test);
			imshow("2", dilateImage);
			waitKey(1);
		}
	}
