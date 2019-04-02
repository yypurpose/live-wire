#include "stdafx.h"
#include <iostream>
#include<cmath>
#include<queue>
#include<time.h>
#include <windows.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <Commdlg.h>
#include<stdio.h>
#define pi acos(-1)
using namespace  cv;
using namespace  std;
const double INF = 1000000000000000000.0;
const double eps = 1e-8;
typedef pair<int, int>  Pii;
Mat img;                                         //ԭʼͼ�����
Mat fz, fg;                                      //�洢fZ��fGֵ�ľ���
Mat grad_x, grad_y;                              //���������ݶȾ���
Mat abs_grad_x, abs_grad_y;                      //�ݶȾ������ֵ��
Mat dist;                                        //������󣬴洢ÿ���㵽Seed���λ��
Mat pre;                                         //���ھ��������󣬴洢ÿ����ĵ�ǰ���ڵ�
Mat img_draw;                                    //��������
Mat used;                                        //dijkstra��ʹ�ã����޷��ʹ�
Mat D;                                           //fD�е�D����
Mat result_img;                                  //�������
Mat tmp;                                         //��ͼʱʹ�õ�01����
double Cost[2005][2005][8];                      //CostԤ����
double wz = 0.4, wd = 0.4, wg = 0.2;
int dir_x[8] = { -1, -1, -1, 0, 1, 1, 1, 0 };    //����ͨ
int dir_y[8] = { -1, 0, 1, 1, 1, 0, -1, -1 };    //����ͨ

struct P
{
	double first;
	Vec2i second;
	P(double first, Vec2i second)
	{
		this->first = first;
		this->second = second;
	}
	bool operator <(const P&b)const
	{
		return first > b.first;
	}
	bool operator > (const P&b)const
	{
		return first < b.first;
	}
};
struct img_Stack                     //ģ��ջ
{
	Mat img[15];
	Point seed[15];
	int sz;
}Sta;
void init()
{
	fz.create(img.rows, img.cols, CV_8UC1);
	dist.create(img.rows, img.cols, CV_64FC1);
	pre.create(img.rows, img.cols, CV_32SC2);
	used.create(img.rows, img.cols, CV_8UC1);
	D.create(img.rows, img.cols, CV_64FC2);
//	Cost.create(img.rows, img.cols, CV_64FC1);
	tmp.create(img.rows, img.cols, CV_8UC1);
}
/*����s��xΪMat�����ϵ��У�yΪMat�����ϵ���*/
void dijkstra(Point s)
{
	LARGE_INTEGER  nFreq, t1, t2;
	double dt;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&t1);


	swap(s.x, s.y);
	int cnt = 0;
	priority_queue< P > que;
	while (!que.empty())que.pop();
	dist.setTo(INF);
	used.setTo(0);
	dist.ptr<double>(s.x)[s.y] = 0;
	Vec2i start;
	start[0] = s.x;
	start[1] = s.y;
	pre.ptr<Vec2i>(s.x)[s.y] = start;
	que.push(P(0.0, start));

	bool flag = true;
	while (!que.empty())
	{
		P p = que.top();
		que.pop();
		Vec2i v = p.second;
		if (dist.at<double>(v[0], v[1]) < p.first)continue;
		used.ptr<uchar>(v[0])[v[1]] = 1;
		for (int i = 0; i < 8; i++)
		{
			flag = i & 1;
			int tx = v[0] + dir_x[i];
			int ty = v[1] + dir_y[i];
			if (tx<0 || ty<0 || tx >= img.rows || ty >= img.cols)
				continue;
			if (used.ptr<uchar>(tx)[ty] == 1)
				continue;
			Vec2i to = Vec2i(tx, ty);
			double Ipq = Cost[tx][ty][i];
			if (dist.ptr<double>(tx)[ty] > dist.ptr<double>(v[0])[v[1]] + Ipq)
			{
				dist.ptr<double>(tx)[ty] = dist.ptr<double>(v[0])[v[1]] + Ipq;

				pre.ptr<Vec2i>(tx)[ty] = v;
				que.push(P(dist.ptr<double>(tx)[ty], to));

			}
		}
		flag = false;
	}
	QueryPerformanceCounter(&t2);
	dt = (t2.QuadPart - t1.QuadPart) / (double)nFreq.QuadPart;
	cout << "Running time : " << dt * 1000000 << "us" << endl;
}
int LmouseOn;      //�������Ƿ���
int RmouseOn;      //����Ҽ��Ƿ���
Point Seed;        //��ǰ���������ؽڵ�
Point StartPoint;  //ȫ���������ؽڵ�
bool work_flag;    //����Ҽ����Ϊfalse�������ٶ�ͼƬ���в���
void onMouse(int Event, int x, int y, int flags, void* param)
{
	if (Event == CV_EVENT_MOUSEMOVE&&LmouseOn == 2 && work_flag)
	{
		img.copyTo(img_draw);
		Point cur = Point(x, y);
		Point tmp;
		while (cur != Seed)
		{
			tmp = Point(pre.at<Vec2i>(cur.y, cur.x)[1], pre.at<Vec2i>(cur.y, cur.x)[0]);
			line(img_draw, cur, tmp, Scalar(0, 0, 255), 2, CV_AA);
			cur = tmp;
		}
		imshow("Live-Wire Test", img_draw);
	}
	else if (Event == CV_EVENT_LBUTTONDOWN&&work_flag)
	{
		LmouseOn = 1;
	}
	else if (Event == CV_EVENT_LBUTTONUP&&work_flag)
	{
		LmouseOn++;
		if (LmouseOn == 2)
		{
			Sta.sz++;
			img.copyTo(Sta.img[Sta.sz - 1]);
			if(Sta.sz>1)Sta.seed[Sta.sz - 1] = Seed;
			Seed = Point(x, y);
			dijkstra(Seed);
			if (Sta.sz == 1)
			{
				StartPoint = Seed;

			}
			else
			{
				img_draw.copyTo(img);
			}
		}
	}
	else if (Event == CV_EVENT_RBUTTONDOWN&&work_flag)
	{
		RmouseOn = 1;
	}
	else if (Event == CV_EVENT_RBUTTONUP &&Sta.sz>1 && RmouseOn&&work_flag)
	{
		img.copyTo(img_draw);
		Point cur = StartPoint;
		Point tmp;
		while (cur != Seed)
		{
			tmp = Point(pre.at<Vec2i>(cur.y, cur.x)[1], pre.at<Vec2i>(cur.y, cur.x)[0]);
			line(img_draw, cur, tmp, Scalar(0, 0, 255), 2, CV_AA);
			cur = tmp;
		}
		imshow("Live-Wire Test", img_draw);
		RmouseOn = 0;
		work_flag = false;
	}
}
void bfs(int x, int y)
{
	queue<Pii> que;
	que.push(Pii(0, 0));
	tmp.ptr<uchar>(0)[0] = 0;
	int x1, y1;
	while (!que.empty())
	{
		x1 = que.front().first;
		y1 = que.front().second;

		que.pop();
		int tx, ty;
		for (int i = 0; i < 8; i++)
		{
			tx = x1 + dir_x[i];

			ty = y1 + dir_y[i];
			if (tx < 0 || ty < 0 || tx >= img.rows || ty >= img.cols || tmp.ptr<uchar>(tx)[ty] == 0)
				continue;
			else
			{
				tmp.ptr<uchar>(tx)[ty] = 0;
				que.push(Pii(tx, ty));
			}
		}
	}
}
void solve()
{
	Vec3b red;
	red[0] = 0; red[1] = 0; red[2] = 255;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (result_img.at<Vec3b>(i, j)[0] < 20 && result_img.at<Vec3b>(i, j)[1] < 20 && result_img.at<Vec3b>(i, j)[2]>230)
			{
				tmp.ptr<uchar>(i)[j] = 0;
			}
			else tmp.ptr<uchar>(i)[j] = 1;
		}
	}

	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (tmp.ptr<uchar>(i)[j] == 0)
			{
				result_img.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
//	namedWindow("tmp", CV_WINDOW_AUTOSIZE);
//	imshow("tmp", result_img);
	bfs(0, 0);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			if (tmp.ptr<uchar>(i)[j] == 0)
			{
				result_img.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
		}
	}
}

int main()
{
	//	img = imread("1.jpg", CV_LOAD_IMAGE_UNCHANGED);
	img = imread("images//girl_1.png", CV_LOAD_IMAGE_UNCHANGED);  //300x300
	if (img.empty())
	{
		cout << "ͼ�����ʧ�ܣ�" << endl;
		return -1;
	}
	cout << " rws = " << img.rows << " cols = " << img.cols << endl;
	init();
	cout << img.type() << endl;
	Mat grayImage;
	cvtColor(img, grayImage, COLOR_BGR2GRAY);
	GaussianBlur(grayImage, grayImage, Size(5, 5), 0, 0, BORDER_DEFAULT);
	Mat img_canny;
	Canny(grayImage, img_canny, 50, 100);
	fz = 1 - img_canny / 255;
	Sta.sz = 0;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_64F;

	/// �� X�����ݶ�
	Sobel(grayImage, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_x, abs_grad_x);
	/// �� Y�����ݶ�
	Sobel(grayImage, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	/// �ϲ��ݶ�(����)
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, fg);
	Mat angle;
	cartToPolar(grad_x, grad_y, fg, angle);//���ÿ⺯��
	double max_G = 0.0;
	minMaxLoc(fg, 0, &max_G, 0, 0);
	fg = 1 - fg / max_G;

	double Dpx, Dpy, len;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			Dpx = grad_y.at<double>(i, j);
			Dpy = -grad_x.at<double>(i, j);
			len = sqrt(Dpx*Dpx + Dpy*Dpy);
			if (len - 0>eps)
			{
				Dpx /= len;
				Dpy /= len;
			}
			D.at<Vec2d>(i, j)[0] = Dpx;
			D.at<Vec2d>(i, j)[1] = Dpy;
		}
	}
	LARGE_INTEGER  nFreq, t1, t2;
	double dt;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&t1);
	Vec2i p, q;
	double fZ, fG, fD;
	Vec2d Dp, Dq;
	Vec2d Lpq;
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			p[0] = i; p[1] = j;
			Dp = D.at<double>(p[0], p[1]);
			for (int k = 0; k < 8; k++)
			{
				int tx = i+dir_x[k], ty = j+dir_y[k];
				if (tx<0 || ty<0 || tx >= img.rows || ty >= img.cols)
					continue;
				q[0] = tx;
				q[1] = ty;
				
				fZ = fz.ptr<uchar>(q[0])[q[1]];
				fG = fg.ptr<double>(q[0])[q[1]];

				Dq = D.at<double>(q[0], q[1]);
				
				if (Dp.dot(q - p) >= 0)
					Lpq = q - p;
				else
					Lpq = p - q;
				double dp = Dp.dot(Lpq);
				double dq = Lpq.dot(Dq);
				if (!(k & 1))
				{
					dp /= sqrt(2);
					dq /= sqrt(2);
				}
				fD = (acos(dp) + acos(dq)) / pi;
				if ((k&1))fG /= sqrt(2);
				Cost[i][j][k] =  wz*fZ + wd*fD + wg*fG;
			}
		}
	}
	QueryPerformanceCounter(&t2);
	dt = (t2.QuadPart - t1.QuadPart) / (double)nFreq.QuadPart;
	cout << "Running time : " << dt * 1000000 << "us" << endl;
	work_flag = 1;
	LmouseOn = false;
	RmouseOn = false;
	namedWindow("Live-Wire Test", CV_WINDOW_AUTOSIZE);
	setMouseCallback("Live-Wire Test", onMouse);
	imshow("Live-Wire Test", img);

	//�ȴ�ֱ���м�����
	while (true)
	{
		int c = waitKey(0);
		if (c == 13)
		{
			img_draw.copyTo(result_img);
			solve();
			namedWindow("Result", CV_WINDOW_AUTOSIZE);
			imshow("Result", result_img);
		}
		else if (c == 0&&Sta.sz)
		{
			work_flag = 1;
			Sta.img[Sta.sz-1].copyTo(img);
			Sta.img[Sta.sz - 1].copyTo(img_draw);
			imshow("Live-Wire Test", img_draw);
			Seed = Sta.seed[Sta.sz-1];
			Sta.sz--;
			if(Sta.sz)dijkstra(Seed);
			else LmouseOn = 0;
		}
	}

	waitKey(0);
	//����MyWindow�Ĵ���
	destroyWindow("Live-Wire Test");
	return 0;

}

/*
Scalar(B,G,R)
Vec3b �Ĵ洢Ҳ��BGR
line��������ֱ�ߵ�ʱ��Point���������x�ᣬ����y��
Mat��Ϊx�ᣬ��Ϊy��
*/