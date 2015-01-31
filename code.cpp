#include <stdio.h>
#include "iostream"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <queue>
using namespace std;
using namespace cv;
const int lmargin = 10;
const int rmargin = 790;
const int umargin = 10;
const int dmargin = 590;
const int r = 16;
const int t = 16;
const int R = 50;
const int lcols = 460;
Point pos;
Point bcentre;
VideoCapture v("c.avi");
Mat rgb_to_hsl(Mat img1)
{
        Mat img2;
        cvtColor(img1, img2, CV_BGR2HLS);
        //imshow("2", img2);
        return (img2);
}
 
Mat blob(Mat img)
{
        queue<Point> q;
        int i, j, c = 1, m, n;
        Mat x(img.rows, img.cols, CV_8SC1, Scalar(-1));
        for (i = 0; i < img.rows; i++)
        {
                for (j = 0; j < img.cols; j++)
                {
                        m = i; n = j;
                        if (img.at<uchar>(i, j) == 255 && x.at<char>(i, j) == -1)
                        {
                                Point temp(m, n);
                                q.push(temp);
                                x.at<char>(i, j) = c;
                                while (!q.empty())
                                {
                                        for (int k = m - 1; k <= m + 1; k++)
                                        {
                                                for (int l = n - 1; l <= n + 1; l++)
                                                {
                                                        if (img.at<uchar>(k, l) == 255 && x.at<char>(k, l) == -1)
                                                        {
                                                                Point temp3(k, l);
                                                                q.push(temp3);
                                                                x.at<char>(k, l) = c;
                                                        }
                                                }
                                        }
                                        q.pop();
                                        if (!q.empty())
                                        {
                                                Point temp2;
                                                temp2 = q.front();
                                                m = temp2.x;
                                                n = temp2.y;
                                        }
                                }
                                c++;
                        }
                }
        }
        return x;
}
Point centreball(Mat img1)
{
        Mat img2;
        //imshow("pop2", img1);
        cvtColor(img1, img2, CV_BGR2HLS);
        //imshow("pop1", img2);
        Mat img3(img1.rows, img1.cols, CV_8UC1, Scalar(0));
        for (int i = 0; i < img1.rows; i++)
        {
                for (int j = 0; j < img1.cols; j++)
                {
                        if ((img2.at<Vec3b>(i, j)[0]<60 + 5) && (img2.at<Vec3b>(i, j)[0]>60 - 5)) img3.at<uchar>(i, j) = 255;
                }
        }
        //namedWindow("pop", CV_WINDOW_AUTOSIZE);
        //imshow("pop", img3);
        Mat img4(img3.rows, img3.cols, CV_8SC1, Scalar(-1));
        img4 = blob(img3);
        int max_val, val, max_no = 0, no;
        for (int i =20; i < img4.rows-70; i++)
        {
                for (int j = 16; j < img4.cols-16; j++)
                {
                        if (img4.at<char>(i, j) != -1)
                        {
                                no = 0;
                                val = img4.at<char>(i, j);
                                for (int k = 0; k < img4.rows; k++)
                                {
                                        for (int l = 0; l < img4.cols; l++)
                                        {
                                                if (img4.at<char>(k, l) == val)
                                                {
                                                        no++;
                                                }
                                        }
                                }
                                if (no>max_no)
                                {
                                        max_no = no;
                                        max_val = val;
                                }
                        }
                }
        }
        Point center;
        center.x = 0;
        center.y = 0;
        for (int i = 0; i < img4.rows; i++)
        {
                for (int j = 0; j < img4.cols; j++)
                {
                        if (img4.at<char>(i, j) == max_val)
                        {
                                center.x += j;
                                center.y += i;
                        }
                }
        }
        center.x = center.x / max_no;
        center.y /= max_no;
        return center;
}
float slope()
{
        float s=0.0;
        int n = 0;
        int i, j;
       
        Mat frame;
        Point initpos;
        v >> frame;
        bcentre = centreball(frame);
        initpos.x = bcentre.x;
        initpos.y = bcentre.y;
        cout << "(" << initpos.x << "," << initpos.y << ")" << endl;
startslope:
        for (i = 0; i<5; i++)
        {  
                v >> frame;
                //imshow("a", frame);
                bcentre = centreball(frame);
                //cout << i <<endl;
                if ((bcentre.x + r >= rmargin - t) || (bcentre.x - r <= lmargin + t))
                {
                        cout << "in";
                        initpos.x = bcentre.x;
                        initpos.y = bcentre.y;
                        goto startslope;
                }
                if ((bcentre.x - initpos.x) != 0)
                {
                        cout << "(" << bcentre.x << "," << bcentre.y << ")" << endl;
                        cout << "(" << initpos.x << "," << initpos.y << ")" << endl;
                        s =s+ (float)(bcentre.y - initpos.y) / (float)(bcentre.x - initpos.x);
                        cout << s << endl;
                        n++;
                }
        }
        s /= n;
        return s;
}
float ncollisions(float s)
{
        Mat frame;
        int a, b, c, d;
        v >> frame;
        pos.y = lcols;
        pos.x = bcentre.x;
        int x;
  if(s==0)
  return s;
        bcentre = centreball(frame);
        if (s>0)
        {
                d = lcols - (bcentre.y + (rmargin - bcentre.x) *s);
                if (d > 0)
                {
                        c = s*(rmargin - lmargin);
                        b = d / c;
                        a = d%c;
                        x = a / s;
                        cout << "d" << d << " c" << c << " b" << b << " a" << a << " x" << x;
                        if (b % 2 == 0)
                        {
                                s = -s;
                                pos.x = rmargin - x;
                        }
                        else
                                pos.x = x;
                }
                if (d < 0)
                {
                        x = (lcols - bcentre.y) / s;
                        pos.x = bcentre.x + x;
                }
        }
        if (s<0)
        {
                d = lcols - (bcentre.y + bcentre.x *(-s));
                if (d>0)
                {
                        c = (-s)*(rmargin - lmargin);
                        b = d / c;
                        a = d%c;
                        x = a / s;
                        if (b % 2 == 0)
                        {
                                s = -s;
                                pos.x = x;
                        }
                        else
                                pos.x = rmargin - x;
 
                }
                else
                {
                        x = (lcols - bcentre.y) /s;
                        pos.x = bcentre.x + x;
                }
        }
        return s;
}
Point brick()
{
        Mat frame;
        //frame = imread("v5.png", CV_LOAD_IMAGE_COLOR);
        v >> frame;
        //imshow("pop2", frame);
        int val;
        Mat img2;
        //namedWindow("pop2", CV_WINDOW_AUTOSIZE);
       
        cvtColor(frame, img2, CV_BGR2HLS);
        //imshow("pop1", img2);
        Mat img3(frame.rows, frame.cols, CV_8UC1, Scalar(0));
        for (int i = 0; i < frame.rows; i++)
        {
                for (int j = 0; j < frame.cols; j++)
                {
                        if ((img2.at<Vec3b>(i, j)[0]<120 + 5) && (img2.at<Vec3b>(i, j)[0]>120 - 5)) img3.at<uchar>(i, j) = 255;
                }
        }
        imshow("pop5", img3);
        Mat img = blob(img3);
        Point hit, rect;
        hit.x = 0;
        hit.y = 0;
        for (int i = umargin; i < (dmargin + umargin) / 2; i++)
        {
                for (int j = lmargin; j < rmargin; j++)
                {
                        int n = 0;
                        if (img.at<char>(i, j) != -1)
                        {
                                val = img.at<char>(i, j);
                                for (int k = umargin; k < (dmargin + umargin) / 2; k++)
                                {
                                        for (int l = lmargin; l < rmargin; l++)
                                        {
                                                if (img.at<char>(k, l) == val)
                                                        n++;
                                        }
                                }
                        }
                        rect.x = 0;
                        rect.y = 0;
                        if (n>1500)
                        {
                                for (int k = umargin; k < (dmargin + umargin) / 2; k++)
                                {
                                        for (int l = lmargin; l < rmargin; l++)
                                        {
                                                if (img.at<char>(k, l) == val)
                                                {
                                                        rect.x += l;
                                                        rect.y += k;
                                                }
                                        }
                                }
                                rect.x /= n;
                                rect.y /= n;
                        }
                        if (rect.y > hit.y - 10)
                        {
                                if (rect.x > hit.x)
                                        hit = rect;
                        }
                }
        }
        return hit;
}
int paddlept(float slope, Point br)
{
        float theta = atan(slope);
        int i, c;
        float beta;
        Point hit[3];
        float delta[3];
        float alpha[3] = { 0.78, 1.047, 1.57 };
        float min = 0.0;
        int posi;
        for (i = 0; i < 3; i++)
        {
                if (slope < 0)
                        alpha[i] = (-1)*alpha[i];
        }
        for (i = 0; i < 3; i++)
        {
                hit[i].y = lcols + R - R*fabs(sin(alpha[i]));
                hit[i].x = (hit[i].y - pos.y) / slope + pos.x;
                beta = atan((br.y - hit[i].y) / (br.x - hit[i].x));
                delta[i] = 2 * alpha[i] - atan(slope) - beta;
                delta[i] = fabs(delta[i]);
        }
        min = delta[0];
        posi = 0;
        for (i = 1; i < 3; i++)
        {
                if (min > delta[i])
                {
                        min = delta[i];
                        posi = i;
                }
        }
        if (slope > 0)
                c = hit[posi].x + R*cos(alpha[posi]);
        else
                c = hit[posi].x - R*cos(alpha[posi]);
        return c;
}
int main()
{
        Mat img;
        while (1)
        {
                float s;
                v >> img;
                Point bcentre = centreball(img);
                Point b1;
                Point br;
                int final;
               
                v >> img;
                b1 = centreball(img);
                //cout << "b1(" << b1.x << "," << b1.y << endl;
 
                //cout << "bcenter(" << bcentre.x << "," << bcentre.y<<endl;
               
                if ((bcentre.y > 150) && (b1.y>bcentre.y)&&(bcentre.y<250))
                {
                        //cout << "bcenter(" << bcentre.x << "," << bcentre.y << endl;
                        //cout << "b1(" << b1.x << "," << b1.y << endl;
 
                        s = slope();
                        cout << s;
                        s = ncollisions(s);
                        cout << s << endl;
                        cout << "(" << pos.x << "," << pos.y << ")";
                        br = brick();
                        final = paddlept(s, br);
                        cout << "final" << final;
                        break;
                }
 
        }
        /*img = imread("v5.png", CV_LOAD_IMAGE_COLOR);
        namedWindow("pop3", CV_WINDOW_AUTOSIZE);
        imshow("pop3", img);*/
        /*Point a;
        a = centreball(img);
        cout << "(" << a.x << "," << a.y<<endl;*/
        /*a = brick();
        cout << "(" << a.x << "," << a.y;*/
        waitKey(0);
        getchar();
        return 0;
}