#include "opencv2/highgui/highgui.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{

    
    Mat frame;

    namedWindow("MyVideo"); //create a window called "MyVideo"

        int ht=0, wd=0, temp;
        Vec3i color;
        cin>>ht;
        cin>>wd;
        //cout<<ht<<" "<<wd<<"\n";
        Size size(ht, wd);
        frame=Mat::zeros( size, CV_8UC3 );
    while(1)
    {
        for(int i=0;i<ht;i++)
          {  for(int j=0;j<wd;j++)
            {
                cin>>temp;
                color.val[0]=(uchar)temp;
                cin>>temp;
                color.val[1]=(uchar)temp;
                cin>>temp;
                color.val[2]=(uchar)temp;
               // cout<<color.val[0]<<" "<<color.val[1]<<" "<<color.val[2]<<"\n";
                frame.at<Vec3i>(Point(i,j)) = color;
            }
        }
        imshow("MyVideo", frame);
        waitKey(1);
    }

    return 0;

}