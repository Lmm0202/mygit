#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <memory>
#include <thread>

using namespace std;
using namespace cv;
using namespace boost::asio;

void _read(char *d,boost::system::error_code ec,std::size_t transferred)
{
        cout.write(d, transferred);
}

void write(string a,int c)
{
	 	io_service mios;
		serial_port port(mios, "/dev/pts/19");
		port.set_option(serial_port::baud_rate(19200));
        port.set_option(serial_port::flow_control(serial_port::flow_control::none));
        port.set_option(serial_port::parity(serial_port::parity::none));
        port.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        port.set_option(serial_port::character_size(8));
		
		write(port, buffer(a,c));
		char d[100];
        async_read(port, buffer(d), boost::bind(_read, d, _1, _2));
		
		boost::asio::deadline_timer timer(mios);
		timer.expires_from_now(boost::posix_time::millisec(10));
        timer.async_wait(boost::bind(&boost::asio::serial_port::cancel,  &port));
		mios.run();
}


class shibie
{
	public:
			shibie(string name)
		  	   	   :capture(name){}
			void find ();
		    
	private:
			VideoCapture capture;
};

void shibie::find()
{
	while (capture.isOpened()) {
        Mat frame;
        capture>>frame;
        if(frame.empty())
            {
                break;
            }

		Mat hsvimg;
        cvtColor(frame,hsvimg,COLOR_BGR2HSV);
		
		vector<Mat> hsvsplit;
	    split(hsvimg,hsvsplit);
        equalizeHist(hsvsplit[2],hsvsplit[2]);
        merge(hsvsplit,hsvimg);

		Mat threshold;
        inRange(hsvimg,Scalar(160,160,245),Scalar(180,255,255),threshold);
		
		RotatedRect s;
		bool b = false;
		vector<vector<Point> > contour;
		findContours(threshold, contour, RETR_CCOMP , CHAIN_APPROX_SIMPLE);
		for (int i=0; i<contour.size(); i++)
            {
                if (contour[i].size()> 25)  //判断当前轮廓是否大于25个像素点
                {
                    b = true;
                    s = fitEllipse(contour[i]);  //拟合目标区域成为椭圆，返回一个旋转矩形（中心、角度、尺寸）
                    for (int n = 0; n < 10; n++)
                    {
                        for (int j = 0; j < 30; j++)  //遍历以旋转矩形中心点为中心的10*30的像素块
                      m  {
                            if (s.center.y - 2 + j > 0 && s.center.y - 2 + j < 480 && s.center.x - 2 + n > 0 && s.center.x - 2 + n <  640)  //判断该像素是否在有效的位置
                            {   
                                Vec3b a = hsvimg.at<Vec3b>((int)(s.center.y - 2 + j), (int)(s.center.x - 2 + n)); //获取遍历点点像素值
                               //判断hsv图像中，中心是否为红色
                                if (a[0]>0 && a[1]>0 && a[2]<250)
                                    b = false;        //如果中心不是红色，则不是目标
                            }
                        }
                    }
                    if (b)
                    {
						circle(frame,s.center,3,Scalar(255,0,0),-1,8,0);
						string str1=to_string(s.center.x);
						string str2=to_string(s.center.y);
						string str3=str1+' '+str2;
						int e=str3.length();
						write(str3,e);
                    }
                }

            }
		namedWindow( "find", 0);
  		imshow( "find",frame);
		int key=cvWaitKey(5);
        if (key==27)
        {
         break;
        }
     }
}
 
int main()
{
	shibie one("task.mp4");
	thread t(&shibie::find,one);
	t.join();
	return 0;
}
