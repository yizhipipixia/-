#include <iostream>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <stdio.h>

#include <wiringPi.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <cmath>
//#define Blocks 10
#define X 16//通过控制X和Y来调节画面的分割比率(记得将下面过滤函数的一号区域解开注释,并且将下部while换成for,以及注释掉嵌套的while内的i域j,从而使得这个宏定义功能生效,有红绿双份函数要改)
#define Y 12

/* #define zhuang_x1 4 //为提高障碍物判断速度建议取值靠前如4,4 勉强在中间而且靠前
#define zhuang_x2 4
#define zhuang_y1 4
#define zhuang_y2 4   */  //.........................................................得出参数后使用

short int p1 = 0;//障碍物情况判断参数
short int p2 = 0;
short int p3 = 0;

using namespace cv;
//滴滴zo7
using namespace std;


//初始化参数
int p_colour = 0;//红色判断数
//int p_cir = 0;//圆判断数

const int pin0=26;//输入信号 使用const,表明值不可变
const int pin1=27;//out
const int pin2=28;//out zuo
const int pin3=29;//out you

bool pan = 0;//进入两边中间都有障碍物,开启向间隙方向运行函数的参数

bool pan1 = 0;//向左统计移动长度,开启参数




//int fb = 2; int jl = 179; int sx = 230; int dg = 40; int rb = 12; int RR = 123;//设置检测圆的初始化参量。

int zhuang_x1;//前方障碍物范围阈值x值
int zhuang_x2;

int zhuang_y1;//桩点平行y轴值
int zhuang_y2;

int you_x1;//右边有在阻挡范围的障碍物阈值x值
int you_x2;

int zuo_x1;//左边有在阻挡范围的障碍物阈值x值
int zuo_x2;

int gre_l_y = 555;//绿色障碍物参数
int gre_l_x =0;

int l_y = 555;//红色障碍物参数
int l_x =0;



//int iLowH = 35; int iHighH = 85; int iLowS = 40; int iHighS = 255; int iLowV = 90; int iHighV = 255;int tuo = 12;//设置红绿 转灰色参数
//int iLowH = 0; int iHighH = 179; int iLowS = 33; int iHighS = 98; int iLowV = 218; int iHighV = 99;int tuo = 18;//设置形态学参数//红 绿 白 参数
//int iLowH = 108; int iHighH = 103; int iLowS = 35; int iHighS = 93; int iLowV = 111; int iHighV = 47;int tuo = 8;//绿
int iLowH = 0; int iHighH = 18; int iLowS = 131; int iHighS = 255; int iLowV = 63; int iHighV = 0;int guo_lv =10;  int l_zhudang = 4;
//int tuo = 4;//形态学操作


 int p = 0;
 int q = 0;	

 int a[(160/X)+1];//a[11]
 int b[(120/Y)+1];

 
 
int y_x = 0;
int z_x = 0;
int y_x1 = 200;
int z_x1 = 200;
int chu_z_x1 = 0;


int l2_x = 0;
int l2_y = 0;
int chu_y_x = 0;//障碍物开始
int	chu_z_x = 0;
int tong_x = 10;

//根据障碍物坐标,输出应对情况



void Pickup_list()
 
{

/* short int zhuang_x1;//前方障碍物范围阈值
short int zhuang_x2;
short int zhuang_y1;
short int zhuang_y2;
 */



if(   ( ( (gre_l_x > zhuang_x1)&&(gre_l_x < zhuang_x2)) && (gre_l_y <zhuang_y2 )   )      ||     ( (l_x > zhuang_x1)&&(l_x < zhuang_x2) ) &&(l_y < zhuang_y2 )      )//检测到正前方有会影响进桩的障碍物
{
	
	p1 = 1;
	
	
}


if(    ( (gre_l_x > you_x1))&&((gre_l_x < you_x2) ) && (gre_l_y <zhuang_y2 )      ||     ( (l_x > you_x1)&&(l_x < you_x2) ) && (l_y < zhuang_y2 )       ) //右边区域内有障碍物 
 {
	
	p2 = 1;
	if( (gre_l_x > y_x) || (l_x > y_x) )//红色绿色,比较得出值
	{
		if(gre_l_x > l_x)
		{
			y_x = gre_l_x;//注意这里的y_x 和下面的z_x是障碍物最右边的值
		}
		else 
		{
			y_x = l_x;
			
		}
		
	}
	//..............................................................................
		if( (gre_l_x < y_x1) || (l_x < y_x1) )//红色绿色,比较得出值
	{
				if(gre_l_x < l_x)
				{
					y_x1 = gre_l_x;//注意这里的y_x 和下面的z_x是障碍物最左边的值
				}
				else 
				{
					y_x1 = l_x;
					
				}
		
	}
	
	
	
	
	
 }

if(    ( (gre_l_x > zuo_x1)&&(gre_l_x < zuo_x2) ) && (gre_l_y <zhuang_y2 )      ||     ( (l_x > zuo_x1)&&(l_x < zuo_x2) ) && (l_y < zhuang_y2 )       ) //左边有在阻挡范围的障碍物
{
	
	p3 = 1;
	if( (gre_l_x > z_x) || (l_x > z_x) )
	{
		if(gre_l_x > l_x)
		{
			z_x = gre_l_x;//who big,who =
		}
		else 
		{
			z_x = l_x;
			
		}
		
	}




  		if( (gre_l_x < z_x1) || (l_x < z_x1) )//红色绿色,比较得出值
	{
				if(gre_l_x < l_x)
				{
					z_x1 = gre_l_x;//注意这里的y_x 和z_x是障碍物最左边的值
				}
				else 
				{
					z_x1 = l_x;
					
				}
		
	}
	
	
	
	
	
}
//以上函数的调用目的,是为了得出遍历一帧图像(为遍历所以是放在下方颜色过滤的while/for循环内了)后,
//得出障碍物的存在情况(还没有总体判断),随便求出,前左,前右的右最大x坐标,即z_x(左最大),y_x(右最大)




} 






//颜色过滤函数
void Pickup_cor(Mat &inputframe, Mat &outputframe)//红色检测函数
 
{
 
Mat hsvframe;
 
cvtColor(inputframe, hsvframe, COLOR_BGR2HSV);
 
outputframe = Mat(hsvframe.rows, hsvframe.cols,CV_8UC3, cv::Scalar(255, 255, 255));
 
/* int rowNumber = hsvframe.rows;
 
int colNumber = hsvframe.cols; */           // 一号区域
 
double H = 0.0, S = 0.0, V = 0.0;


//测试点坐标
 
/* int z_xx =0;//简称到的颜色区域左右上下的坐标点
int y_xx =0;
int s_xx =0;
int x_xx =0;

int z_yy =0;//简称到的颜色区域左右上下的坐标点
int y_yy=0;
int s_yy=0;
int x_yy=0;
  */
  
  
  
  
  
  //.................................................
 
 

 int g_l = 0;
 int w = 0;
 int z = 0;
 int i = 0;
 int j = 0;
 
	
 
// ...................................................................
//  120:rowNumber,
	//for (int i = 0; i < rowNumber; i++)
	 while(i<120)
	{
	         //160:colNumber
			//for (int j = 0; j < colNumber; j++)//调整了分辨率,要么根据分辨率修改120和160,要么使用for循环.不适用while循环,这里用只是为了尽可能加快程序运行速度
			 while(j<160)
			{


				//以下得到图像HSV值
				H = hsvframe.at<Vec3b>(i, j)[0];
				 
				S = hsvframe.at<Vec3b>(i, j)[1];
				 
				V = hsvframe.at<Vec3b>(i, j)[2];
				 
				 
				if (((H >= iLowH && H <= iHighH) || (H >= iLowS && H <= iHighS)) && S >= iLowV && V >= iHighV)
				 
				{
				 //...............
						   
				
					   
							
					outputframe.at<Vec3b>(i, j)[0] =inputframe.at<Vec3b>(i, j)[0];
					 
					outputframe.at<Vec3b>(i, j)[1] =inputframe.at<Vec3b>(i, j)[1];
					 
					outputframe.at<Vec3b>(i, j)[2] =inputframe.at<Vec3b>(i, j)[2];
					
						
					/* if(i>y_xx)
					{
					    y_xx =i;//右边最大的x坐标
						y_yy =j;
						
					}
					else
					{
					    z_xx = i;//左边最小的x坐标
						z_yy = j;
						
					}
					
					
					if(j>s_xx)
					{
					   s_xx = i;
					   s_yy = j;//上面最大y坐标
					}
					else
					{
					   x_xx = i;
					   x_yy = j;//下面最小y坐标
						
					}
				 */
					
					
					
//...........................................................						
					/* 	for( ; w<=(120/Y);w++)
						{
							if(b[w]<i && i<b[w+1])
							{
								g_l++;
								l2_y = w;break;
								
							}
							
							
						}
						 */
						while(w<10)
						{
							
							if(b[w]<i && i<b[w+1])//画图理解
							{
								g_l++;
								l2_y = w;break;
								
							}
							w++;
							
						}
						
//.....................................................................................................
			/* 			for( ; z<=(160/X);z++)
						{
							if(a[z]<j && j<a[z+1])
							{
								g_l++;
								l2_x = z;break;
								
							}
								
						} */
						
						while(z<10)//这里根据160/X X是为16,即长16为一个分割线域上面的120/Y一起将画面分割成10*10即100份
						{
							if(a[z]<j && j<a[z+1])
							{
							g_l++;
							l2_x = z;break;

							}
							z++;

						}
//...............................................................................................

                           if(  g_l > guo_lv )
						   {
							   
							   l_x = l2_x;
							   l_y = l2_y;
								printf("红色所在分区%d,%d\n",l_x,l_y);//这里是对分割的单元区域统计红色够不够多,免得由于红色的噪点引起误判,输出错误的障碍物坐标
								printf("didi\n");   
							    g_l =0;
							   
						   }
				
				
					
				 
				}

		 

		 
		    j++;
		 
			}
			/* 		printf("障碍物最左端 %d,%d\n",z_xx,z_yy);
		printf("障碍物最右端 %d,%d\n",y_xx,y_yy);
		printf("障碍物最上端 %d,%d\n",s_xx,s_yy);
		printf("障碍物最下端 %d,%d\n",x_xx,x_yy); */
	j =0;			
	i++;
 
    }
 


}
 int gre2_l_x = 0;
 int gre2_l_y = 0 ;
void Pickup_gre(Mat &inputframe, Mat &outputframe)//绿色检测函数
 
{
 
Mat hsvframe;
 
cvtColor(inputframe, hsvframe, COLOR_BGR2HSV);
 
outputframe = Mat(hsvframe.rows, hsvframe.cols,CV_8UC3, cv::Scalar(255, 255, 255));
 
int rowNumber = hsvframe.rows;
 
int colNumber = hsvframe.cols;
 
double H = 0.0, S = 0.0, V = 0.0;
/* 
short int p = 0;
short int q = 0;	

short int a[Blocks];
short int b[Blocks];

short int l_y;
short int l_x;

short int x_p = 0;
short int y_p = 0;
 */
/*   int p = 0;
 int q = 0;	

 int a[(160/X)+1];//a[11]
 int b[(120/Y)+1]; */

 
 int g_l = 0;
 int w = 0;
 int z = 0;
 int i = 0;
 int j = 0;

 
 


/* 
	for( int ii = 0; ii<= 160;ii+=X)
	{
		a[p] = ii;
		p++;

	}

	for( int jj = 0; jj<=120;jj+=Y)
	{
		b[q] = jj;
		q++;

	} */
// ...................................................................
//  120:rowNumber,
	//for (int i = 0; i < rowNumber; i++)
	 while(i<120)
	{
	         //160:colNumber
			//for (int j = 0; j < colNumber; j++)
			 while(j<160)
			{


				//以下得到图像HSV值
				H = hsvframe.at<Vec3b>(i, j)[0];
				 
				S = hsvframe.at<Vec3b>(i, j)[1];
				 
				V = hsvframe.at<Vec3b>(i, j)[2];
				 
				 
				//if (((H >= iLowH && H <= iHighH) || (H >= iLowS && H <= iHighS)) && S >= iLowV && V >= iHighV)
					//int iLowH = 108; int iHighH = 103; int iLowS = 35; int iHighS = 93; int iLowV = 111; int iHighV = 47;int tuo = 8;//绿
				 if (((H >= 108 && H <= 103) || (H >= 35 && H <= 93)) && S >= 111 && V >= 47)
				{
				 //...............
						   
				
					   
							
					outputframe.at<Vec3b>(i, j)[0] =inputframe.at<Vec3b>(i, j)[0];
					 
					outputframe.at<Vec3b>(i, j)[1] =inputframe.at<Vec3b>(i, j)[1];
					 
					outputframe.at<Vec3b>(i, j)[2] =inputframe.at<Vec3b>(i, j)[2];
					
					
					//  120:rowNumber,160:colNumber
					
			
				 


//...........................................................						
					/* 	for( ; w<=(120/Y);w++)
						{
							if(b[w]<i && i<b[w+1])
							{
								g_l++;
								gre2_l_y = w;break;
								
							}
							
							
						}
						 */
						while(w<10)
						{
							
							if(b[w]<i && i<b[w+1])//进行分区,生成坐标块
							{
								g_l++;
								gre2_l_y = w;break;
								
							}
							w++;
							
						}
						
//.....................................................................................................
			/* 			for( ; z<=(160/X);z++)
						{
							if(a[z]<j && j<a[z+1])
							{
								g_l++;
								gre2_l_x = z;break;
								
							}
								
						} */
						
						while(z<10)
						{
							if(a[z]<j && j<a[z+1])
							{
							g_l++;
							gre2_l_x = z;break;

							}
							z++;

						}
//................................................................
                      if(  g_l > guo_lv )
					  {
						  gre_l_x = gre2_l_x;//if gre2_l_x to fill the region,give the value to gre_l_x,to help Pickup_list to filter noise
						  gre_l_y = gre2_l_y;
						  printf("绿色所在分区%d,%d\n",gre_l_x,gre_l_y);
					      printf("didi\n");
						  g_l =0;
						  
					  }
					
				    Pickup_list( );
				 
				}

		    j++;
		    
			
			
						
			}
	
	j =0;		
	i++;
 
    }
		
}



 void tactics()//这个函数是根据最上面的函数p1,p2,p3等全局变量的值来判断现在障碍物的存在情况
 {
	 int spacing_max;//jian ju zui da bian liang
	 int pin_x =27;//zuo
	 


if(p1 && !p2 && !p3)//正前方有会影响进桩的障碍物,左右都没有障碍物
{
	
	 digitalWrite(pin1,1);//
	//digitalWrite(pin2,0);//
	//digitalWrite(pin3,0);//
	
	
}

else if(p1 && p2 && !p3)//正前方有会影响进桩的障碍物,左没有障碍物 p3 = 0,右有障碍物p2 = 1
{
	 
	//digitalWrite(pin1,0);//
	digitalWrite(pin2,1);//zuo
	//digitalWrite(pin3,0);//
	 
	 
	
}

else if(p1 && !p2 && p3)//正前方有会影响进桩的障碍物,左有障碍物 p3 = 1,右没有障碍物p2 = 0
{
	//digitalWrite(pin1,1);//
	//digitalWrite(pin2,1);//
	digitalWrite(pin3,1);// you
	
		
		
		
	
}
 else if(p1 && p2 && p3)//正前方有会影响进桩的障碍物,左有障碍物 p3 = 1,右有障碍物p2 = 1   !!!!
{

     pan = 1;

			if(  (y_x1 - zhuang_x2) > (zhuang_x1 - z_x)  )//内间距
			{

			spacing_max = ( y_x1 - zhuang_x2 );
			pin_x = pin3;
			pan1 = 1;



			}
			else
			{
				spacing_max = (zhuang_x1 - z_x);
				pin_x = pin2;
				

			}


				
				
			if( ( spacing_max - zhuang_x1 ) > tong_x  )//判断,间距是否大于可通过开口距离tong_x,
			                                           //这里给tong_x作为调节的阈值从而控制

			{
				
						digitalWrite(pin_x,1);// 谁开口大,就去那个口 
				
			}
			
			else
			{
				if( (zhuang_x1 - z_x1) > (y_x - zhuang_x2) )//内间距进不去,外间距小的那边就向那边去
					{
					  digitalWrite(pin3,1);
					}
				else
					{
					  digitalWrite(pin2,1);
					}
				
			}
				

}
else
{
	digitalWrite(pin1,0);
	digitalWrite(pin2,0);
	digitalWrite(pin3,0);

    printf("NO Obstacle,pass!");
}
	 
	 
 }

int main()
{
	
	//矩形坐标分析阵列初始化
	
	for( int ii = 0; ii<= 160;ii+=X)
	{
		a[p] = ii;
		p++;

	}

	for( int jj = 0; jj<=120;jj+=Y)
	{
		b[q] = jj;
		q++;

	}
	

	
	//wiring pi初始化
	int isOK = 0;
	isOK = wiringPiSetup();
	if (isOK == -1) 
	{
	printf("wiringPiSetup设置失败\n");
	}
	else 
	{
	printf("wiringPiSetup设置OK == %d\n",isOK);	
	}
	//以上为wiring pi设置

 // wiring pi pow init
pinMode(pin0, INPUT);
pinMode(pin1,OUTPUT);
pinMode(pin2,OUTPUT);
pinMode(pin3,OUTPUT);
	
//.....................................................................................
    //摄像头初始化
	VideoCapture cap(0);//打开摄像头
	cap.set(CV_CAP_PROP_FPS, 60);//帧数设置
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 180);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
	if ( !cap.isOpened() )

	{

	cout << "打开摄像头失败" << endl;

	return -1;

	}
	
	//视频通道初始化
		Mat frame;//设定原视频通道
	    Mat out;//开闭操作后的输出源
		//Mat gr_out;//开闭操作后的输出源
		//Mat dstframe;
		Mat corframe;
	    Mat greframe;
		
	//	Mat src_gray;
	
	
	
	//以下通过read来给树莓派jie信号,表示进入圆域
	//int read = digitalRead(pin1);//IO口读信号,输入为低,进入大圆域拉高 pin1
	
	//..............................................................................
	   //以下是颜色过滤调试窗口

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"//窗口声明
	//.....................................................................

	cvCreateTrackbar("色调:低", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("色调:高", "Control", &iHighH, 179);

	cvCreateTrackbar("饱和度:低", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("饱和度:高:", "Control", &iHighS, 255);

	cvCreateTrackbar("亮度:低", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("亮度:高", "Control", &iHighV, 255);


	//cvCreateTrackbar("形态学", "Control", &tuo, 50);
	cvCreateTrackbar("红绿过滤程度", "Control", &guo_lv, 255);

    cvCreateTrackbar("前障x1阈值", "Control", &zhuang_x1, 4);
	cvCreateTrackbar("前障x2阈值", "Control", &zhuang_x2, 4);
	cvCreateTrackbar("桩点y1阈值", "Control", &zhuang_y1, 4);
	cvCreateTrackbar("桩点y2阈值", "Control", &zhuang_y2, 4);

	cvCreateTrackbar("桩点y2阈值", "Control", &tong_x, 5);
/* short int zhuang_x1;//前方障碍物范围阈值
short int zhuang_x2;
short int zhuang_y1;
short int zhuang_y2;
 */

 
						
 while( 1)
	{
		Mat frame;//设定原视频通道
	   
		bool bSuccess = cap.read(frame); 
		if (!bSuccess) 
		{ cout << "无法获得视频帧" << endl; } 

	
	
		imshow("读取原视频", frame);//++++++++++++++++++++++++++++++++++++++++1
//考虑到运行速度不调用形态学操作库		
	/* 	
		//获取自定义核 第一个参数MORPH_RECT表示矩形的卷积核，当然还可以选择椭圆形的、交叉型的
		//Mat element = getStructuringElement(MORPH_RECT, Size(tuo, tuo)); //方形
		//Mat element = getStructuringElement(MORPH_CROSS, Size(tuo, tuo)); //cross-shaped（交错形状）
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(tuo, tuo)); //椭圆形结构元素，																   
		morphologyEx(frame, out, MORPH_OPEN, element);//开操作
		//morphologyEx(out, out, MORPH_CLOSE, element);//闭操作
		namedWindow("形态学处理操作", 0);
		cvResizeWindow("形态学处理操作", 200, 200); //创建一个300*300大小的窗口
		imshow("形态学处理操作", out);//+++++++++++++++++++++++++++++++++++++++++++2
		 */
		
		
		
	
		thread x_cor(Pickup_cor,ref(frame),ref(corframe));
		x_cor.join();//颜色判断 红
		
		
		thread x_gr(Pickup_gre,ref(frame),ref(greframe));
		x_gr.join();//颜色判断 绿
		
		
		namedWindow("过滤视频",0);//创建窗口
		cvResizeWindow("过滤视频", 300, 300); //创建一个300*300大小的窗口
		imshow("过滤视频", corframe);//++++++++++++++++++++++++++++++++++++++++2
		
		
		
		namedWindow("绿过滤视频",0);//创建窗口
		cvResizeWindow("绿过滤视频", 300, 300); //创建一个300*300大小的窗口
		imshow("绿过滤视频", greframe);//++++++++++++++++++++++++++++++++++++++++2
	if( digitalRead(pin0 == 1))//进入圆域,获取障碍物最右边初值
	{
	
    chu_z_x = z_x;
	chu_y_x = y_x;
	chu_z_x1 = z_x1;

	}

	if ( digitalRead(pin1) );//IO口读信号,输入为低,进入大圆域拉高 pin1
		{
	       tactics();//根据函数内的全局变量,选择对应的小车应对方法


	    }
	



	if(pan)//进入根据内外间距,调控stm32左右移动的函数,并终止它,根据之前的tactics函数内的判断开启
{

		if(pan1)
			 {
						if( (chu_y_x - y_x) > tong_x)
							{
								digitalWrite(pin1,0);
								digitalWrite(pin2,0);
								digitalWrite(pin3,0);
				
							}



		    }
		else
			{
			           if((z_x1 - chu_z_x1) > tong_x)
			             	{
			             	    digitalWrite(pin1,0);
								digitalWrite(pin2,0);
								digitalWrite(pin3,0);


					        }
			





		    }


		
           

 }
	/* 	cvtColor(out, src_gray, COLOR_RGB2GRAY);//彩色图像转化成灰度图
		namedWindow("只灰视频",0);//创建窗口
		cvResizeWindow("只灰视频", 200, 200); //创建一个300*300大小的窗口
		imshow("只灰视频", src_gray); //+++++++++++++++++++++++++++++++++++++++3
		 */
		
		
		
				
        //圆检测线程的开启与回收
	/* 	thread x_cir(p_circle,ref(corframe),ref(out));
		x_cir.join();
		 */
		
	/* 	if(pin1 == 1)//进入大圆域拉高为真
		{
			
			if( p_colour >100)
			{
				
				digitalWrite(pin2,0);//检测到红色拉低pin2口
				p_colour = 0;
				printf("端口拉高,判断:红色出现");
			
					
			}
		}
 */
/* 
printf("障碍物最左端 %d,%d\n",z_x,z_y);
printf("障碍物最右端 %d,%d\n",y_x,y_y);
printf("障碍物最上端 %d,%d\n",s_x,s_y);
printf("障碍物最下端 %d,%d\n",x_x,x_y);


 */
		
	waitKey(30);
	}
}

