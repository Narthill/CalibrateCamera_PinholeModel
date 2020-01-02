#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
using namespace std;
using namespace cv;

// 保存图像列表
vector<Mat> imgs;
// 保存灰度图列表
vector<Mat> imgGrays;
// 图像平面二维角点
vector<Point2f> corners;
// 保存图像平面二维角点的容器
vector<vector<Point2f> > image_points;
// 世界坐标系的三维点
vector<Point3f> opt;
// 保存所有世界坐标系的三维点容器
vector<vector<Point3f> > object_points;
// 函数声明
void imgshow(string&,Mat&);
Mat fillImage(Mat&,Mat&,int);
void pinholeModel();

// 图片路径
string PATH="./q1/";
// 图片格式
string FORMAT="bmp";
// 图片数量
int NUM=6;
//角点横纵数量
int board_w = 14;
int board_h = 9;


// 误差数据结构体
typedef  struct Err{
	double mean_err;
	double max_err;
};
Err computeReprojectionErrors(const vector<vector<Point3f> >& ,
        const vector<vector<Point2f> >& ,
        const vector<Mat>& , const vector<Mat>& ,
        const Mat& , const Mat& );

/**
** 计算平均重投影误差与最大重投影误差
**/
Err computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs )
{
	//误差结构体
	Err e;
	vector<float> perViewErrors;

    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }
	for(unsigned int j=0;j<perViewErrors.size();j++){
		cout<<"perViewErrors:"<<perViewErrors[j]<<endl;
	}
	//排序
	sort(perViewErrors.begin(),perViewErrors.end());
	//最大重投影误差
	e.max_err=perViewErrors.back();
	//平均
	e.mean_err=std::sqrt(totalErr/totalPoints);

    return e;
}

/**
** 小孔成像模型内参与重投影误差计算
**/
void pinholeModel(){
	// 定义相机矩阵，畸变矩阵
	Mat cameraMatrix;
	Mat distCoeffs;
	// 定义用来保存旋转和平移矩阵的容器
	vector<Mat> rvecs, tvecs;

	//标定
	calibrateCamera(object_points,image_points,
			imgs[0].size(),
			cameraMatrix,
			distCoeffs,
            rvecs,
			tvecs);
	
	cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
	cout << "相机矩阵:[fx, 0, u0; 0, fy, v0; 0,0,1]" <<endl;
	cout << cameraMatrix<< endl;

	cout << distCoeffs.rows << "x" <<distCoeffs.cols << endl;
	cout<<"畸变系数:k1,k2,p1,p2,k3"<<endl;
	cout << distCoeffs << endl;

	//计算平均重投影误差
	Err tmpE=computeReprojectionErrors(object_points,image_points,rvecs,tvecs,cameraMatrix,distCoeffs);
	double mean_error=tmpE.mean_err;
	double max_error=tmpE.max_err;

	ifstream inFile("pinholeModelParams.csv", ios::in);
	ofstream outFile;
	if(!inFile){
		outFile.open("pinholeModelParams.csv", ios::out);
		outFile<<"Module ID"<< ',' <<"Cali Mode"<< ','
			<<"fx"<< ','<<"fy"<< ','<<"u0"<< ','<<"v0"<< ','<<"k1"<< ','
			<<"k2"<< ','<<"k3"<< ','<<"p1"<< ','<<"p2"<< ','
			<<"skew"<< ','<<"平均重投影误差"<< ','<<"最大重投影误差"<<endl;
	}else{
		outFile.open("pinholeModelParams.csv", ios::app);
		outFile<<"\n";
	}
	outFile<<"Module ID"<< ',' <<0<< ','
		<<cameraMatrix.at<double>(0,0)<< ','<<cameraMatrix.at<double>(1,1)<< ','<<cameraMatrix.at<double>(0,2)<< ','<<cameraMatrix.at<double>(1,2)<< ','
		<<distCoeffs.at<double>(0,0)<< ','<<distCoeffs.at<double>(0,1) << ','<<distCoeffs.at<double>(0,4) << ','<<distCoeffs.at<double>(0,2)<< ','<<distCoeffs.at<double>(0,3)<< ','
		<<0<< ','<<mean_error<< ','<<max_error;

	outFile.close();
}

/**
** 显示图像的通用函数
**/
void imgshow(string& title,Mat& srcImg){
	namedWindow(title, CV_WINDOW_NORMAL);
	resizeWindow(title, 640, 480);
	imshow(title,srcImg);
	//title.append(".jpg");
	//imwrite(title,srcImg);
}

int main(int argc,char *argv[]){

	
	imgs.clear();
	imgGrays.clear();
	corners.clear();
	image_points.clear();
	opt.clear();
	object_points.clear();
	/*
	if( argc < 2 )
    {
        printf( "This is a camera calibration program.\n"
		"Usage: calibration\n"
			"     -p <params>         # 参数设置：1.文件夹路径，2.图片数量，3.图像格式，4.角点长，5.角点宽\n");
		return 0;
    }
	
	if(!strcmp(argv[1],"-p")){
		PATH=argv[2];
		NUM=atoi(argv[3]);
		FORMAT=argv[4];
		board_w=atoi(argv[5]);
		board_h=atoi(argv[6]);
	}else{
		return 0;
	}
	*/
	for(int imgNum=1;imgNum<=NUM;imgNum++){
		stringstream str;
		str << PATH << imgNum << "."<<FORMAT;
		Mat imageGray;
		Mat img = imread(str.str());
		imgshow(string("原图"),img);
	

		//亚像素精确需要灰度化
		cvtColor(img, imageGray, COLOR_BGR2GRAY);//灰度化
		imgshow(string("灰度化图像"),imageGray);

		imgs.push_back(img);
		imgGrays.push_back(imageGray);
		cout<<endl;
		cout<<str.str()<<endl;
	}


	//棋盘格内角点总数
	const int board_n = board_w * board_h;
	Size board_sz= Size(board_w,board_h);

	for(unsigned int i = 0; i < imgs.size(); i++){
		
		//找图像平面二维角点
		bool found = findChessboardCorners(imgs[i], board_sz,corners,CALIB_CB_ADAPTIVE_THRESH |CALIB_CB_NORMALIZE_IMAGE);
		if(found){
			//精确到亚像素
			cornerSubPix( imgGrays[i], corners, Size(3,3),Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			//画出单张图的角点
			drawChessboardCorners(imgs[i],board_sz,corners, found);
			//将找到的二维角点放入容器中
			image_points.push_back(corners);
			//显示图像
			stringstream str;
			str<<i;
			imgshow(str.str(),imgs[i]);

			//单张图片的三维角点坐标记录
			opt.resize(board_n);
			for (int j = 0; j < board_n; j++)
			{
				opt[j]=(Point3f((float)(j / board_w), (float)(j % board_w), 0.0f));
			}
			//将三维角点数据放入容器
			object_points.push_back(opt);
		}
	}
	//调用小孔成像
	pinholeModel();
	waitKey(30000);
    return 0;
}