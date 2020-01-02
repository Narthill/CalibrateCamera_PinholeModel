#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
using namespace std;
using namespace cv;

// ����ͼ���б�
vector<Mat> imgs;
// ����Ҷ�ͼ�б�
vector<Mat> imgGrays;
// ͼ��ƽ���ά�ǵ�
vector<Point2f> corners;
// ����ͼ��ƽ���ά�ǵ������
vector<vector<Point2f> > image_points;
// ��������ϵ����ά��
vector<Point3f> opt;
// ����������������ϵ����ά������
vector<vector<Point3f> > object_points;
// ��������
void imgshow(string&,Mat&);
Mat fillImage(Mat&,Mat&,int);
void pinholeModel();

// ͼƬ·��
string PATH="./q1/";
// ͼƬ��ʽ
string FORMAT="bmp";
// ͼƬ����
int NUM=6;
//�ǵ��������
int board_w = 14;
int board_h = 9;


// ������ݽṹ��
typedef  struct Err{
	double mean_err;
	double max_err;
};
Err computeReprojectionErrors(const vector<vector<Point3f> >& ,
        const vector<vector<Point2f> >& ,
        const vector<Mat>& , const vector<Mat>& ,
        const Mat& , const Mat& );

/**
** ����ƽ����ͶӰ����������ͶӰ���
**/
Err computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs )
{
	//���ṹ��
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
	//����
	sort(perViewErrors.begin(),perViewErrors.end());
	//�����ͶӰ���
	e.max_err=perViewErrors.back();
	//ƽ��
	e.mean_err=std::sqrt(totalErr/totalPoints);

    return e;
}

/**
** С�׳���ģ���ڲ�����ͶӰ������
**/
void pinholeModel(){
	// ����������󣬻������
	Mat cameraMatrix;
	Mat distCoeffs;
	// ��������������ת��ƽ�ƾ��������
	vector<Mat> rvecs, tvecs;

	//�궨
	calibrateCamera(object_points,image_points,
			imgs[0].size(),
			cameraMatrix,
			distCoeffs,
            rvecs,
			tvecs);
	
	cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
	cout << "�������:[fx, 0, u0; 0, fy, v0; 0,0,1]" <<endl;
	cout << cameraMatrix<< endl;

	cout << distCoeffs.rows << "x" <<distCoeffs.cols << endl;
	cout<<"����ϵ��:k1,k2,p1,p2,k3"<<endl;
	cout << distCoeffs << endl;

	//����ƽ����ͶӰ���
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
			<<"skew"<< ','<<"ƽ����ͶӰ���"<< ','<<"�����ͶӰ���"<<endl;
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
** ��ʾͼ���ͨ�ú���
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
			"     -p <params>         # �������ã�1.�ļ���·����2.ͼƬ������3.ͼ���ʽ��4.�ǵ㳤��5.�ǵ��\n");
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
		imgshow(string("ԭͼ"),img);
	

		//�����ؾ�ȷ��Ҫ�ҶȻ�
		cvtColor(img, imageGray, COLOR_BGR2GRAY);//�ҶȻ�
		imgshow(string("�ҶȻ�ͼ��"),imageGray);

		imgs.push_back(img);
		imgGrays.push_back(imageGray);
		cout<<endl;
		cout<<str.str()<<endl;
	}


	//���̸��ڽǵ�����
	const int board_n = board_w * board_h;
	Size board_sz= Size(board_w,board_h);

	for(unsigned int i = 0; i < imgs.size(); i++){
		
		//��ͼ��ƽ���ά�ǵ�
		bool found = findChessboardCorners(imgs[i], board_sz,corners,CALIB_CB_ADAPTIVE_THRESH |CALIB_CB_NORMALIZE_IMAGE);
		if(found){
			//��ȷ��������
			cornerSubPix( imgGrays[i], corners, Size(3,3),Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			//��������ͼ�Ľǵ�
			drawChessboardCorners(imgs[i],board_sz,corners, found);
			//���ҵ��Ķ�ά�ǵ����������
			image_points.push_back(corners);
			//��ʾͼ��
			stringstream str;
			str<<i;
			imgshow(str.str(),imgs[i]);

			//����ͼƬ����ά�ǵ������¼
			opt.resize(board_n);
			for (int j = 0; j < board_n; j++)
			{
				opt[j]=(Point3f((float)(j / board_w), (float)(j % board_w), 0.0f));
			}
			//����ά�ǵ����ݷ�������
			object_points.push_back(opt);
		}
	}
	//����С�׳���
	pinholeModel();
	waitKey(30000);
    return 0;
}