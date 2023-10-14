
#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
/*
#include </home/l/app/RM_zi(4)/RM_zi/devices/camera/MVSDK/CameraApi.h>
#include </home/l/app/RM_zi(4)/RM_zi/devices/camera/MVSDK/CameraDefine.h>
#include </home/l/app/RM_zi(4)/RM_zi/devices/camera/MVSDK/CameraStatus.h>

#include </home/l/app/RM_zi(4)/RM_zi/devices/camera/mv_camera.hpp>
#include </home/l/app/RM_zi(4)/RM_zi/devices/camera/mv_camera.cpp>
*/

#include </home/l/code/Zi_Miao/devices/camera/mv_camera.hpp>
#include </home/l//code/Zi_Miao/devices/camera/mv_camera.cpp>

using namespace std;
using namespace cv;
using namespace Devices;

Eigen::Matrix<double, 3, 3> R;//观测协方差
 Eigen::Matrix<double, 6, 3> K;//卡尔曼增益
 //Eigen::Matrix<double, 3, 6>;
 Eigen::Matrix<double, 6, 6> P;//预测协方差
 Eigen::Matrix<double, 6, 6> F;
 Eigen::Matrix<double, 6, 6> I;//单位矩阵
 Eigen::Matrix<double, 6, 6> Q;//噪声
 Eigen::Matrix<double, 3, 1> Zp;//观测
 Eigen::Matrix<double, 3, 6> H;//观测的那个
 Eigen::Matrix<double, 6, 1> X,Xe;
 Eigen::Matrix<double, 3, 1> X0;

Mat tempimg, dstimg;

double chang = 0.142/2;
double kuan = 0.059/2;
    cv::Point3f wtop1(-chang,kuan,0);//左下
    cv::Point3f wtop2(chang,kuan,0);//右下
    cv::Point3f wbottom1(-chang,-kuan,0);//左上
    cv::Point3f wbottom2(chang,-kuan,0);//右上
vector<cv::Point3f> big_obj(4);
 vector<Point2f> points_2d(4);
   Eigen::Matrix3d R_C2G;

int main() 
{


  auto last_time = std::chrono::system_clock::now();
    auto new_time = std::chrono::system_clock::now();
  
  cv::Mat dian_3d = (cv::Mat_<double>(3,1)<<0,0,0);
   cv::Mat dian_3dd = (cv::Mat_<double>(3,1)<<0,0,0);

  Mat img, gray, dstimg;
  // system("sudo -S chmod 777 /dev/vedio1");

    double timestamp_ms;  //曝光时间, 单位us

    Devices::MV_Camera a;
    bool flags_2=a.open();
   

    X0 << 0,
       0,
       0;

    P.setIdentity();

    Xe<< 0,
      0,
      0,
      0,
      0,
      0;

      X<< 0,
      0,
      0,
      0,
      0,
      0;

    I.setIdentity();

    cv::Mat X3(3,1,CV_64F);

    while (1) {
      bool flag_2 = a.read(img);
  cv::cvtColor(img,  gray, COLOR_BGR2GRAY);
  cv::imshow("gray", gray);
  
  cv::inRange(gray, cv::Scalar(160), cv::Scalar(255), dstimg);

  cv::imshow("binary",dstimg);
    std::vector<std::vector<cv::Point>> contours;
  vector<Vec4i> ContourTree;
    cv::findContours(dstimg, contours, ContourTree, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    cv::Mat dian3 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    Mat dian2;
    Mat dian22;

  if(contours.size() > 1)
  {

    cv::RotatedRect target_rect;
    cv::Point2f pts[4];
    cv::Point2f pts1[4];
    cv::Point2f pts2[4];
    cv::Point2f top1;
    cv::Point2f bottom1;
    cv::Point2f top2;
    cv::Point2f bottom2;
    cv::RotatedRect rect1 = cv::minAreaRect(contours[0]); 
    rect1.points(pts1);
    std::sort(pts1,pts1 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});
    top1 = (pts1[0] + pts1[1]) / 2;
    bottom1 = (pts1[2] + pts1[3]) / 2;
    cv::RotatedRect rect2 = cv::minAreaRect(contours[1]);
    rect2.points(pts2);
    std::sort(pts2,pts2 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});
    top2 = (pts2[0] + pts2[1]) / 2;
    bottom2 = (pts2[2] + pts2[3]) / 2;

    cv::line(img,top1,bottom2,cv::Scalar(0,255,0),2,8,0);
    cv::line(img,top2,bottom1,cv::Scalar(0,255,0),2,8,0);

    if(top1.x>top2.x)//默认左边等条是top1
    {
      cv::Point2f top3;
      top3 = top1;
      top1 = top2;
      top2 = top3;
      cv::Point2f bottom3;
      bottom3 = bottom1;
      bottom1 = bottom2;
      bottom2 = bottom3;
    }

    points_2d[0]=bottom1;//左下
    points_2d[1]=top1;//左上
    points_2d[2]=top2;//右上
    points_2d[3]=bottom2;//右下

    big_obj[0]=wbottom1;
    big_obj[1]=wtop1;
    big_obj[2]=wtop2;
    big_obj[3]=wbottom2;
/*
    double k1 = (top1.y - bottom2.y)/(top1.x - bottom2.x);
    double b1 = top1.y - k1*top1.x;
    double k2 = (bottom1.y - top2.y)/(bottom1.x - top2.x);
    double b2 = bottom1.y - k2*bottom1.x;

    Point2f centre;
    centre.x = (b2 - b1)/(k1 - k2);
    centre.y = k1*centre.x + b1;
    //cv::circle(img,centre,10,cv::Scalar(255,0,0),-1);
    cout<<"centre.x:   "<<centre.x;
    cout<<"\ncentre.y:   "<<centre.y;
*/

  // 相机内参矩阵
        Mat camera_matrix = (Mat_<double>(3, 3) << 1562.718391961961,    4.478471970184, 616.284509563135,
                                                             0, 1563.803986201450, 489.040600564709,
                                                             0,                 0,                1 );
        // 相机畸变系数
        Mat dist_coeffs = (Mat_<double>(5, 1) <<  -0.088250992965441, 0.154271559512706, -0.000829270875611, -0.001394637877056, 0 );

    // 旋转向量
        Mat rotation_vector;
        // 平移向量
        Mat translation_vector;

        // pnp求解
        solvePnP(big_obj, points_2d, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false,cv::SOLVEPNP_ITERATIVE);

  Mat Rvec;
	Mat_<float> Tvec;
	rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
	translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

	Mat_<float> rotMat(3, 3);
	Rodrigues(Rvec, rotMat);
	// 旋转向量转成旋转矩阵


	Mat P_oc;
	P_oc = -rotMat.inv() * Tvec;

//时间戳
  auto end_time = std::chrono::system_clock::now();
        
  new_time = end_time;
  auto waste_time = std::chrono::duration_cast<std::chrono::microseconds>(new_time-last_time).count();
  last_time = end_time;
  double t = waste_time * 0.000001;

 X(1,0)=(translation_vector.at<double>(0,0)-X0(0,0))/t;
 X(3,0)=(translation_vector.at<double>(1,0)-X0(1,0))/t;
 X(5,0)=(translation_vector.at<double>(2,0)-X0(2,0))/t;
 X(0,0)=X(0,0)+X(1,0)*0.1;
 X(2,0)=X(2,0)+X(3,0)*0.1;
 X(4,0)=X(4,0)+X(5,0)*0.1;

cv::Mat XX(3, 1, 6);
Mat trans_1111;
Mat trans_2222;
XX.at<double>(0,0) = X(0,0);
XX.at<double>(1,0) = X(2,0);
XX.at<double>(2,0) = X(4,0);
cv::projectPoints(dian_3dd, rotation_vector, translation_vector,camera_matrix, dist_coeffs, trans_1111);
//cv::projectPoints(dian_3dd, rotation_vector, XX,camera_matrix, dist_coeffs, trans_2222);

Point2f center_point_1111;
center_point_1111.x=trans_1111.at<double>(0,0);
center_point_1111.y=trans_1111.at<double>(0,1);

//Point2f center_point_2222;
//center_point_2222.x=trans_2222.at<double>(0,0);
//center_point_2222.y=trans_2222.at<double>(0,1);

cv::circle(img,center_point_1111,10,cv::Scalar(255,0,0),-1);
//cv::circle(img,center_point_2222,5,cv::Scalar(0,0,255),-1);


F <<    1,  0.1,  0,  0,  0,  0,
        0,  1,  0,  0,  0,  0,
        0,  0,  1,  0.1,  0,  0,
        0,  0,  0,  1,  0,  0,
        0,  0,  0,  0,  1,  0.1,
        0,  0,  0,  0,  0,  1;

 Q <<    pow(t,4) / 4,  pow(t,3) / 2,  0,  0,  0,  0,
         pow(t,3) / 2,  pow(t,2),   0,   0,  0, 0,
         0, 0,  pow(t,4) / 4,  pow(t,3) / 2,  0,  0,
         0, 0,  pow(t,3) / 2,  pow(t,2),  0,   0,
         0, 0,  0,  0,  pow(t,4) / 4,  pow(t,3) / 2,
         0, 0,  0,  0,  pow(t,3) / 2,  pow(t,2); 
  Q=Q*0.01; 

 P = F * P * F.transpose()+Q;

H <<    1,  0,  0,  0,  0,  0,
        0,  0,  1,  0,  0,  0,
        0,  0,  0,  0,  1,  0;
                
 Zp << translation_vector.at<double>(0,0),translation_vector.at<double>(1,0),translation_vector.at<double>(2,0);  

 R << 0,0,0,
      0,0,0,
      0,0,0;

 K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

 Xe = Xe + K * (Zp - H * Xe);
 
 P = (I - K * H) * P;

 X0(0,0)=translation_vector.at<double>(0,0);
 X0(1,0)=translation_vector.at<double>(1,0);
 X0(2,0)=translation_vector.at<double>(2,0);


 Mat trans;//translation
 Mat trans_1;

 Mat trans_3D = (cv::Mat_<double>(3,1)<<Xe(0,0),Xe(1,0),Xe(2,0));
 cv::projectPoints(trans_3D, rotation_vector, translation_vector,camera_matrix, dist_coeffs, trans);
 
 X3.at<double>(0,0) = Xe(0,0);
 X3.at<double>(1,0) = Xe(2,0);
 X3.at<double>(2,0) = Xe(4,0);
 

 }
    cv::imshow("img", img);
    waitKey(2);
    }
        return 0;
}