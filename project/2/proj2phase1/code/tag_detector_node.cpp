#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
//#include <opencv2/core/eigen.hpp>

//using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;
Matrix3d K_eigen;

double x_error=0;
double y_error=0;
double z_error=0;
int cnt = 0;

// test function, can be used to verify your estimation
// This is to verify the correctness of Rotation matrix R and translation vector t
// and calculate the error between real pts_2 and (R*p_mat+t) at given pts_3 location
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);          // 64bits float channel 1 per pixel
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    // note: 
    // 1. this reference assumes pts_2, 2D position in image frame is given (find this thing)
    // 2. it calculates the rotation vector and translation vector, which are exactly the things we need to fill
    // 3. pts_id.size() = pts_3.size() = pts_2.size() = n and this n varies according to each image
    cv::Mat r, rvec, t;
    // bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, 
    //              OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    // ref: http://blog.sina.com.cn/s/blog_5fb3f125010100hp.html
    // Converts a rotation matrix to a rotation vector or vice versa. here r is a rotation matrix
    cv::Rodrigues(rvec, r); 
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {   
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;  // convert matrix3d rotation to Quaternion
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // Assignment requirement
    // 1. Calculating the camera’s pose corresponding to every image.
    // 2. Publishing camera pose information in the form of nav_msgs/Odometry.
    // 3. Plotting these poses with rqt rviz.
    // 4. Comparing your result with the reference

    // Note that the pose you calculated is (cPw, cRw), 
    // which represents the pose of world frame respecting to the camera frame

    // EIgen SVD libnary, may help you solve SVD
    // JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    ROS_INFO("jixin's code here!");

    // Matrix3d x, y;
    // x << 1,2,3,4,5,6,7,8,9;
    // cout << x << endl;
    // cout << x.block<3,1>(0,0) << endl;
    // cout << x.block<3,1>(0,1) << endl;
    // cout << x.block<3,1>(0,2) << endl;
    // cout << "now it is the time for y" << endl;
    // y << x.block<3,1>(0,0), x.block<3,1>(0,1), x.block<3,1>(0,2);
    // cout << y << endl;
    // while(true){}

    // H = K*(R|t), K-1*H = (R|t)
    // 1. get 4 3d postion points
    // 2. solve A*h_hat = 0 using SVD
    // 3. find matrix [h1_hat h2_hat h1_hat*h2_hat]

    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    
    // initialization
    int ptsnum = pts_3.size();
    MatrixXd coef(2*ptsnum, 9);
    Matrix3d h_hat, h_ortho;

    for(int i=0; i<ptsnum; i++){
        coef.block<2,9>(2*i,0) << 
            pts_3[i].x,pts_3[i].y,1,0,0,0,-(pts_3[i].x)*(un_pts_2[i].x),-(pts_3[i].y)*(un_pts_2[i].x),-un_pts_2[i].x,
            0,0,0,pts_3[i].x,pts_3[i].y,1,-(pts_3[i].x)*(un_pts_2[i].y),-(pts_3[i].y)*(un_pts_2[i].y),-un_pts_2[i].y;
    }
    // MatrixXd A(8, 9);
    // A << pts_3[0].x, pts_3[0].y, 1, 0, 0, 0, -pts_3[0].x*un_pts_2[0].x, -pts_3[0].y*un_pts_2[0].x, -un_pts_2[0].x,
    //      0, 0, 0, pts_3[0].x, pts_3[0].y, 1, -pts_3[0].x*un_pts_2[0].y, -pts_3[0].y*un_pts_2[0].y, -un_pts_2[0].y,
    //      pts_3[1].x, pts_3[1].y, 1, 0, 0, 0, -pts_3[1].x*un_pts_2[1].x, -pts_3[1].y*un_pts_2[1].x, -un_pts_2[1].x,
    //      0, 0, 0, pts_3[1].x, pts_3[1].y, 1, -pts_3[1].x*un_pts_2[1].y, -pts_3[1].y*un_pts_2[1].y, -un_pts_2[1].y,
    //      pts_3[2].x, pts_3[2].y, 1, 0, 0, 0, -pts_3[2].x*un_pts_2[2].x, -pts_3[2].y*un_pts_2[2].x, -un_pts_2[2].x,
    //      0, 0, 0, pts_3[2].x, pts_3[2].y, 1, -pts_3[2].x*un_pts_2[2].y, -pts_3[2].y*un_pts_2[2].y, -un_pts_2[2].y,
    //      pts_3[3].x, pts_3[3].y, 1, 0, 0, 0, -pts_3[3].x*un_pts_2[3].x, -pts_3[3].y*un_pts_2[3].x, -un_pts_2[3].x,
    //      0, 0, 0, pts_3[3].x, pts_3[3].y, 1, -pts_3[3].x*un_pts_2[3].y, -pts_3[3].y*un_pts_2[3].y, -un_pts_2[3].y;
    // H_col = A.colPivHouseholderQr().solve(b);       // a column vector

    JacobiSVD<MatrixXd> svda(coef, ComputeThinU | ComputeThinV);
    MatrixXd AV = svda.matrixV();
    MatrixXd AU = svda.matrixU(); 
    VectorXd H_col = AV.rightCols(1);

    Matrix<double,3,3,RowMajor> H_hat(H_col.data());   // convert column to Matrix3d
    // explanation: look at L5 page 45, if translation tz is negative, reverse H (reverse lamda)
    // because we get the same image projection on image plane from above ground and below ground
    // cout << "H_hat is: " << H_hat << endl;
    if(H_hat(2,2) < 0)
        H_hat = -H_hat;
    // after cv::undistortPoints, the camera matrix K will become identity so no need K
    // h_hat = K_eigen.inverse() * H_hat;
    h_ortho << H_hat.col(0), H_hat.col(1), H_hat.col(0).cross(H_hat.col(1));
    JacobiSVD<MatrixXd> svd(h_ortho, ComputeThinU | ComputeThinV);

    R = (svd.matrixU()) * ( svd.matrixV().transpose() );
    T = H_hat.col(2) / ( H_hat.col(0).norm() );

    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);

    // recurrent rms
    // final rms:   0.232831    0.193553    0.585309
    x_error = sqrt( (cnt*pow(x_error,2)+pow(t.at<double>(0, 0) - T(0),2))/(cnt+1) );
    y_error = sqrt( (cnt*pow(y_error,2)+pow(t.at<double>(1, 0) - T(1),2))/(cnt+1) );
    z_error = sqrt( (cnt*pow(z_error,2)+pow(t.at<double>(2, 0) - T(2),2))/(cnt+1) );
    cnt++;
    std::cout << x_error <<endl;
    std::cout << y_error <<endl;
    std::cout << z_error <<endl;
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref      = n.advertise<nav_msgs::Odometry>("odom_ref",10);

    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++){   
            K_eigen(i,j) = K.at<double>(i, j);
        }
    cout << "K is :   " << K << endl;
    cout << "K_eigen is" << K_eigen << endl;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
