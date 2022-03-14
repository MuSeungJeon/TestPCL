#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace pcl;
using namespace std;
using namespace cv;

struct color_point_t
{
    int16_t xyz[3];
    uint8_t bgr[3];
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_color_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_color_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr ptr_normal (new pcl::PointCloud<pcl::Normal>);

    k4a_device_t azureHandle = NULL;
    k4a_transformation_t transformation_handle;
    k4a_capture_t capture_handle = NULL;
    k4a_device_configuration_t AzureConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t image_color = NULL;
    k4a_image_t azDepthImage = NULL;
    k4a_image_t azDepthTransImage = NULL;
    k4a_image_t azPtCloud = NULL;
    k4a_calibration_t sensor_calibration;

    uint32_t count = k4a_device_get_installed_count();
    if(count==0){
        qDebug()<<"Auzre device not connected"<<endl; return;
    }
    Mat pLocImage;

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &azureHandle))
    {
        qDebug()<<"Failed to start cameras!\n";
        k4a_device_close(azureHandle); return;
    }
    AzureConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    AzureConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    AzureConfig.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    AzureConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
    AzureConfig.synchronized_images_only    = true;

    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(azureHandle, AzureConfig.depth_mode,
                                                           AzureConfig.color_resolution, &sensor_calibration))
    {
        qDebug()<<"Get depth camera calibration failed!\n"; return;
    }
    transformation_handle = k4a_transformation_create(&sensor_calibration);
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(azureHandle, &AzureConfig))
    {
        qDebug()<<"Failed to start device!\n"; return;
    }

    k4a_device_get_capture(azureHandle, &capture_handle, 5000);
    image_color = k4a_capture_get_color_image(capture_handle);
    azDepthImage = k4a_capture_get_depth_image(capture_handle);
    colorWidth = k4a_image_get_width_pixels(image_color);
    colorHeight = k4a_image_get_height_pixels(image_color);
    depthWidth = k4a_image_get_width_pixels(azDepthImage);
    depthHeight = k4a_image_get_height_pixels(azDepthImage);
    pDepthBuffer    = (unsigned short* )new unsigned short[depthWidth * depthHeight];

    image_ptr_color = k4a_image_get_buffer(image_color);
    image_ptr_depth = k4a_image_get_buffer(azDepthImage);

    pLocImage = Mat(colorHeight,colorWidth,CV_8UC4,(void *)image_ptr_color,cv::Mat::AUTO_STEP);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,colorHeight,
                                                 colorWidth*(int)sizeof(uint16_t),&azDepthTransImage ))
    {
        qDebug()<<"azDepthTransImage could not be written!\n"; return;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,colorHeight,
                                                 colorWidth*3*(int)sizeof(int16_t),&azPtCloud ))
    {
        qDebug()<<"azPtCloud could not be created!\n"; return;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation_handle,
                                                                               azDepthImage, azDepthTransImage))
    {
        qDebug()<<"transformation from depth to color image could n't be done. !\n"; return;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              azDepthTransImage,K4A_CALIBRATION_TYPE_COLOR, azPtCloud))
    {
        qDebug()<<"transformation from depth to point cloud could n't be done. !\n"; return;
    }

    int16_t * azPtCloudp      = (int16_t *)(void *)k4a_image_get_buffer(azPtCloud);

    PointXYZ p = PointXYZ();
    PointXYZRGB c_p = PointXYZRGB();
    for (int i=0; i<colorWidth*colorHeight; i++ )
    {
        p.x = c_p.x = azPtCloudp[3*i + 0];
        p.y = c_p.y = azPtCloudp[3*i + 1];
        p.z = c_p.z = azPtCloudp[3*i + 2];
        c_p.r = image_ptr_color[4*i + 0];
        c_p.g = image_ptr_color[4*i + 1];
        c_p.b = image_ptr_color[4*i + 2];
        cloud.push_back(p);
        color_cloud.push_back(c_p);
    }

//    if (GetPointCloud())
//    {
//        PointXYZ p = PointXYZ();
//        PointXYZRGB c_p = PointXYZRGB();
//        for (int i=0; i<depthWidth*depthHeight; i++ )
//        {
//            p.x = c_p.x = azPtCloudp[3*i + 0];
//            p.y = c_p.y = azPtCloudp[3*i + 1];
//            p.z = c_p.z = azPtCloudp[3*i + 2];
//            c_p.r = image_ptr_color[4*i + 0];
//            c_p.g = image_ptr_color[4*i + 1];
//            c_p.b = image_ptr_color[4*i + 2];
//            cloud.push_back(p);
//            color_cloud.push_back(c_p);
//        }
//    }
//    else
//    {
//        PointXYZ p = PointXYZ();
//        PointXYZRGB c_p = PointXYZRGB();
//        QString data_path = "C:\\Rover_G_UPI_Data\\ImageTest\\";
//        QString txt_file = data_path + "WingSkinPC.txt";
//        QFile file(txt_file);
//        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
//            return;
//        int i = 1;
//        while(!file.atEnd()) {
//            QByteArray line = file.readLine();
//            QList<QByteArray> lineList = line.split(' ');
//            if (!(lineList.size()==6))
//                break;
//            p.x = c_p.x = lineList[0].toInt();
//            p.y = c_p.y = lineList[1].toInt();
//            p.z = c_p.z = lineList[2].toInt();
//            c_p.r = lineList[3].toInt();
//            c_p.g = lineList[4].toInt();
//            c_p.b = lineList[5].toInt();
//            cloud.push_back(p);
//            color_cloud.push_back(c_p);
//        }
//    }


    *ptr_cloud = cloud;
    *ptr_color_cloud = color_cloud;

    ptr_cloud->width = static_cast<uint32_t> (colorWidth);
    ptr_cloud->height = static_cast<uint32_t> (colorHeight);
    ptr_cloud->points.resize(ptr_cloud->width*ptr_cloud->height);
    ptr_cloud->is_dense = false;

    normalAngle.create(ptr_cloud->height,ptr_cloud->width, CV_32FC3);
    normalAngle = Scalar::all(0);
    phi_mat.create(normalAngle.size(),CV_16SC1);
    phi_mat = Scalar::all(0);
    theta_mat.create(normalAngle.size(),CV_16SC1);
    theta_mat = Scalar::all(0);


    /* Filtering Part */
    /*
    StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(ptr_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*ptr_cloud_filtered);
    cloud_filtered = *ptr_cloud_filtered;

    StatisticalOutlierRemoval<pcl::PointXYZRGB> color_sor;
    color_sor.setInputCloud(ptr_color_cloud);
    color_sor.setInputCloud (ptr_color_cloud);
    color_sor.setMeanK (50);
    color_sor.setStddevMulThresh (1.0);
    color_sor.filter(*ptr_color_cloud_filtered);
    color_cloud_filtered = *ptr_color_cloud_filtered;
    */

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(ptr_cloud);
    ne.compute(*ptr_normal);
    normal = *ptr_normal;
    normalsByPcl_mat.create(normalAngle.size(), CV_32FC3);

    for (int y=0;y<ptr_cloud->height;y++)
    {
        for (int x=0;x<ptr_cloud->width;x++)
        {
            float nx, ny, nz, phi, theta, r = 0;
            if(!(isnan(ptr_normal->at(x,y).normal_x)))
            {
                Vec3f nVector(ptr_normal->at(x,y).normal_x, ptr_normal->at(x,y).normal_y, ptr_normal->at(x,y).normal_z);
                normalsByPcl_mat.at<Vec3f>(y, x) = -1*nVector;
                nx = ptr_normal->at(x,y).normal_x;
                ny = ptr_normal->at(x,y).normal_y;
                nz = ptr_normal->at(x,y).normal_z;
                r = sqrt(nx*nx+ny*ny+nz*nz);
                phi     = atan(ny/nz)*((float)180/M_PI);
                theta   = asin(nx/r)*((float)180/M_PI);

                theta *= -1;
                phi *= -1;

            }
            Vec3f spherical(r,phi,theta);
            normalAngle.at<Vec3f>(y,x) = spherical;
            phi_mat.at<short>(y,x) = phi;
            theta_mat.at<short>(y,x) = theta;
        }
    }
    int phi_max = 20;
    int theta_max = 20;
    Mat phiPosMat, thetaPosMat;
    phiPosMat = phi_mat+phi_max;
    thetaPosMat = theta_mat+theta_max;
    phiPosMat.convertTo(phiPosMat,CV_8U,(float)255/(float)(2*phi_max),0);
    thetaPosMat.convertTo(thetaPosMat,CV_8U,(float)255/(float)(2*theta_max),0);
    applyColorMap(phiPosMat,phiPosMat,COLORMAP_JET);
    applyColorMap(thetaPosMat,thetaPosMat,COLORMAP_JET);
    namedWindow("Phi", WINDOW_AUTOSIZE);
    imshow("Phi", phiPosMat);


    namedWindow("Theta", WINDOW_AUTOSIZE);
    imshow("Theta", thetaPosMat);

    k4a_device_stop_cameras(azureHandle);
    k4a_image_release(image_color);
    k4a_image_release(azDepthImage);
    k4a_image_release(azDepthTransImage);
    k4a_image_release(azPtCloud);
    k4a_capture_release(capture_handle);
    k4a_transformation_destroy(transformation_handle);
    k4a_device_close(azureHandle);



}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButtonPointCloud_clicked()
{
    pcl::visualization::CloudViewer viewer("Point Cloud");
    viewer.showCloud(cloud.makeShared());
    while (!viewer.wasStopped());
}


void MainWindow::on_pushButtonColorPointCloud_clicked()
{
    pcl::visualization::CloudViewer viewer("Color Point Cloud");
    viewer.showCloud(color_cloud.makeShared());
    while (!viewer.wasStopped());
}


void MainWindow::on_pushButtonFilteredPointCloud_clicked()
{
    pcl::visualization::CloudViewer viewer("Point Cloud Filtered");
    viewer.showCloud(cloud_filtered.makeShared());
    while (!viewer.wasStopped());
}


void MainWindow::on_pushButtonFilteredColorPointCloud_clicked()
{
    pcl::visualization::CloudViewer viewer("Color Point Cloud Filtered");
    viewer.showCloud(color_cloud_filtered.makeShared());
    while (!viewer.wasStopped());
}


bool MainWindow::GetPointCloud()
{
    k4a_device_t azureHandle = NULL;
    k4a_transformation_t transformation_handle;
    k4a_capture_t capture_handle = NULL;
    k4a_device_configuration_t AzureConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_image_t image_color = NULL;
    k4a_image_t azDepthImage = NULL;
    k4a_image_t azDepthTransImage = NULL;
    k4a_image_t azPtCloud = NULL;
    k4a_calibration_t sensor_calibration;

    uint32_t count = k4a_device_get_installed_count();
    if(count==0){
        qDebug()<<"Auzre device not connected"<<endl;
        return false;
    }
    Mat pLocImage;

    if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &azureHandle))
    {
        qDebug()<<"Failed to start cameras!\n";
        k4a_device_close(azureHandle);
        return false;
    }
    AzureConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    AzureConfig.color_resolution = K4A_COLOR_RESOLUTION_720P;
    AzureConfig.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
    AzureConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
    AzureConfig.synchronized_images_only    = true;

    if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(azureHandle, AzureConfig.depth_mode,
                                                           AzureConfig.color_resolution, &sensor_calibration))
    {
        qDebug()<<"Get depth camera calibration failed!\n";
        return false;
    }
    transformation_handle = k4a_transformation_create(&sensor_calibration);
    if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(azureHandle, &AzureConfig))
    {
        qDebug()<<"Failed to start device!\n";
        return false;
    }

    k4a_device_get_capture(azureHandle, &capture_handle, 5000);
    image_color = k4a_capture_get_color_image(capture_handle);
    azDepthImage = k4a_capture_get_depth_image(capture_handle);
    colorWidth = k4a_image_get_width_pixels(image_color);
    colorHeight = k4a_image_get_height_pixels(image_color);
    depthWidth = k4a_image_get_width_pixels(azDepthImage);
    depthHeight = k4a_image_get_height_pixels(azDepthImage);
    pDepthBuffer    = (unsigned short* )new unsigned short[depthWidth * depthHeight];

    image_ptr_color = k4a_image_get_buffer(image_color);
    image_ptr_depth = k4a_image_get_buffer(azDepthImage);

    pLocImage = Mat(colorHeight,colorWidth,CV_8UC4,(void *)image_ptr_color,cv::Mat::AUTO_STEP);

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,colorHeight,
                                                 colorWidth*(int)sizeof(uint16_t),&azDepthTransImage ))
    {
        qDebug()<<"azDepthTransImage could not be written!\n";
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,colorHeight,
                                                 colorWidth*3*(int)sizeof(int16_t),&azPtCloud ))
    {
        qDebug()<<"azPtCloud could not be created!\n";
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(transformation_handle,
                                                                               azDepthImage, azDepthTransImage))
    {
        qDebug()<<"transformation from depth to color image could n't be done. !\n";
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                              azDepthTransImage,K4A_CALIBRATION_TYPE_COLOR, azPtCloud))
    {
        qDebug()<<"transformation from depth to point cloud could n't be done. !\n";
        return false;
    }
    int16_t * azPtCloudp      = (int16_t *)(void *)k4a_image_get_buffer(azPtCloud);

    k4a_device_stop_cameras(azureHandle);
    k4a_image_release(image_color);
    k4a_image_release(azDepthImage);
    k4a_image_release(azDepthTransImage);
    k4a_image_release(azPtCloud);
    k4a_capture_release(capture_handle);
    k4a_transformation_destroy(transformation_handle);
    k4a_device_close(azureHandle);

    return true;
}
