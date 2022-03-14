#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>

#include "k4a/k4a.h"
#include "k4a.hpp"

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>
#include "pcl/point_cloud.h"
#include "pcl/visualization/cloud_viewer.h"

#include <QString>
#include <QFile>
#include <iostream>
#include <fstream>
#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QLabel>
#include <QToolTip>
#include <QTimer>
#include <QVector>
#include <cmath>

using namespace pcl;
using namespace std;
using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filtered;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_color_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_color_cloud_filtered;
    pcl::PointCloud<pcl::Normal>::Ptr ptr_normal;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud_filtered;
    pcl::PointCloud<pcl::Normal> normal;
    Mat normalAngle, normalsByPcl_mat, phi_mat, theta_mat;

    int colorHeight,colorWidth,depthHeight,depthWidth;

    unsigned short* pDepthBuffer;
    unsigned short* pDepthBufferAvg;
    unsigned short* pDepthBufferSum;
    uint8_t * image_ptr_color;
    uint8_t * image_ptr_depth;
    Mat depImgSumMat,depImgAvgMat,depImgAvgDisMat;


    bool GetPointCloud();

private slots:
    void on_pushButtonPointCloud_clicked();

    void on_pushButtonColorPointCloud_clicked();

    void on_pushButtonFilteredPointCloud_clicked();

    void on_pushButtonFilteredColorPointCloud_clicked();

private:
    Ui::MainWindow *ui;
};


#endif // MAINWINDOW_H
