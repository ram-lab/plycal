#include "pointcloudviewer.h"
#include "ui_pointcloudviewer.h"

#include <QMenu>
#include <QDebug>


PointcloudViewer::PointcloudViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PointcloudViewer)
{
    ui->setupUi(this);
    setWindowFlags(Qt::Window
                   | Qt::WindowMinimizeButtonHint
                   | Qt::WindowMaximizeButtonHint);

    viewer_ = std::unique_ptr<pcl::visualization::PCLVisualizer>(
                  new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    viewer_->resetCamera();
    showCoordinateSystem(Eigen::Affine3f::Identity(), 0, 2);
}

PointcloudViewer::~PointcloudViewer()
{
    delete ui;
}


/**
 * @brief show(add/update) pointcloud in the viewer
 * @param [in] pc xyzrgb pointcloud pointer
 * @param [in] index pointcloud id
 */
void PointcloudViewer::showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, int index)
{
    std::string pc_name = std::string("cloud_")+std::to_string(index);
    if(cloud_names_.count(index))
    {
        // update
        viewer_->updatePointCloud(pc, pc_name);
    }
    else
    {
        //add
        cloud_names_.insert(index);
        viewer_->addPointCloud(pc, pc_name);
    }
    ui->qvtkWidget->update();
}


/**
 * @brief show coordinate in the viewer
 * @param [in] tf    the coordinate transformation
 * @param [in] index the coordinate id
 * @param [in] scale the scale, default 1.0
 */
void PointcloudViewer::showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale)
{
    std::string frame_name = std::string("frame_") + std::to_string(index);
    if(frame_names_.count(index))
    {
        viewer_->updateCoordinateSystemPose(frame_name, tf);
    }
    else
    {
        frame_names_.insert(index);
        viewer_->addCoordinateSystem(scale, tf, frame_name);
    }
}


void PointcloudViewer::contextMenuEvent(QContextMenuEvent *event)
{
    qDebug() << "right click";
    QMenu menu(this);
    menu.addAction(ui->actionSave_PointCloud);
    menu.exec(event->globalPos());
}

void PointcloudViewer::on_actionSave_PointCloud_triggered()
{
    qDebug() << "right click";
}
