#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

#include <QMainWindow>
#include <QContextMenuEvent>

#include <set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

namespace Ui {
class PointcloudViewer;
}

class PointcloudViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PointcloudViewer(QWidget *parent = 0);
    ~PointcloudViewer();

public slots:
    void showPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc, int index = 0);
    void showCoordinateSystem(const Eigen::Affine3f& tf, int index, double scale=1.0);

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void on_actionSave_PointCloud_triggered();

private:
    Ui::PointcloudViewer *ui;
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer_;
    std::set<int> cloud_names_;
    std::set<int> frame_names_;
};

#endif // POINTCLOUDVIEWER_H
