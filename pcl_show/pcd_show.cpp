#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../thing_three.pcd", *cloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud"); // 添加点云到可视化对象
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点的大小
    viewer->addCoordinateSystem(1.0); //视图器中添加一个坐标系统，坐标轴长度为 1.0


    while (!viewer->wasStopped()) {
        viewer->spinOnce(10000); // 处理事件并更新视图
    }

    return 0;
}