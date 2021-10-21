#include "utils/visualizer.h"

void Visualizer::showWorldFrame(const cv::String& id, float scale)
{
    showWidget(id, cv::viz::WCoordinateSystem(scale));
}

void Visualizer::showPoints(const std::string& id, 
                            const std::vector<cv::Point3f>& points,
                            float point_size, 
                            const cv::Scalar& color)
{
    cv::viz::WCloud cloud_widget = cv::viz::WCloud(points);
    cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, point_size);
    cloud_widget.setColor(color);
    showWidget(id, cloud_widget);
}

void Visualizer::showFrame(const std::string& id, const cv::Affine3f& pose, float scale)
{   
    showWidget(id, cv::viz::WCoordinateSystem(scale), pose);
}
