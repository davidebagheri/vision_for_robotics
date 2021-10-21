#include <opencv2/core/core.hpp>
#include <opencv2/viz/vizcore.hpp>

class Visualizer : public cv::viz::Viz3d {
public:
    Visualizer(const cv::String& window_name = cv::String(), 
    bool show_world_frame=true, 
    float scale=0.3,
    bool show_grid=true) : cv::viz::Viz3d(window_name)
    {
        if (show_world_frame) showWorldFrame("world", scale);
        if (show_grid) showWidget("grid", cv::viz::WGrid());
    }

    void showWorldFrame(const cv::String& id = "world", float scale=1.0);

    void showPoints(const std::string& id, 
                    const std::vector<cv::Point3f>& points, 
                    float point_size=10,
                    const cv::Scalar& color=cv::Scalar(0, 0, 255));

    void showFrame(const std::string& id, const cv::Affine3f& pose, float scale=1.0);
};
