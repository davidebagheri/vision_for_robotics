#include "utils/visualizer.h"


int main(){
    Visualizer window("test");
    window.showWorldFrame();

    std::vector<cv::Point3f> points = {cv::Point3f(0.5, 0.2, 0.1),
                                        cv::Point3f(0.6, 0.2, 1.2),
                                        cv::Point3f(0.1, 0.1, 2.54),
                                        cv::Point3f(1.2, 0.32, 0.6),
                                        cv::Point3f(0.24, 1.2, 1.1),
                                        cv::Point3f(1.5, 2.2, 0.2),
                                        cv::Point3f(2.5, 1.2, 1.9)
                                    };
window.showPoints("p", points, 10);
    while (!window.wasStopped())
    {
        //window.showWorldFrame();
        window.spinOnce(1, true);
        

    }

    return 0;
}