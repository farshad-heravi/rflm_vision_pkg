#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <depthai/depthai.hpp>
#include <depthai/device/DataQueue.hpp>
#include <chrono>
#include <thread>


// Camara class to handle OAK-D camera operations/communications
class Camera {
    public:
        Camera(int FPS, int lensPos, int marker_color, int object_color, bool realtime_visualization);
        std::tuple<double, double> get_target_position();
        void calibrate();
        bool is_camera_connected = false;
        bool is_camera_calibrated = false;

    private:
        cv::Mat capture_image();
        cv::Mat apply_mask(cv::Mat image, int target);
        cv::Mat apply_threshold(const cv::Mat& image);
        std::tuple<int, int, double, double> find_countors(const cv::Mat& masked_image, int target);
        double pixel2mm(int value);
        double offset_correction(double value);

        dai::Pipeline pipeline_;
        std::shared_ptr<dai::DataOutputQueue> outputQueue_;

        int fps_;                               // Frame per Second (max 60 for the OAK-D RGB camera)
        int lensPos_;                           // an integer between 0 and 255, used for manual focus of the camera
        int marker_color_, object_color_;         // marker and object color codes (0 for red, 1 for yellow, 2 for green)
        double pixel2mm_coefficient_;           // pixel 2 milimeter conversion ration
        bool realtime_visualization_ = false;   // whether to visualize the picture while working
        bool is_object_detected_ = false;       // whether an external object is detected or not
        double calibration_offset_;

        int calibration_data_count_ = 1000;
};

#endif // CAMERA_HPP