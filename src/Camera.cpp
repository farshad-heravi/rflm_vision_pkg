#include "Camera.hpp"

Camera::Camera(int FPS, int lensPos, int marker_color, int object_color, bool realtime_visualization) :
    fps_(FPS),
    lensPos_(lensPos),
    marker_color_(marker_color),
    object_color_(object_color),
    realtime_visualization_(realtime_visualization)
{
    // Define source and output
    auto camRgb = pipeline_.create<dai::node::ColorCamera>();
    auto xoutRgb = pipeline_.create<dai::node::XLinkOut>();
    xoutRgb->setStreamName("rgb");

    // Set Image Properties
    camRgb->setPreviewSize(1920/8, 1080/8);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setFps(fps_);

    // Create an ImageManip node, used for cropping of the incoming frames
    auto imageManip = pipeline_.create<dai::node::ImageManip>();
    // Set the imageManip properties for cropping
    dai::Point2f topLeftNorm(0.0f, 0.2f);       // Normalized coordinates for top left corner of the ROI
    dai::Point2f bottomRightNorm(0.7f, 0.53f);  // Normalized coordinates for bottom right corner of the ROI
    imageManip->initialConfig.setCropRect(topLeftNorm.x, topLeftNorm.y, bottomRightNorm.x, bottomRightNorm.y);

    camRgb->video.link(imageManip->inputImage);     // Link the camera to the ImageManip node
    xoutRgb->setStreamName("output");               // Set the name of the XLinkOut stream; will be used to get the frames from the output of imageManip node
    imageManip->out.link(xoutRgb->input);           // Link the ImageManip node's output to the XLinkOut node

    // Set Camera Control Parameters
    auto controlIn = pipeline_.create<dai::node::XLinkIn>();
    auto configIn = pipeline_.create<dai::node::XLinkIn>();
    controlIn->setStreamName("control");
    configIn->setStreamName("config");
    controlIn->out.link(camRgb->inputControl);
    configIn->out.link(camRgb->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline_, dai::UsbSpeed::SUPER);

    // camera control
    auto controlQueue = device.getInputQueue("control");
    auto configQueue = device.getInputQueue("config");
    dai::CameraControl ctrl;
    ctrl.setManualFocus(lensPos_);
    ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
    // ctrl.setManualWhiteBalance(6000);
    controlQueue->send(ctrl);

    std::cout << "Connected cameras: " << device.getConnectedCameraFeatures() << std::endl;
    std::cout << "Usb speed: " << device.getUsbSpeed() << std::endl;      // Print USB speed
    if(device.getBootloaderVersion()) { // Bootloader version
    std::cout << "Bootloader version: " << device.getBootloaderVersion()->toString() << std::endl;
    }
    std::cout << "Device name: " << device.getDeviceName() << " Product name: " << device.getProductName() << std::endl;    // Device name

    outputQueue_ = device.getOutputQueue("output", 1, false);   // Output queue will be used to get the frames from the output of imageManip node
    is_camera_connected = true;
}

cv::Mat Camera::capture_image()
{
    if (is_camera_connected) {
        auto inRgb = outputQueue_->get<dai::ImgFrame>();    // Get a frame
        cv::Mat image = inRgb->getCvFrame();                //convertToCvMat
        return image;
    } else {
        std::cout << "Camera is not working!" << std::endl;
        return cv::Mat();
    }
}

std::tuple<double, double> Camera::get_target_position()    // get new frame from camera and perform image processing
{
    cv::Mat image = capture_image();
    // apply mask
    cv::Mat masked_marker = apply_mask(image, 0);   // filter marker
    cv::Mat masked_object = apply_mask(image, 1);   // filter object

    // find marker/object center of area
    std::tuple<int, int, double, double> result_marker, result_object;  // in [px, px, m, m] units
    result_marker = find_countors(masked_marker, 0);
    result_object = find_countors(masked_object, 1);
    int cX_marker = std::get<0>(result_marker),      cY_marker = std::get<1>(result_marker);
    double cXmm_marker = std::get<2>(result_marker);
    int cX_object = std::get<0>(result_object),      cY_object = std::get<1>(result_object);
    double cXmm_object = std::get<2>(result_object);

    if (realtime_visualization_) {
        cv::circle(image, cv::Point(cX_marker, cY_marker), 5, cv::Scalar(0, 0, 255), -1);     // represent center of marker with a red dot
        cv::circle(image, cv::Point(cX_object, cY_object), 5, cv::Scalar(255, 255, 255), -1); // represent center of obejct with a white dot
    }

    // rectify offsets using calibration data
    cXmm_marker = offset_correction(cXmm_marker);
    cXmm_object = -offset_correction(cXmm_object);

    return std::make_tuple(cXmm_marker, cXmm_object);
}

void Camera::calibrate()
{
    std::cout << "##########" << std::endl;
    std::cout << "Calibrating the camera . . . " << std::endl;
    std::cout << "Please keep it still!" << std::endl;

    std::vector<double> calibration_data(calibration_data_count_);

    int counter = 0;
    while (counter < calibration_data_count_) {
        std::tuple<double, double> result;      // marker and object position (if exists)
        result = get_target_position();
        double marker_position = std::get<0>(result);
        calibration_data.push_back(marker_position);
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));     // sleep for definite ms for next frame
    }
    // calculate the mean value
    double sum = std::accumulate(calibration_data.begin(), calibration_data.end(), 0.0);
    calibration_offset_ = sum / calibration_data.size();
    is_camera_calibrated = true;

    std::cout << "Camera Calibration Done!" << std::endl;
    std::cout << "##########" << std::endl;
}

double Camera::offset_correction(double value)
{
    return value - calibration_offset_;
}

cv::Mat Camera::apply_mask(cv::Mat image, int target)   // target: 0 for marker; 1 for object
{
    int hMin, sMin, vMin, hMax, sMax, vMax;
    if (target == 0) {
        if ( marker_color_ == 0 ) {    // red marker
            hMin = 0; sMin = 201; vMin = 0;
            hMax = 179; sMax = 255; vMax = 255;
        } else if ( marker_color_ == 1 ) { // yellow marker
            hMax = 30; sMax = 255; vMax = 255;
            hMin = 20; sMin = 100; vMin = 100;
        }
    } else if (target == 1){
        if ( object_color_ == 2 ) {    // green object
            hMin = 35; sMin = 60; vMin = 40;
            hMax = 85; sMax = 255; vMax = 255;
        }
    }
    cv::Scalar lower(hMin, sMin, vMin);
    cv::Scalar upper(hMax, sMax, vMax);

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // Create a mask based on the HSV range
    cv::Mat mask;
    cv::inRange(hsv, lower, upper, mask);

    // Apply the mask to the original image
    cv::Mat output;
    cv::bitwise_and(image, image, output, mask);
    return output;
}

cv::Mat Camera::apply_threshold(const cv::Mat& image) {
    // Convert image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // Convert the grayscale image to binary image
    cv::Mat thresh;
    cv::threshold(gray_image, thresh, 0, 255, cv::THRESH_BINARY);

    return thresh;
}

std::tuple<int, int, double, double> Camera::find_countors(const cv::Mat& masked_image, int target)  // target: 0 for marker; 1 for object
{
    cv::Mat thresh = apply_threshold(masked_image);

    // Calculate moments of the binary image
    cv::Moments M = cv::moments(thresh, true);

    // Calculate x, y coordinates of the center
    int cX = 0, cY = 0;
    if (M.m00 != 0) {
        if (target == 1 && M.m00<10)
            return std::make_tuple(-0, -0, -0, -0);         // no object is found yet
        
        // calculate the center of objects in pixel
        cX = static_cast<int>(M.m10 / M.m00);
        cY = static_cast<int>(M.m01 / M.m00);
        if ( target == 1 && !is_object_detected_)
            is_object_detected_ = !is_object_detected_;
    }
    return std::make_tuple(cX, cY, pixel2mm(cX), pixel2mm(cY));
}

double Camera::pixel2mm(int value)
{
    return pixel2mm_coefficient_ * value;
}

