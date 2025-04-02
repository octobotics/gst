#include <ros/ros.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>
#include <std_srvs/Trigger.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <csignal> 
#include <thread>
GstElement *pipeline, *appsink;
cv::Mat captured_frame;
bool running = true;

void signalHandler(int signum) {
    ROS_INFO("Interrupt signal (%d) received. Shutting down...", signum);
    running = false;
    gst_element_set_state(pipeline, GST_STATE_NULL); 
    ros::shutdown(); 
}

std::string getTimestampedFilename(const std::string &directory) {
    std::ostringstream filepath;
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);

    filepath << directory << "/screenshot_"
             << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".jpg";
    return filepath.str();
}

bool captureFrame(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    std::string directory = std::string(getenv("HOME")) + "/Desktop/OctoPilot/SCREENSHOT";

    std::string filename = getTimestampedFilename(directory);

    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    if (!sample) {
        ROS_ERROR("Failed to capture frame from appsink.");
        res.success = false;
        res.message = "Failed to capture frame from appsink.";
        return false;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    if (!caps) {
        ROS_ERROR("Failed to get caps from sample.");
        gst_sample_unref(sample);
        res.success = false;
        res.message = "Failed to get caps from sample.";
        return false;
    }

    GstStructure *s = gst_caps_get_structure(caps, 0);
    int width, height;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    captured_frame = cv::Mat(height, width, CV_8UC3, (char *)map.data).clone();
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    if (cv::imwrite(filename, captured_frame)) {
        ROS_INFO("Frame captured and saved to %s", filename.c_str());
        res.success = true;
        res.message = "Frame saved to " + filename;
    } else {
        ROS_ERROR("Failed to save the frame to %s", filename.c_str());
        res.success = false;
        res.message = "Failed to save frame to " + filename;
    }

    return true;
}

void runGStreamerPipeline() {
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    GstBus *bus = gst_element_get_bus(pipeline);
    GstMessage *msg;

    while (running) {
        msg = gst_bus_timed_pop_filtered(bus, GST_MSECOND * 100,
                                         GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg != nullptr) {
            GError *err;
            gchar *debug_info;
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR:
                    gst_message_parse_error(msg, &err, &debug_info);
                    ROS_ERROR("GStreamer error: %s", err->message);
                    g_error_free(err);
                    g_free(debug_info);
                    break;
                case GST_MESSAGE_EOS:
                    ROS_INFO("End of stream reached.");
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
            break;
        }
    }

    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gstreamer_node");
    ros::NodeHandle nh;

    gst_init(nullptr, nullptr);

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    pipeline = gst_parse_launch(
        "udpsrc address=10.42.0.6 port=5002 caps=\"application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96\" ! "
        "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=appsink ! autovideosink", nullptr);

    appsink = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    if (!appsink) {
        ROS_ERROR("Failed to find appsink element in the pipeline.");
        return -1;
    }

    gst_app_sink_set_emit_signals((GstAppSink *)appsink, true);
    gst_app_sink_set_drop((GstAppSink *)appsink, true);
    gst_app_sink_set_max_buffers((GstAppSink *)appsink, 1);

    ros::ServiceServer service = nh.advertiseService("/capture_frame", captureFrame);
    ROS_INFO("Capture frame service ready.");

    std::thread gst_thread(runGStreamerPipeline);

    while (ros::ok() && running) {
        ros::spinOnce();
    }

    gst_thread.join();
    return 0;
}
