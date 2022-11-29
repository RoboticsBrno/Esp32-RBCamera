#pragma once

#include <esp_err.h>
#include <memory>
#include <freertos/queue.h>

#include "apriltag/apriltag.h"

#include "esp_camera.h"

namespace rb {

struct RbCameraConfig {
    // Camera
    framesize_t framesize;
    uint8_t jpeg_quality;  //0-63 lower number means higher quality

    // AprilTags
    float quad_decimate; // Decimate input image by this factor
    bool refine_edges; // Spend more time trying to align edges of tags
};

struct RbCameraTagDetected {
    // The corners of the tag in image pixel coordinates. These always
    // wrap counter-clock wise around the tag.
    float corners[4][2];

    // The center of the detection in image pixel coordinates.
    float center[2];

    // A measure of the quality of the binary decoding process: the
    // average difference between the intensity of a data bit versus
    // the decision threshold. Higher numbers roughly indicate better
    // decodes. This is a reasonable measure of detection accuracy
    // only for very small tags-- not effective for larger tags (where
    // we could have sampled anywhere within a bit cell and still
    // gotten a good detection.)
    float decision_margin;

    // The decoded ID of the tag
    uint16_t id;

    // How many error bits were corrected? Note: accepting large numbers of
    // corrected errors leads to greatly increased false positive rates.
    // NOTE: As of this implementation, the detector cannot detect tags with
    // a hamming distance greater than 2.
    uint8_t hamming;
};

class RbCamera {
public:
    static RbCamera& get();

    esp_err_t init(const RbCameraConfig& cfg);

    // queue of RbCameraTagDetected structs
    QueueHandle_t tagQueue() { return m_tag_queue; }

    std::shared_ptr<camera_fb_t> getLastFb() const {
        return m_last_fb;
    };

    static void rbWebCallback(const char *request_path, int out_fd);

private:
    RbCamera();
    RbCamera(const RbCamera&) = delete;
    ~RbCamera() {}

    struct jpeg_ctx {
        camera_fb_t *fb;
        uint32_t w;
        uint32_t h;
        uint8_t *grey_frame;
    };

    static void processingTask(void *camVoid);
    static uint32_t jpgRead(void *arg, size_t index, uint8_t *buf, size_t len);
    static bool jpgGrescaleWrite(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data);

    apriltag_detector_t *m_detector;
    std::shared_ptr<camera_fb_t> m_last_fb;
    QueueHandle_t m_tag_queue;
};
};
