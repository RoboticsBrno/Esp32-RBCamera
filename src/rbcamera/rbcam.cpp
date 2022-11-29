#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_camera.h"
#include "esp_jpg_decode.h"
#include "esp_log.h"

#include "apriltag/apriltag.h"
#include "apriltag/tag16h5.h"

#include "rbcam.h"

namespace rb {


RbCamera& RbCamera::get() {
    static RbCamera instance;
    return instance;
}


RbCamera::RbCamera() : m_detector(nullptr) {

}

esp_err_t RbCamera::init(const RbCameraConfig& cfg) {
    if(m_detector != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    static const camera_config_t camera_config = {
        .pin_pwdn  = 32,
        .pin_reset = -1,
        .pin_xclk = 0,
        .pin_sscb_sda = 26,
        .pin_sscb_scl = 27,

        .pin_d7 = 35,
        .pin_d6 = 34,
        .pin_d5 = 39,
        .pin_d4 = 36,
        .pin_d3 = 21,
        .pin_d2 = 19,
        .pin_d1 = 18,
        .pin_d0 = 5,
        .pin_vsync = 25,
        .pin_href = 23,
        .pin_pclk = 22,

        .xclk_freq_hz = 20000000, //EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
        .frame_size = cfg.framesize, //QQVGA-QXGA Do not use sizes above QVGA when not JPEG

        .jpeg_quality = cfg.jpeg_quality, //0-63 lower number means higher quality

        .fb_count = 3, //if more than one, i2s runs in continuous mode.
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY,//CAMERA_GRAB_LATEST. Sets when buffers should be filled
    };

    if(camera_config.pin_pwdn != -1) {
        gpio_set_direction((gpio_num_t)camera_config.pin_pwdn, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)camera_config.pin_pwdn, 0);
    }

    esp_err_t cam_err;
    for(int i = 0; i < 10; ++i) {
        cam_err = esp_camera_init(&camera_config);
        if(cam_err == ESP_OK) {
            break;
        } else if(cam_err != ESP_ERR_NOT_FOUND) {
            return cam_err;
        }
        gpio_set_level((gpio_num_t)camera_config.pin_pwdn, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level((gpio_num_t)camera_config.pin_pwdn, 0);
    }

    if(cam_err != ESP_OK) {
        return cam_err;
    }

    auto *family = &tag16h5_family;
    m_detector = apriltag_detector_create();

    // Sets family and how many wrong bits to tolerate
    apriltag_detector_add_family_bits(m_detector, family, 0);

    m_detector->quad_decimate = 2; // Decimate input image by this factor
    m_detector->quad_sigma = 0.0;  // Apply low-pass blur to input; negative sharpens
    m_detector->nthreads = 1;
    m_detector->debug = false;
    m_detector->refine_edges = true; // Spend more time trying to align edges of tags

    m_tag_queue = xQueueCreate(4, sizeof(RbCameraTagDetected));

    xTaskCreate(processingTask, "rbcam_task", 3584, this, 0, NULL);

    return ESP_OK;
}

void RbCamera::processingTask(void *camVoid) {
    auto *self = (RbCamera*)camVoid;

    auto *fb = esp_camera_fb_get();
    uint8_t *grey_frame =  (uint8_t*)heap_caps_malloc(fb->width*fb->height, MALLOC_CAP_SPIRAM);
    esp_camera_fb_return(fb);

    while(true) {
        fb = esp_camera_fb_get();
        self->m_last_fb.reset(fb, esp_camera_fb_return);

        const jpeg_ctx dec_ctx = {
            .fb = fb,
            .w = fb->width,
            .h = fb->height,
            .grey_frame = grey_frame,
        };

      /*  esp_jpg_decode(fb->len, JPG_SCALE_NONE, jpgRead, jpgGrescaleWrite, (void*)&dec_ctx);

        image_u8_t im = {
            .width = (int32_t)dec_ctx.w,
            .height = (int32_t)dec_ctx.h,
            .stride = (int32_t)dec_ctx.w,
            .buf = dec_ctx.grey_frame,
        };

        zarray_t *detections = apriltag_detector_detect(self->m_detector, &im);
        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            RbCameraTagDetected tag;
            tag.decision_margin = det->decision_margin;
            tag.id = (uint16_t)det->id;
            tag.hamming = (uint8_t)det->hamming;
            memcpy(tag.corners, det->p, 4*2*sizeof(float));
            memcpy(tag.center, det->c, 2*sizeof(float));

            if(xQueueSend(self->m_tag_queue, &tag, 0) != pdTRUE) {
                ESP_LOGW("RbCam", "tag queue full, not reading fast enough!");
            }
        }
        apriltag_detections_destroy(detections);*/

        taskYIELD();
    }
}

uint32_t RbCamera::jpgRead(void *arg, size_t index, uint8_t *buf, size_t len) {
    jpeg_ctx * ctx = (jpeg_ctx *)arg;
    if(buf) {
        memcpy(buf, ctx->fb->buf + index, len);
    }
    return len;
}

bool RbCamera::jpgGrescaleWrite(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
    jpeg_ctx * ctx = (jpeg_ctx *)arg;
    if(!data){
        if(x == 0 && y == 0){
            //write start
        } else {
            //write end
        }
        return true;
    }

    size_t jw = ctx->w*3;
    size_t jw2 = ctx->w;
    size_t t = y * jw;
    size_t t2 = y * jw2;
    size_t b = t + (h * jw);
    size_t l = x;
    uint8_t *out = ctx->grey_frame;
    uint8_t *o = out;
    size_t iy, iy2, ix, ix2;

    w = w * 3;

    for(iy=t, iy2=t2; iy<b; iy+=jw, iy2+=jw2) {
        o = out+iy2+l;
        for(ix2=ix=0; ix<w; ix+= 3, ix2 +=1) {
            const uint8_t r = data[ix];
            const uint8_t g = data[ix+1];
            const uint8_t b = data[ix+2];
            const float px_linear = r*0.3f + g*0.59f + b*0.11f;
            o[ix2] = std::min(px_linear, 255.f);
        }
        data+=w;
    }
    return true;
}

static void client_error(int fd, int status, char* msg, char* longmsg) {
    char buf[256];
    sprintf(buf, "HTTP/1.1 %d %s\r\n", status, msg);
    sprintf(buf + strlen(buf),
        "Content-length: %u\r\n\r\n", strlen(longmsg));
    sprintf(buf + strlen(buf), "%s", longmsg);
    write(fd, buf, strlen(buf));
}

void RbCamera::rbWebCallback(const char *request_path, int out_fd) {
    const char camera_fn[] = "camera.jpg";
    if(strncmp(request_path, camera_fn, sizeof(camera_fn)-1) != 0) {
        client_error(out_fd, 404, "Error", "Not found");
        return;
    }

    auto fb = RbCamera::get().getLastFb();
    if(!fb) {
        client_error(out_fd, 500, "Error", "Camera not ready");
        return;
    }

    char buf[256];
    snprintf(buf, sizeof(buf),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %u\r\n\r\n", fb->len);
    write(out_fd, buf, strlen(buf));
    write(out_fd, fb->buf, fb->len);
}


};
