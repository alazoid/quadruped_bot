#include "esp_http_server.h"
#include "esp_camera.h"
#include "fd_forward.h"
#include "fr_forward.h"

extern bool isYelling;
void playYell();

static mtmn_config_t mtmn_config = {0};
static face_id_list id_list = {0};
httpd_handle_t camera_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    
    // initialize face memory
    face_id_init(&id_list, 5, 5); 
    mtmn_config = mtmn_init_config();

    while(true){
        fb = esp_camera_fb_get();
        if (!fb) return ESP_FAIL;

        // Convert frame to matrix for analysis
        dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);

        // Detect Faces
        box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
        
      if (net_boxes) {
            // Try to match face with enrolled IDs
            int matched_id = face_recognize(image_matrix, &recognition_model_params, &id_list);
            
            if (matched_id < 0) {
                // UNKNOWN FACE DETECTED
                Serial.println("STRANGER DANGER!");
                bark(); // Call the new bark function
            } else {
                // AUTHORIZED PERSON
                Serial.printf("Hello, User %d\n", matched_id);
                isYelling = false;
                digitalWrite(ISD1820_PLAY_PIN, LOW); 
            }
            free(net_boxes->box); free(net_boxes->landmark); free(net_boxes);
        } else {
            // NO FACE IN VIEW
            isYelling = false;
            digitalWrite(ISD1820_PLAY_PIN, LOW);
        }

        dl_matrix3du_free(image_matrix);
        esp_camera_fb_return(fb);
        
        // Add a small delay so we don't crash the CPU
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return res;
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &stream_uri);
    }
}
