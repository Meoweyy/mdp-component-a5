#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>
#include <time.h> // For pthread_cond_timedwait
#include <errno.h> // For ETIMEDOUT
#include <ctype.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "shared_types.h"
#include "rpi_hal.h"
#include "json_parser.h" // New include

// Definition for DIR_MAP_ANDROID_STR, declared in shared_types.h
// Maps RPi internal (0,2,4,6) to Android's (1=N, 2=E, 3=S, 4=W)
const char* DIR_MAP_ANDROID_STR[8] = {
    "1", "1", "2", "2", "3", "3", "4", "4"
};

// --- Configuration Toggles ---
// Set to 1 to use Named Pipes (Simulation), 0 to use /dev/ devices (Hardware)
#ifndef STM32_SIM
#define STM32_SIM   0
#endif

#ifndef ANDROID_SIM
#define ANDROID_SIM 0
#endif

// Device Paths
const char* STM32_HW_DEVICE    = "/dev/ttyACM0";
const char* ANDROID_HW_DEVICE  = "/dev/rfcomm0";

const char* STM32_PIPE_WRITE   = "rpi_to_stm";
const char* STM32_PIPE_READ    = "stm_to_rpi";
const char* ANDROID_PIPE_READ  = "android_to_rpi";
const char* ANDROID_PIPE_WRITE = "rpi_to_android";

// Laptop running the algorithm + image servers, on the robot's WiFi network.
// Check with `ipconfig` on the laptop after every reconnect -- the address can
// change, and the old 192.168.22.x values here were another group's machines.
// Benjamin's laptop, 10 Sep 2026: 192.168.23.11 (Pi is the gateway, .23.1).
const char* PATHFINDING_SERVER_URL = "http://192.168.23.11:5000/path";
const char* IMAGE_SERVER_URL       = "http://192.168.23.11:4000/detect";
// Bullseye recovery endpoint. Same host/port as PATHFINDING_SERVER_URL -- keep them in sync.
const char* BULLSEYE_SERVER_URL    = "http://192.168.23.11:5000/bullseye";

// --- Bullseye recovery ---
// When the camera sees the bullseye marker instead of a symbol, ask the algorithm server
// for a route to a different face of the same obstacle, and drive that instead.
// Set to 0 to disable the whole feature: navigation then behaves exactly as before.
#ifndef ENABLE_BULLSEYE_RECOVERY
#define ENABLE_BULLSEYE_RECOVERY 1
#endif
// img_id the image server reports for the marker (never a valid answer).
#define BULLSEYE_IMG_ID 41
// How long navigation waits at a snapshot for the recognition verdict before moving on.
#define RECOGNITION_WAIT_SECONDS 12

// --- Livestream (PC YOLO) wiring for Task 2 ---
// Set to 1 to use livestream-triggered detection for Task 2 instead of uploading images to IMAGE_SERVER_URL.
// Can be overridden from the Makefile with: -DUSE_LIVESTREAM_TASK2=0/1
#ifndef USE_LIVESTREAM_TASK2
#define USE_LIVESTREAM_TASK2 0
#endif
// PC host running Image/livestream/stream_test.py (LOCK trigger listener, default port 5002).
// IMPORTANT: set this to your laptop IP on the Pi network.
const char* PC_LOCK_HOST = "192.168.22.26";
const int   PC_LOCK_PORT = 5002;
// Pi Python stream_server.py forwards RESULT JSON to this localhost UDP port.
const int   RESULT_UDP_PORT = 5555;

const int BAUD_RATE = 115200;
const char* CAPTURE_FILENAME = "capture.jpg";

// --- Post-turn overshoot correction ---------------------------------------
// The STM's TURN90L/TURN90R routines drive a short straight segment AFTER the
// 90-degree arc completes, and that segment is too long. The arcs themselves
// are correct: both measure exactly 30cm of forward travel (= 3 cells), which
// is what the algorithm server assumes. Only the lateral component overshoots.
//
// Because the overshoot is a straight run in the post-turn heading, an equal
// reverse cancels it exactly, and reverses into space the car just drove
// through. REVS was measured accurate to the centimetre.
//
// Measured 10 Sep 2026 with RPI/stm_test.py, start (15,15) facing North:
//   TURN90R -> ended (60,45), expected (45,45)  => 15cm lateral overshoot
//   TURN90L -> ended (65,45), expected (75,45)  => 10cm lateral overshoot
//
// Set either to 0 to disable that correction. Once the firmware's post-turn
// pads are shortened, set BOTH to 0 -- leaving them on would then overcorrect.
//
// 14 Sep 2026: ZEROED. The MDP-STM-14-9 firmware lands TURN90L/TURN90R on
// target by itself, so these would now pull each forward turn 15/10cm SHORT.
// Every error is corrected in one place -- firmware or Pi, never both.
const int TURN_RIGHT_CORRECTION_CM = 0;
const int TURN_LEFT_CORRECTION_CM  = 0;

// Reverse turns (BL90 -> REVL, BR90 -> REVR) fail differently: they overshoot
// EQUALLY in both axes, which means the arc radius is too large (~35cm instead
// of 30cm) rather than a post-pad being too long.
//
// Measured 10 Sep 2026, start (105,105) facing North:
//   REVL -> ended (70,70),  expected (75,75)  => 5cm over on both axes
//   REVR -> ended (140,70), expected (135,75) => 5cm over on both axes
//
// A symmetric overshoot needs a DIAGONAL correction relative to the final
// heading, which no single straight move can provide. So it is split in two,
// both forward: PRE runs before the turn (car still on its original heading)
// and POST runs after (car on its new heading). Together they cancel it.
// Set both to 0 once the firmware's reverse-turn radius is corrected.
//
// 14 Sep 2026: REVL is calibrated in the 14-9 firmware, so it gets no
// correction. REVR cannot be calibrated there (servo saturates ~225; lands
// ~7cm long along the reverse direction; PWM makes no difference). PRE now
// applies to BR90 ONLY: a 7cm forward move before the arc cancels it exactly.
//
// Safety note: the nudge drives 7cm INTO the space ahead. After a SNAP the car
// is ~14.5cm from the obstacle face, so it clears by ~7cm nominally. Accepted.
const int REVERSE_TURN_PRE_CM  = 7;   // FWD before BR90 only
const int REVERSE_TURN_POST_CM = 0;   // unused

// --- Global Shared Application Context ---
SharedAppContext g_app_context;

// Separate descriptor for reading ACKs in simulation mode
int g_stm32_ack_fd = -1;

// Task 2 livestream RESULT synchronization
static pthread_mutex_t g_task2_result_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_task2_result_cond  = PTHREAD_COND_INITIALIZER;
static int g_task2_result_ready[3] = {0, 0, 0}; // index 1..2
static int g_task2_result_img_id[3] = {-1, -1, -1};

// Signal handler for Ctrl+C to ensure clean exit and port closure
void handle_sigint(int sig) {
    printf("\n[Signal] SIGINT (Ctrl+C) received. Closing ports and exiting...\n");
    
    // Explicitly close all file descriptors if they were opened
    if (g_stm32_ack_fd != -1) close(g_stm32_ack_fd);
    if (g_app_context.stm32_fd != -1 && g_app_context.stm32_fd != g_stm32_ack_fd) {
        close(g_app_context.stm32_fd);
    }
    if (g_app_context.android_fd != -1) close(g_app_context.android_fd);
    if (g_app_context.android_write_fd != -1 && g_app_context.android_write_fd != g_app_context.android_fd) {
        close(g_app_context.android_write_fd);
    }
    
    curl_global_cleanup();
    exit(0);
}


// =================================================================================
// THREAD 3: Image Processing (Temporary, "Fire-and-Forget")
// =================================================================================
// Updated post_image_to_server_thread to return response for parsing
static int post_image_to_server_thread(int obstacle_id, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct chunk = { .memory = malloc(1), .size = 0 };
    if (chunk.memory == NULL) { // Check for malloc failure
        fprintf(stderr, "[ImgThread] Failed to allocate memory for CURL response.\n");
        return -1;
    }

    curl = curl_easy_init();
    if (curl) {
        printf("[ImgThread] Sending image for obstacle %d to image server at %s...\n", obstacle_id, IMAGE_SERVER_URL);
        curl_mime *form = curl_mime_init(curl);
        curl_mimepart *field;

        field = curl_mime_addpart(form); curl_mime_name(field, "image"); curl_mime_filedata(field, CAPTURE_FILENAME);
        char id_str[10]; snprintf(id_str, sizeof(id_str), "%d", obstacle_id);
        field = curl_mime_addpart(form); curl_mime_name(field, "object_id"); curl_mime_data(field, id_str, CURL_ZERO_TERMINATED);

        curl_easy_setopt(curl, CURLOPT_URL, IMAGE_SERVER_URL);
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            long code; curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
            if (code >= 200 && code < 300) {
                if (response_buffer != NULL && chunk.memory != NULL) { // Only copy if buffer provided and memory exists
                    strncpy(response_buffer, chunk.memory, buffer_size - 1);
                    response_buffer[buffer_size - 1] = '\0';
                }
                result = 0;
            } else {
                fprintf(stderr, "[ImgThread] Image server returned non-2xx response: %ld\n", code);
            }
        } else {
            fprintf(stderr, "[ImgThread] post_image_to_server_thread failed: %s\n", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);
        curl_mime_free(form);
    } else {
        fprintf(stderr, "[ImgThread] curl_easy_init() failed.\n");
    }
    free(chunk.memory); // Free after curl_easy_cleanup
    return result;
}

void* process_image_thread(void* args) {
    ImageTaskArgs* task_args = (ImageTaskArgs*)args;
    SharedAppContext* context = task_args->context;
    char image_server_response[2048]; // Buffer for image server JSON response
    char class_label[100]; // To hold the detected class label
    int detected_img_id = -1;  // What this snap resolved to (-1 = nothing). Logging only --
                               // the all-faces sweep below continues regardless of this value.

    printf("[ImgThread] Capturing image for obstacle %d...\n", task_args->obstacle_id);
    if (capture_image(CAPTURE_FILENAME) != 0) {
        fprintf(stderr, "[ImgThread] Failed to capture image.\n");
        // Signal image capture failure by setting ID to 0 or another error code, or just don't signal
        pthread_mutex_lock(&context->image_capture_mutex);
        context->last_image_capture_id = 0; // Indicate failure or no successful capture
        pthread_cond_signal(&context->image_capture_cond);
        pthread_mutex_unlock(&context->image_capture_mutex);
    } else {
        printf("[ImgThread] Image captured successfully for obstacle %d.\n", task_args->obstacle_id);
        // Signal image capture success
        pthread_mutex_lock(&context->image_capture_mutex);
        context->last_image_capture_id = task_args->obstacle_id;
        pthread_cond_signal(&context->image_capture_cond);
        pthread_mutex_unlock(&context->image_capture_mutex);

        // Notify Android that capture is starting
        char capture_status[100];
        snprintf(capture_status, sizeof(capture_status), "Capturing image for obstacle %d", task_args->obstacle_id);
        send_android_json(context->android_write_fd, "image", capture_status, false);

        // Send image detection result to Android in the new JSON format
        if (post_image_to_server_thread(task_args->obstacle_id, image_server_response, sizeof(image_server_response)) == 0) {
            printf("[ImgThread] Image server response: %s\n", image_server_response);

            /* Compatible with object_detection_server.py: server returns success, detected, count, objects[] with class_label, img_id, confidence, bbox.
             * Use "count" for detection (integer); prefer "img_id" from JSON; do not skip Bullseye — use first object with valid img_id. */
            int count = 0;
            if (get_json_int(image_server_response, "count", &count) != 0 || count <= 0) {
                printf("[ImgThread] No object detected by image server for obstacle %d.\n", task_args->obstacle_id);
            } else {
                // Robustly find the start of the "objects" array
                const char* objects_key = strstr(image_server_response, "\"objects\"");
                const char* objects_array_start = NULL;
                
                if (objects_key) {
                    const char* p = objects_key + strlen("\"objects\"");
                    while (*p && isspace((unsigned char)*p)) p++;
                    if (*p == ':') {
                        p++;
                        while (*p && isspace((unsigned char)*p)) p++;
                        if (*p == '[') {
                            objects_array_start = p + 1;
                        }
                    }
                }

                if (objects_array_start) {
                    const char* ptr = objects_array_start;
                    int sent = 0;
                    while (*ptr && sent == 0) {
                        const char* obj_start = strchr(ptr, '{');
                        if (!obj_start) break;
                        
                        // Find matching closing brace
                        int depth = 0;
                        const char* p = obj_start;
                        const char* obj_end = NULL;
                        while (*p) {
                            if (*p == '{') depth++;
                            else if (*p == '}') {
                                depth--;
                                if (depth == 0) {
                                    obj_end = p;
                                    break;
                                }
                            }
                            p++;
                        }
                        
                        if (!obj_end) break;
                        
                        size_t obj_len = (size_t)(obj_end - obj_start + 1);
                        char single_obj_json[1024]; // Increased size just in case
                        if (obj_len >= sizeof(single_obj_json)) obj_len = sizeof(single_obj_json) - 1;
                        strncpy(single_obj_json, obj_start, obj_len);
                        single_obj_json[obj_len] = '\0';

                        class_label[0] = '\0'; 
                        if (get_json_string(single_obj_json, "class_label", class_label, sizeof(class_label)) != 0)
                            get_json_string(single_obj_json, "class", class_label, sizeof(class_label));
                        
                        if (class_label[0] != '\0') {
                            char* dash = strstr(class_label, " - ");
                            if (dash) *dash = '\0';
                            
                            int img_id = -1;
                            if (get_json_int(single_obj_json, "img_id", &img_id) != 0 || img_id < 0) {
                                img_id = get_img_id_from_class_name(class_label);
                            }
                            
                            if (img_id >= 0) {
                                // Find actual obstacle coordinates and direction for Android feedback
                                int obs_x = -1;
                                int obs_y = -1;
                                int obs_d = -1;
                                
                                pthread_mutex_lock(&context->lock);
                                for (int i = 0; i < context->obstacle_count; i++) {
                                    if (context->obstacles[i].id == task_args->obstacle_id) {
                                        obs_x = context->obstacles[i].x;
                                        obs_y = context->obstacles[i].y;
                                        obs_d = context->obstacles[i].d;
                                        break;
                                    }
                                }
                                pthread_mutex_unlock(&context->lock);

                                // Fallback to robot snap position if obstacle not found (though it should be in context)
                                if (obs_x == -1) {
                                    obs_x = task_args->robot_snap_position.x;
                                    obs_y = task_args->robot_snap_position.y;
                                    obs_d = task_args->robot_snap_position.d;
                                }

                                const char* dir_str = (obs_d >= 0 && obs_d < 8) ?
                                                       DIR_MAP_ANDROID_STR[obs_d] : "U";
                                
                                printf("[ImgThread] Forwarding detection to Android: obstacle_id=%d, class=%s, img_id=%d, pos=(%d,%d), d=%s\n", 
                                       task_args->obstacle_id, class_label, img_id, obs_x, obs_y, dir_str);

                                send_image_recognition_to_android(context->android_write_fd,
                                                                  obs_x,
                                                                  obs_y,
                                                                  dir_str,
                                                                  img_id,
                                                                  task_args->obstacle_id);
                                sent = 1;

#if ENABLE_BULLSEYE_RECOVERY
                                // Remember what this face resolved to. The decision to keep
                                // circling the block is made once, after all parsing, so that a
                                // face which detects NOTHING still advances the sweep.
                                detected_img_id = img_id;
#endif
                            }
                        }
                        ptr = obj_end + 1;
                    }
                    if (sent == 0)
                        fprintf(stderr, "[ImgThread] No valid object with img_id found in 'objects' array for obstacle %d.\n", task_args->obstacle_id);
                } else {
                    fprintf(stderr, "[ImgThread] Failed to find 'objects' array in server response for obstacle %d.\n", task_args->obstacle_id);
                }
            }
        } else {
            fprintf(stderr, "[ImgThread] Failed to upload image or no ACK received from image server.\n");
            send_android_ack(context->android_write_fd, "Failed to capture image for obstacle");
        }
    }

#if ENABLE_BULLSEYE_RECOVERY
    // --- All-faces sweep -------------------------------------------------------------
    // Seeing the bullseye marker on an obstacle puts it into "search mode": from then on
    // every remaining face is visited, whether or not a valid symbol turns up on the way.
    // The marker's own face goes into the checked list immediately, so it is never
    // revisited -- after the marker exactly three more sides get examined.
    // Runs here (not inside the detection branch) so a face that detects nothing at all
    // still advances the sweep instead of silently ending it.
    {
        // The face just examined is the one opposite the robot's heading.
        int examined_face = -1;
        if (task_args->robot_snap_position.d >= 0) {
            examined_face = (task_args->robot_snap_position.d + 4) % 8;
        }

        pthread_mutex_lock(&context->lock);
        bool searching_this = (context->bullseye_obstacle_id == task_args->obstacle_id);

        // A marker starts the sweep for this obstacle.
        if (detected_img_id == BULLSEYE_IMG_ID && !searching_this) {
            context->bullseye_obstacle_id = task_args->obstacle_id;
            context->bullseye_checked_count = 0;
            searching_this = true;
        }

        if (searching_this) {
            bool already_known = false;
            for (int f = 0; f < context->bullseye_checked_count; f++) {
                if (context->bullseye_checked_faces[f] == examined_face) {
                    already_known = true;
                    break;
                }
            }
            if (!already_known && examined_face >= 0 && context->bullseye_checked_count < 4) {
                context->bullseye_checked_faces[context->bullseye_checked_count++] = examined_face;
            }

            int faces_done = context->bullseye_checked_count;
            bool more_faces = (faces_done < 4);
            if (more_faces) {
                context->bullseye_snap_pos = task_args->robot_snap_position;
                context->bullseye_detected = true;
            }
            pthread_mutex_unlock(&context->lock);

            printf("[ImgThread] Obstacle %d: face %d examined (img_id=%d), %d/4 faces done.%s\n",
                   task_args->obstacle_id, examined_face, detected_img_id, faces_done,
                   more_faces ? " Continuing around the block." : " All faces examined.");
        } else {
            pthread_mutex_unlock(&context->lock);
        }
    }
#endif

    // Publish the verdict for this obstacle. Navigation may be waiting on it so that it
    // can act on a bullseye before leaving. Signalled on every path, including failures,
    // so a dead image server can never stall the mission.
    pthread_mutex_lock(&context->recognition_mutex);
    context->last_recognition_id = (uint32_t)task_args->obstacle_id;
    pthread_cond_broadcast(&context->recognition_cond);
    pthread_mutex_unlock(&context->recognition_mutex);

    free(task_args); // Free the dynamically allocated arguments
    return NULL;
}


#if ENABLE_BULLSEYE_RECOVERY
/**
 * Ask the algorithm server for a route to a different face of the obstacle that showed
 * the bullseye. On success the current command list is REPLACED by the returned route
 * (which also re-plans any obstacles still outstanding).
 *
 * Caller must NOT hold context->lock.
 * Returns 0 if a new route was installed, -1 otherwise (caller should carry on as before).
 */
static int request_bullseye_recovery(SharedAppContext* context) {
    char payload[2048];
    char obstacles_str[1500] = "";
    char faces_str[64] = "";
    int target_id, robot_x, robot_y, robot_dir;

    pthread_mutex_lock(&context->lock);
    target_id = context->bullseye_obstacle_id;
    robot_x   = context->bullseye_snap_pos.x;
    robot_y   = context->bullseye_snap_pos.y;
    robot_dir = context->bullseye_snap_pos.d;

    bool first_obs = true;
    for (int i = 0; i < context->obstacle_count; i++) {
        if (context->obstacles[i].id == 0) continue;
        char obs_item[100];
        if (!first_obs) strcat(obstacles_str, ",");
        snprintf(obs_item, sizeof(obs_item), "{\"id\":%d,\"x\":%d,\"y\":%d,\"d\":%d}",
                 context->obstacles[i].id, context->obstacles[i].x,
                 context->obstacles[i].y, context->obstacles[i].d);
        strcat(obstacles_str, obs_item);
        first_obs = false;
    }

    // Faces already ruled out, in algo directions (0=N, 2=E, 4=S, 6=W).
    for (int i = 0; i < context->bullseye_checked_count; i++) {
        char face_item[8];
        snprintf(face_item, sizeof(face_item), "%s%d", (i == 0) ? "" : ",",
                 context->bullseye_checked_faces[i]);
        strcat(faces_str, face_item);
    }
    pthread_mutex_unlock(&context->lock);

    snprintf(payload, sizeof(payload),
             "{\"obstacles\":[%s],\"robot_x\":%d,\"robot_y\":%d,\"robot_dir\":%d,"
             "\"target_obstacle_id\":%d,\"checked_faces\":[%s]}",
             obstacles_str, robot_x, robot_y, robot_dir, target_id, faces_str);
    printf("[Bullseye] Requesting recovery route: %s\n", payload);

    char response[4096];
    if (post_data_to_server(BULLSEYE_SERVER_URL, payload, response, sizeof(response)) != 0) {
        fprintf(stderr, "[Bullseye] Recovery server communication failed.\n");
        send_android_ack(context->android_write_fd, "Error: Bullseye recovery failed.");
        return -1;
    }
    printf("[Bullseye] Raw recovery response:\n---\n%s\n---\n", response);

    // Parse into scratch buffers first, so a bad reply cannot destroy the running route.
    Command new_commands[MAX_COMMANDS];
    int new_command_count = 0;
    SnapPosition new_snaps[MAX_SNAP_POSITIONS];
    int new_snap_count = 0;

    if (parse_command_route_from_server(response, new_commands, &new_command_count,
                                        new_snaps, &new_snap_count) != 0) {
        fprintf(stderr, "[Bullseye] Could not parse recovery route.\n");
        return -1;
    }
    if (new_command_count <= 0) {
        fprintf(stderr, "[Bullseye] Recovery route is empty -- no other face is reachable.\n");
        send_android_ack(context->android_write_fd, "Bullseye: no reachable face.");
        return -1;
    }

    pthread_mutex_lock(&context->lock);
    memcpy(context->commands, new_commands, sizeof(Command) * new_command_count);
    context->command_count = new_command_count;
    memcpy(context->snap_positions, new_snaps, sizeof(SnapPosition) * new_snap_count);
    context->snap_position_count = new_snap_count;
    context->snap_position_idx = 0;
    pthread_mutex_unlock(&context->lock);

    printf("[Bullseye] Installed recovery route: %d commands.\n", new_command_count);
    send_android_ack(context->android_write_fd, "Bullseye detected. Navigating around obstacle.");
    return 0;
}
#endif // ENABLE_BULLSEYE_RECOVERY

// =================================================================================
// THREAD 2: Navigation Executor (Main Logic)
// =================================================================================

// Send one extra command to the STM and block until its ACK, reusing the same
// ack mutex/condvar protocol as the main command loop. Used for the post-turn
// overshoot correction. Returns 0 on ACK, -1 on timeout/error/stop.
static int send_followup_and_wait(SharedAppContext* context, Command cmd, uint32_t cmd_id) {
    send_command_to_stm32(context->stm32_fd, cmd, cmd_id);
    printf("[NavThread] Sent follow-up command %u (Type: %d, Val: %d) to STM32. Waiting for ACK...\n",
           cmd_id, cmd.type, cmd.value);

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 15;

    int ack_result = 0;
    pthread_mutex_lock(&context->stm32_ack_mutex);
    while (context->stm32_last_ack_id != cmd_id && !context->stop_requested) {
        int rc = pthread_cond_timedwait(&context->stm32_ack_cond, &context->stm32_ack_mutex, &ts);
        if (rc == ETIMEDOUT) {
            fprintf(stderr, "[NavThread] Timeout waiting for ACK for follow-up command %u.\n", cmd_id);
            ack_result = -1;
            break;
        } else if (rc != 0) {
            fprintf(stderr, "[NavThread] Error waiting for ACK condition variable: %d\n", rc);
            ack_result = -1;
            break;
        }
    }
    if (ack_result == 0 && context->stm32_last_ack_id == cmd_id) {
        printf("[NavThread] Received ACK for follow-up command %u.\n", cmd_id);
    }
    pthread_mutex_unlock(&context->stm32_ack_mutex);

    if (context->stop_requested) return -1;
    return ack_result;
}

void execute_navigation() {
    SharedAppContext* context = &g_app_context;
    printf("[NavThread] State: [NAVIGATING]. Executing %d commands.\n", context->command_count);

    // Clear serial buffers (both read and write) to ensure we start clean
    flush_serial_port(context->stm32_fd);
    flush_serial_port(g_stm32_ack_fd); // Flush the ACK read channel
    flush_serial_port(context->android_write_fd);

    // Reset ACK state before starting
    pthread_mutex_lock(&context->stm32_ack_mutex);
    context->stm32_last_ack_id = 0;
    pthread_mutex_unlock(&context->stm32_ack_mutex);

    // Tell Android we are starting
    send_android_ack(context->android_write_fd, "ready-to-roll");

    pthread_mutex_lock(&context->lock);
    context->snap_position_idx = 0; // Reset snap position index for new navigation
#if ENABLE_BULLSEYE_RECOVERY
    // Fresh mission: forget any marker seen during a previous run.
    context->bullseye_detected = false;
    context->bullseye_obstacle_id = 0;
    context->bullseye_checked_count = 0;
#endif
    pthread_mutex_unlock(&context->lock);

    uint32_t current_cmd_id = 1; // Start command IDs from 1 for the sequence

    for (int i = 0; i < context->command_count; i++) {
        pthread_mutex_lock(&context->lock);
        // Time-based auto-stop: 5 minutes 30 seconds limit
        time_t now;
        time(&now);
        if (difftime(now, context->mission_start_time) > 330) {
            printf("[NavThread] 5:30 mission limit reached! Stopping robot now.\n");
            context->stop_requested = true;
        }

        if (context->stop_requested) {
            printf("[NavThread] Stop requested. Aborting navigation.\n");
            context->stop_requested = false;
            context->state = STATE_IDLE;
            pthread_mutex_unlock(&context->lock);
            break;
        }
        pthread_mutex_unlock(&context->lock);

        Command cmd = context->commands[i];
        if (cmd.type == CMD_SNAPSHOT) {
            // ... (Snapshot logic remains the same)
            printf("[NavThread] --- Spawning image thread for obstacle %d ---\n", cmd.value);
            pthread_t tid;
            ImageTaskArgs* args = malloc(sizeof(ImageTaskArgs));
            if (!args) {
                fprintf(stderr, "[NavThread] Failed to allocate ImageTaskArgs.\n");
                continue;
            }
            args->context = context;
            args->obstacle_id = cmd.value;
            // Get current snap position from context
            pthread_mutex_lock(&context->lock);
            if (context->snap_position_idx < context->snap_position_count) {
                args->robot_snap_position = context->snap_positions[context->snap_position_idx];
                context->snap_position_idx++;
            } else {
                // Fallback if snap positions don't match commands, should not happen with correct parsing
                args->robot_snap_position = (SnapPosition){.x = -1, .y = -1, .d = -1};
                fprintf(stderr, "[NavThread] Warning: Snap position index out of bounds.\n");
            }
            pthread_mutex_unlock(&context->lock);


            // Clear previous results before spawning. Bullseye recovery can re-snap the SAME
            // obstacle from another face, and a stale id here would satisfy the waits below
            // instantly with the old verdict.
            pthread_mutex_lock(&context->image_capture_mutex);
            context->last_image_capture_id = 0;
            pthread_mutex_unlock(&context->image_capture_mutex);
            pthread_mutex_lock(&context->recognition_mutex);
            context->last_recognition_id = 0;
            pthread_mutex_unlock(&context->recognition_mutex);

            pthread_create(&tid, NULL, process_image_thread, args);
            pthread_detach(tid); // Detach to allow thread to clean up its resources automatically

            printf("[NavThread] Spawning image thread for obstacle %d. Waiting for image capture confirmation...\n", cmd.value);

            struct timespec ts_img;
            clock_gettime(CLOCK_REALTIME, &ts_img);
            ts_img.tv_sec += 10; // Wait for up to 10 seconds for image capture confirmation

            int img_ack_result = 0; // 0 for success, -1 for error/timeout
            pthread_mutex_lock(&context->image_capture_mutex);
            while (context->last_image_capture_id != (uint32_t)cmd.value && !context->stop_requested) {
                int rc = pthread_cond_timedwait(&context->image_capture_cond, &context->image_capture_mutex, &ts_img);
                if (rc == ETIMEDOUT) {
                    fprintf(stderr, "[NavThread] Timeout waiting for image capture confirmation for obstacle %d.\n", cmd.value);
                    img_ack_result = -1; // Indicate error
                    break;
                } else if (rc != 0) {
                    fprintf(stderr, "[NavThread] Error waiting for image capture condition variable: %d\n", rc);
                    img_ack_result = -1; // Indicate error
                    break;
                }
            }

            if (img_ack_result == 0 && context->last_image_capture_id == (uint32_t)cmd.value) {
                printf("[NavThread] Received image capture confirmation for obstacle %d. Proceeding.\n", cmd.value);
            } else if (img_ack_result == 0 && context->last_image_capture_id == 0) {
                // This means an image capture failed (last_image_capture_id was set to 0)
                fprintf(stderr, "[NavThread] Image capture for obstacle %d indicated failure. Aborting navigation.\n", cmd.value);
                img_ack_result = -1; // Treat as failure for navigation flow
            }
            pthread_mutex_unlock(&context->image_capture_mutex);

            if (img_ack_result == -1 || context->stop_requested) {
                // If there was an error or stop was requested while waiting, break out of navigation
                pthread_mutex_lock(&context->lock);
                context->stop_requested = true; // Ensure stop state is propagated
                context->state = STATE_IDLE;
                pthread_mutex_unlock(&context->lock);
                break; // Exit the command execution loop
            }

#if ENABLE_BULLSEYE_RECOVERY
            // The capture confirmation above only means the photo was taken; the verdict
            // arrives later. Wait for it here so a bullseye can be acted on while the robot
            // is still parked in front of the obstacle.
            {
                struct timespec ts_rec;
                clock_gettime(CLOCK_REALTIME, &ts_rec);
                ts_rec.tv_sec += RECOGNITION_WAIT_SECONDS;

                pthread_mutex_lock(&context->recognition_mutex);
                while (context->last_recognition_id != (uint32_t)cmd.value && !context->stop_requested) {
                    if (pthread_cond_timedwait(&context->recognition_cond,
                                               &context->recognition_mutex, &ts_rec) != 0) {
                        fprintf(stderr, "[NavThread] No recognition verdict for obstacle %d in time; continuing.\n",
                                cmd.value);
                        break;
                    }
                }
                pthread_mutex_unlock(&context->recognition_mutex);
            }

            bool do_recovery = false;
            pthread_mutex_lock(&context->lock);
            if (context->bullseye_detected && context->bullseye_obstacle_id == cmd.value) {
                context->bullseye_detected = false; // consume the flag either way
                do_recovery = true;
            }
            pthread_mutex_unlock(&context->lock);

            if (do_recovery && !context->stop_requested) {
                printf("[NavThread] Bullseye on obstacle %d -- requesting route to another face.\n",
                       cmd.value);
                if (request_bullseye_recovery(context) == 0) {
                    // A fresh route replaced the old one: restart execution from its first
                    // command. i = -1 because the for-loop increment runs next.
                    i = -1;
                    continue;
                }
                // Recovery unavailable -- fall through and finish the original route.
                fprintf(stderr, "[NavThread] Continuing original route without recovery.\n");
            }
#endif
        } else if (cmd.type == CMD_FINISH) {
            printf("[NavThread] Received FINISH command. Ending navigation.\n");
            break; // Exit the command execution loop
        } else {
            // --- Pre-turn correction for REVERSE turns -----------------------
            // BR90 (REVR) lands ~7cm LONG along the direction the car was
            // reversing -- the firmware cannot tighten it (servo saturated). A
            // forward move BEFORE the arc, while the car still holds its
            // original heading, moves it back along exactly that axis and
            // cancels the overshoot. BL90 is calibrated in firmware; no nudge.
            if (cmd.type == CMD_REVERSE_RIGHT && REVERSE_TURN_PRE_CM > 0) {
                Command pre;
                pre.type = CMD_MOVE_FORWARD;
                pre.value = REVERSE_TURN_PRE_CM;

                uint32_t pre_id = current_cmd_id++;
                printf("[NavThread] BR90 pre-nudge: forward %dcm to cancel REVR overshoot.\n", REVERSE_TURN_PRE_CM);

                if (send_followup_and_wait(context, pre, pre_id) != 0) {
                    fprintf(stderr, "[NavThread] Reverse-turn pre-correction failed. Aborting navigation.\n");
                    pthread_mutex_lock(&context->lock);
                    context->stop_requested = true;
                    context->state = STATE_IDLE;
                    pthread_mutex_unlock(&context->lock);
                    break;
                }
            }

            // Send command to STM32 with a sequential ID
            uint32_t sent_cmd_id = current_cmd_id++; // Store the ID we're sending
            send_command_to_stm32(context->stm32_fd, cmd, sent_cmd_id);
            printf("[NavThread] Sent command %u (Type: %d, Val: %d) to STM32. Waiting for ACK...\n", 
                   sent_cmd_id, cmd.type, cmd.value);

            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 15; // Increased to 15 seconds for robust ACK waiting

            int ack_result = 0; // 0 for success, -1 for error/timeout
            pthread_mutex_lock(&context->stm32_ack_mutex);
            while (context->stm32_last_ack_id != sent_cmd_id && !context->stop_requested) {
                int rc = pthread_cond_timedwait(&context->stm32_ack_cond, &context->stm32_ack_mutex, &ts);
                if (rc == ETIMEDOUT) {
                    fprintf(stderr, "[NavThread] Timeout waiting for ACK for command %u.\n", sent_cmd_id);
                    ack_result = -1; // Indicate error
                    break;
                } else if (rc != 0) {
                    fprintf(stderr, "[NavThread] Error waiting for ACK condition variable: %d\n", rc);
                    ack_result = -1; // Indicate error
                    break;
                }
            }

            if (ack_result == 0 && context->stm32_last_ack_id == sent_cmd_id) {
                printf("[NavThread] Received ACK for command %u.\n", sent_cmd_id);
                
                // Notify Android that the command is complete so the car moves on the UI
                char status_msg[100];
                const char* type_str = "FW";
                if (cmd.type == CMD_MOVE_BACKWARD) type_str = "BW";
                else if (cmd.type == CMD_TURN_LEFT) type_str = "FL";
                else if (cmd.type == CMD_TURN_RIGHT) type_str = "FR";
                else if (cmd.type == CMD_REVERSE_LEFT) type_str = "BL";
                else if (cmd.type == CMD_REVERSE_RIGHT) type_str = "BR";
                
                snprintf(status_msg, sizeof(status_msg), "STM completed %s%d", type_str, cmd.value);
                send_android_ack(context->android_write_fd, status_msg);
            }
            pthread_mutex_unlock(&context->stm32_ack_mutex);

            if (ack_result == -1 || context->stop_requested) {
                // If there was an error or stop was requested while waiting, break out of navigation
                pthread_mutex_lock(&context->lock);
                context->stop_requested = true; // Ensure stop state is propagated
                context->state = STATE_IDLE;
                pthread_mutex_unlock(&context->lock);
                break; // Exit the command execution loop
            }

            // --- Post-turn overshoot correction -----------------------------
            // The STM overshoots laterally on each 90-degree turn (see the
            // TURN_*_CORRECTION_CM notes at the top of this file). Cancel it
            // with an equal reverse in the new heading before continuing.
            int correction_cm = 0;
            CommandType correction_type = CMD_MOVE_BACKWARD;
            if (cmd.type == CMD_TURN_RIGHT) {
                correction_cm = TURN_RIGHT_CORRECTION_CM;
            } else if (cmd.type == CMD_TURN_LEFT) {
                correction_cm = TURN_LEFT_CORRECTION_CM;
            } else if (cmd.type == CMD_REVERSE_LEFT || cmd.type == CMD_REVERSE_RIGHT) {
                // Second half of the diagonal correction, now in the NEW heading.
                correction_cm = REVERSE_TURN_POST_CM;
                correction_type = CMD_MOVE_FORWARD;
            }

            if (correction_cm > 0) {
                Command fix;
                fix.type = correction_type;
                fix.value = correction_cm;

                uint32_t fix_id = current_cmd_id++;
                printf("[NavThread] Correcting turn overshoot: %s %dcm.\n",
                       correction_type == CMD_MOVE_FORWARD ? "forward" : "reversing",
                       correction_cm);

                if (send_followup_and_wait(context, fix, fix_id) != 0) {
                    fprintf(stderr, "[NavThread] Turn correction failed. Aborting navigation.\n");
                    pthread_mutex_lock(&context->lock);
                    context->stop_requested = true;
                    context->state = STATE_IDLE;
                    pthread_mutex_unlock(&context->lock);
                    break;
                }
            }
        }
    } // End of for loop
    // Using send_android_ack for navigation completion status
    send_android_ack(context->android_write_fd, "all-images-scan");
}

// Helper for Task 2 image detection
static int perform_task2_detection(SharedAppContext* context, int obstacle_num) {
    char image_server_response[2048];
    int arrow_dir = 0; // 1 for Left, 2 for Right, 0 for None
    int attempts = 0;
    const int MAX_ATTEMPTS = 5;

    while (attempts < MAX_ATTEMPTS && arrow_dir == 0) {
        attempts++;
        printf("[Task2] Attempt %d/%d: Capturing image for obstacle %d...\n", attempts, MAX_ATTEMPTS, obstacle_num);
        
        if (capture_image(CAPTURE_FILENAME) == 0) {
            if (post_image_to_server_thread(obstacle_num, image_server_response, sizeof(image_server_response)) == 0) {
                printf("[Task2] Attempt %d response: %s\n", attempts, image_server_response);
                int count = 0;
                if (get_json_int(image_server_response, "count", &count) == 0 && count > 0) {
                     // Robustly search for Right Arrow (38) or Left Arrow (39) using JSON helpers
                     int img_id = -1;
                     char class_label[100] = {0};

                     // Try getting img_id directly (handles whitespace after colon)
                     if (get_json_int(image_server_response, "img_id", &img_id) == 0) {
                         if (img_id == 39) arrow_dir = 1;
                         else if (img_id == 38) arrow_dir = 2;
                     }

                     // Fallback to class_label if img_id didn't match or wasn't found
                     if (arrow_dir == 0 && (get_json_string(image_server_response, "class_label", class_label, sizeof(class_label)) == 0 ||
                                          get_json_string(image_server_response, "class", class_label, sizeof(class_label)) == 0)) {
                         if (strstr(class_label, "Left Arrow") != NULL) arrow_dir = 1;
                         else if (strstr(class_label, "Right Arrow") != NULL) arrow_dir = 2;
                     }

                     if (arrow_dir == 1) {
                         printf("[Task2] Detected LEFT arrow for obstacle %d on attempt %d\n", obstacle_num, attempts);
                     } else if (arrow_dir == 2) {
                         printf("[Task2] Detected RIGHT arrow for obstacle %d on attempt %d\n", obstacle_num, attempts);
                     }
                }
            }
        }

        if (arrow_dir == 0 && attempts < MAX_ATTEMPTS) {
            printf("[Task2] No arrow detected on attempt %d. Retrying in 0.5s...\n", attempts);
            usleep(500000); // Wait 0.5 seconds before next attempt
        }
    }

    if (arrow_dir > 0) {
        char stm_cmd[64];
        snprintf(stm_cmd, sizeof(stm_cmd), ":2/GENERAL/CAPTURE/%d/%d;", obstacle_num, arrow_dir);
        write(context->stm32_fd, stm_cmd, strlen(stm_cmd));
        printf("[To STM32]: %s\n", stm_cmd);
        
        char android_msg[128];
        snprintf(android_msg, sizeof(android_msg), "Task 2 Obs %d: %s", obstacle_num, (arrow_dir == 1 ? "Left" : "Right"));
        send_android_ack(context->android_write_fd, android_msg);
        return 1;
    } else {
        printf("[Task2] FAILED to detect arrow for obstacle %d after %d attempts.\n", obstacle_num, MAX_ATTEMPTS);
        send_android_ack(context->android_write_fd, "Error: Task 2 detection failed after 5 attempts.");
        return 0;
    }
}

// Send a LOCK trigger to the PC (stream_test.py listens on PC_LOCK_PORT).
static int send_lock_trigger_to_pc(int obstacle_num) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("[Task2] socket()");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)PC_LOCK_PORT);
    addr.sin_addr.s_addr = inet_addr(PC_LOCK_HOST);

    if (addr.sin_addr.s_addr == INADDR_NONE) {
        fprintf(stderr, "[Task2] Invalid PC_LOCK_HOST: %s\n", PC_LOCK_HOST);
        close(sockfd);
        return -1;
    }

    if (connect(sockfd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("[Task2] connect() to PC lock port failed");
        close(sockfd);
        return -1;
    }

    char msg[32];
    snprintf(msg, sizeof(msg), "LOCK %d\n", obstacle_num);
    if (send(sockfd, msg, strlen(msg), 0) < 0) {
        perror("[Task2] send(LOCK) failed");
        close(sockfd);
        return -1;
    }
    close(sockfd);
    printf("[Task2] Sent trigger to PC: %s", msg);
    return 0;
}

// Wait for the livestream RESULT (img_id) for this obstacle, then send the STM capture command.
static int perform_task2_detection_livestream(SharedAppContext* context, int obstacle_num) {
    // 1) Ask the PC to commit the latest stable arrow for this obstacle.
    if (send_lock_trigger_to_pc(obstacle_num) != 0) {
        send_android_ack(context->android_write_fd, "Task 2: Failed to trigger PC lock.");
        return 0;
    }

    // 2) Wait for RESULT forwarded by stream_server.py -> localhost UDP -> this process.
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 10; // wait up to 10 seconds for PC to respond

    int img_id = -1;
    pthread_mutex_lock(&g_task2_result_mutex);
    while (!g_task2_result_ready[obstacle_num] && !context->stop_requested) {
        int rc = pthread_cond_timedwait(&g_task2_result_cond, &g_task2_result_mutex, &ts);
        if (rc == ETIMEDOUT) {
            fprintf(stderr, "[Task2] Timeout waiting for livestream RESULT for obstacle %d.\n", obstacle_num);
            pthread_mutex_unlock(&g_task2_result_mutex);
            send_android_ack(context->android_write_fd, "Task 2: Timeout waiting for PC detection.");
            return 0;
        }
    }
    img_id = g_task2_result_img_id[obstacle_num];
    g_task2_result_ready[obstacle_num] = 0;
    pthread_mutex_unlock(&g_task2_result_mutex);

    int arrow_dir = 0;
    if (img_id == 39) arrow_dir = 1;       // Left Arrow
    else if (img_id == 38) arrow_dir = 2;  // Right Arrow

    if (arrow_dir == 0) {
        fprintf(stderr, "[Task2] Invalid img_id from livestream for obstacle %d: %d\n", obstacle_num, img_id);
        send_android_ack(context->android_write_fd, "Task 2: Invalid detection result (img_id).");
        return 0;
    }

    char stm_cmd[64];
    snprintf(stm_cmd, sizeof(stm_cmd), ":2/GENERAL/CAPTURE/%d/%d;", obstacle_num, arrow_dir);
    write(context->stm32_fd, stm_cmd, strlen(stm_cmd));
    printf("[To STM32]: %s\n", stm_cmd);
    arrow_dir = 0;

    char android_msg[128];
    snprintf(android_msg, sizeof(android_msg), "Task 2 Obs %d: %s", obstacle_num, (arrow_dir == 1 ? "Left" : "Right"));
    send_android_ack(context->android_write_fd, android_msg);
    return 1;
}

void execute_task2() {
    SharedAppContext* context = &g_app_context;
    printf("[NavThread] State: [TASK2]. Waiting for STM signals.\n");

    for (int obs = 1; obs <= 2; obs++) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 120; // Increased to 120 seconds (2 minutes) for STM to reach obstacle

        pthread_mutex_lock(&context->task2_snap_mutex);
        while (context->task2_snap_obs_id != obs && !context->stop_requested) {
            int rc = pthread_cond_timedwait(&context->task2_snap_cond, &context->task2_snap_mutex, &ts);
            if (rc == ETIMEDOUT) {
                fprintf(stderr, "[NavThread] Task 2: Timeout waiting for snapshot signal !CAPTURE%d from STM32.\n", obs);
                break;
            }
        }
        
        if (context->stop_requested) {
            pthread_mutex_unlock(&context->task2_snap_mutex);
            break;
        }

        if (context->task2_snap_obs_id == obs) {
            context->task2_snap_obs_id = 0; // Reset for next obstacle
            pthread_mutex_unlock(&context->task2_snap_mutex);
            
#if USE_LIVESTREAM_TASK2
            if (perform_task2_detection_livestream(context, obs) == 0) {
#else
            if (perform_task2_detection(context, obs) == 0) {
#endif
                printf("[NavThread] Task 2: Aborting due to detection failure.\n");
                break; // Exit the loop on total failure
            }
        } else {
            pthread_mutex_unlock(&context->task2_snap_mutex);
            break; 
        }
    }

    send_android_ack(context->android_write_fd, "Task 2 completed.");
    printf("[NavThread] Task 2 Finished.\n");
}

void* navigation_executor_thread(void* args) {
    SharedAppContext* context = (SharedAppContext*)args;

    while (1) {
        pthread_mutex_lock(&context->lock);
        while (!context->new_map_received && !context->task2_requested && !context->stop_requested) {
            printf("[NavThread] State: [IDLE]. Waiting for new mission...\n");
            pthread_cond_wait(&context->new_task_cond, &context->lock);
        }
        printf("[NavThread] Woke up! new_map_received=%d, task2_requested=%d, stop_requested=%d\n", 
               context->new_map_received, context->task2_requested, context->stop_requested);

        if (context->stop_requested) {
            context->state = STATE_IDLE;
            context->stop_requested = false;
        }

        if (context->new_map_received) {
            context->state = STATE_PATHFINDING;
            context->new_map_received = false;
            time(&context->mission_start_time); // Record start time for time-based auto-stop
        } else if (context->task2_requested) {
            context->state = STATE_TASK2;
            context->task2_requested = false;
        }
        pthread_mutex_unlock(&context->lock);

        if (context->state == STATE_PATHFINDING) {
            // ... (existing pathfinding logic)
            printf("[NavThread] State: [PATHFINDING]. Requesting route from server...\n");
            char payload[2048];
            char obstacles_str[1500] = ""; // To build the obstacles array string

            bool first_obs = true;
            for (int i = 0; i < context->obstacle_count; i++) {
                if (context->obstacles[i].id == 0) continue; // Skip ID 0 as requested
                char obs_item[100]; // Buffer for a single obstacle JSON object
                if (!first_obs) strcat(obstacles_str, ",");
                // Obstacle x, y are 0-indexed internally, server expects 0-indexed
                // Direction 'd' is integer, server expects integer
                snprintf(obs_item, sizeof(obs_item), "{\"id\":%d,\"x\":%d,\"y\":%d,\"d\":%d}",
                         context->obstacles[i].id, context->obstacles[i].x, context->obstacles[i].y, context->obstacles[i].d);
                strcat(obstacles_str, obs_item);
                first_obs = false;
            }

            // Construct the full payload including robot initial state and retrying flag
            snprintf(payload, sizeof(payload), "{\"obstacles\":[%s],\"robot_x\":%d,\"robot_y\":%d,\"robot_dir\":%d,\"retrying\":false}",
                     obstacles_str, context->robot_start_x, context->robot_start_y, context->robot_start_dir);
            printf("[NavThread] Forwarding arena data to pathfinding server: %s\n", payload);

            char response[4096]; // Increased response buffer size
            if (post_data_to_server(PATHFINDING_SERVER_URL, payload, response, sizeof(response)) == 0) {
                // --- DEBUG: Print raw server response ---
                printf("[NavThread] Raw server response:\n---\n%s\n---\n", response);

                // Call the modified parse_command_route_from_server
                if (parse_command_route_from_server(response, context->commands, &context->command_count,
                                                    context->snap_positions, &context->snap_position_count) == 0) {
                    send_android_ack(context->android_write_fd, "Route calculated. Navigating."); // Using ack send
                    execute_navigation();
                } else {
                    send_android_ack(context->android_write_fd, "Error: Pathfinding failed to parse route."); // Using ack send
                }
            } else {
                send_android_ack(context->android_write_fd, "Error: Pathfinding server communication failed."); // Using ack send
            }
        } else if (context->state == STATE_TASK2) {
            execute_task2();
        }

        pthread_mutex_lock(&context->lock);
        context->state = STATE_IDLE;
        pthread_mutex_unlock(&context->lock);
    }
    return NULL;
}
            
            
            // =================================================================================
            // THREAD 1: Android Listener (High-level Commands)
            // =================================================================================
            
            void* android_listener_thread(void* args) {
                SharedAppContext* context = (SharedAppContext*)args;
                char buffer[8192];      // Temporary read buffer
                char main_buffer[16384]; // Persistent accumulation buffer
                int buffer_pos = 0;
            
                printf("[AndroidThread] Started listening for messages.\n");
                while (1) {
                    ssize_t bytes_read = read(context->android_fd, buffer, sizeof(buffer) - 1);
            
                    if (bytes_read > 0) {
                        buffer[bytes_read] = '\0';
                        printf("[AndroidThread] Incoming data (%zd bytes): %s\n", bytes_read, buffer);

                        // Append new data to main buffer
                        if (buffer_pos + bytes_read >= (ssize_t)sizeof(main_buffer)) {
                            fprintf(stderr, "[AndroidThread] Buffer full. Clearing to recover.\n");
                            buffer_pos = 0;
                        }
                        memcpy(main_buffer + buffer_pos, buffer, bytes_read);
                        buffer_pos += bytes_read;
                        main_buffer[buffer_pos] = '\0';
            
                        // Self-healing: If we have multiple "{"cat":" in the buffer, 
                        // it means we might have a stuck partial message at the front.
                        char* first_msg = strstr(main_buffer, "{\"cat\":");
                        if (first_msg) {
                            char* second_msg = strstr(first_msg + 1, "{\"cat\":");
                            if (second_msg) {
                                // Check if the first one is actually incomplete (no matching })
                                // by seeing if there's a } before the second_msg starts
                                bool has_closing = false;
                                for (char* t = first_msg; t < second_msg; t++) {
                                    if (*t == '}') { has_closing = true; break; }
                                }
                                
                                if (!has_closing) {
                                    printf("[AndroidThread] Detected stuck partial message. Discarding junk at front.\n");
                                    int skip = second_msg - main_buffer;
                                    memmove(main_buffer, second_msg, buffer_pos - skip);
                                    buffer_pos -= skip;
                                    main_buffer[buffer_pos] = '\0';
                                }
                            }
                        }

                        // Process all complete JSON messages
                        char* msg_start;
                        while ((msg_start = strchr(main_buffer, '{')) != NULL) {
                            int depth = 0;
                            char* p = msg_start;
                            char* msg_end = NULL;
                            bool in_string = false;

                            while (*p) {
                                if (*p == '"' && (p == msg_start || *(p-1) != '\\')) {
                                    in_string = !in_string;
                                } else if (!in_string) {
                                    if (*p == '{') depth++;
                                    else if (*p == '}') {
                                        depth--;
                                        if (depth == 0) {
                                            msg_end = p;
                                            break;
                                        }
                                    }
                                }
                                p++;
                            }

                            if (msg_end == NULL) {
                                // Incomplete message, wait for more data
                                break;
                            }

                            // We have a complete message from msg_start to msg_end
                            size_t msg_len = msg_end - msg_start + 1;
                            char current_msg[8192]; 
                            if (msg_len >= sizeof(current_msg)) msg_len = sizeof(current_msg) - 1;
                            memcpy(current_msg, msg_start, msg_len);
                            current_msg[msg_len] = '\0';

                            printf("[AndroidThread] Processing complete JSON: %s\n", current_msg);
            
                                // Check for JSON message first
                                char category[50];
                                if (get_json_string(current_msg, "cat", category, sizeof(category)) == 0) {
                                    if (strcmp(category, "sendArena") == 0) {
                                        const char* value_ptr = strstr(current_msg, "\"value\":");
                                        if (value_ptr) {
                                            const char* map_json_start = strchr(value_ptr, '{');
                                            if (map_json_start) {
                                                pthread_mutex_lock(&context->lock);
                                                if (context->state == STATE_IDLE) {
                                                    if (parse_android_map_and_obstacles(map_json_start, context) == 0) {
                                                        context->new_map_received = true;
                                                        printf("[AndroidThread] Valid mission received. Obstacles: %d. Signalling NavThread.\n", context->obstacle_count);
                                                        send_android_ack(context->android_write_fd, "Map received. Pathfinding...");
                                                        pthread_cond_signal(&context->new_task_cond);
                                                    } else {
                                                        send_android_ack(context->android_write_fd, "Error: Invalid map format.");
                                                    }
                                                } else {
                                                    send_android_ack(context->android_write_fd, "Error: Robot is busy. Cannot start new mission.");
                                                }
                                                pthread_mutex_unlock(&context->lock);
                                            } else {
                                                fprintf(stderr, "[AndroidThread] Malformed 'sendArena': 'value' object not found.\n");
                                                send_android_ack(context->android_write_fd, "Error: Malformed 'sendArena' message.");
                                            }
                                        } else {
                                            fprintf(stderr, "[AndroidThread] Malformed 'sendArena': 'value' key not found.\n");
                                            send_android_ack(context->android_write_fd, "Error: Malformed 'sendArena' message.");
                                        }
                                    } else if (strcmp(category, "stop") == 0) { // STOP command as JSON
                                        pthread_mutex_lock(&context->lock);
                                        send_android_ack(context->android_write_fd, "STOP command received.");
                                        context->stop_requested = true;
                                        // Flush STM32 port to clear pending commands
                                        flush_serial_port(context->stm32_fd);
                                        if(context->state != STATE_IDLE) {
                                            pthread_cond_signal(&context->new_task_cond);
                                        }
                                        pthread_mutex_unlock(&context->lock);
                                    } else if (strcmp(category, "task2") == 0) { // New Task 2 command
                                        pthread_mutex_lock(&context->lock);
                                        if (context->state == STATE_IDLE) {
                                            context->state = STATE_TASK2;
                                            context->task2_requested = true;
                                            printf("[AndroidThread] Task 2 mission received. Signalling NavThread.\n");
                                            
                                            // Send initialization command to STM32 for Task 2
                                            char task2_init_cmd[64];
                                            snprintf(task2_init_cmd, sizeof(task2_init_cmd), ":1/MOTOR/TASK2/1/1;");
                                            write(context->stm32_fd, task2_init_cmd, strlen(task2_init_cmd));
                                            printf("[To STM32]: %s\n", task2_init_cmd);

                                            send_android_ack(context->android_write_fd, "Task 2 started.");
                                            pthread_cond_signal(&context->new_task_cond);
                                        } else {
                                            send_android_ack(context->android_write_fd, "Error: Robot busy, cannot start Task 2.");
                                        }
                                        pthread_mutex_unlock(&context->lock);
                                    } else if (strcmp(category, "stm") == 0) { // Direct STM command from Android
                                        char stm_command_str[100]; // Buffer for the command string like "<FR090>"
                                        if (get_json_string(current_msg, "value", stm_command_str, sizeof(stm_command_str)) == 0) {
                                            parse_and_execute_android_command(context->stm32_fd, stm_command_str, context);
                                        } else {
                                            fprintf(stderr, "[AndroidThread] Malformed 'stm' command: 'value' key not found.\n");
                                            send_android_ack(context->android_write_fd, "Error: Malformed STM command.");
                                        }
                                    } else {
                                        fprintf(stderr, "[AndroidThread] Unrecognized JSON category: %s\n", category);
                                    }
                                } else {
                                    fprintf(stderr, "[AndroidThread] Malformed or unrecognized message: %s\n", current_msg);
                                }

                            // Shift remaining data to the front
                            // Also consume any trailing whitespace/newlines after the JSON object
                            char* next_ptr = msg_end + 1;
                            while (*next_ptr && (isspace((unsigned char)*next_ptr) || *next_ptr == '\n' || *next_ptr == '\r')) {
                                next_ptr++;
                            }

                            int consumed = next_ptr - main_buffer;
                            int remaining = buffer_pos - consumed;
                            if (remaining > 0) {
                                memmove(main_buffer, next_ptr, remaining);
                                buffer_pos = remaining;
                                main_buffer[buffer_pos] = '\0';
                            } else {
                                buffer_pos = 0;
                                main_buffer[0] = '\0';
                            }
                        }
                    } else if (bytes_read == 0) {
                        usleep(10000);
                    } else {
                        // Avoid spamming perror if it's just no data available in non-blocking mode
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            perror("[AndroidThread] Error reading from serial port");
                        }
                        usleep(10000);
                    }
                }
                return NULL;
            }
            
            
            // =================================================================================
            // New THREAD: STM32 Listener
            // =================================================================================
            void* stm32_listener_thread(void* args) {
                SharedAppContext* context = (SharedAppContext*)args;
                char buffer[256];      // Temporary read buffer
                char main_buffer[512]; // Persistent accumulation buffer
                int buffer_pos = 0;
            
                printf("[STM32Thread] Started listening for messages.\n");
            
                while (1) {
                    ssize_t bytes_read = read(g_stm32_ack_fd, buffer, sizeof(buffer) - 1);
            
                                if (bytes_read > 0) {
                                    buffer[bytes_read] = '\0';
                                    printf("[STM32Thread] Incoming raw: %s\n", buffer);
                    
                                    // Append new data to main buffer
                                    if (buffer_pos + bytes_read >= (ssize_t)sizeof(main_buffer)) {                            fprintf(stderr, "[STM32Thread] Buffer overflow. Clearing buffer.\n");
                            buffer_pos = 0;
                        }
                        memcpy(main_buffer + buffer_pos, buffer, bytes_read);
                        buffer_pos += bytes_read;
                        main_buffer[buffer_pos] = '\0';
            
                        // Process all complete messages (delimited by ;)
                        char* semicolon_pos;
                        while ((semicolon_pos = strchr(main_buffer, ';')) != NULL) {
                            size_t msg_len = (semicolon_pos - main_buffer) + 1;
                            char current_msg[256];
                            if (msg_len >= sizeof(current_msg)) msg_len = sizeof(current_msg) - 1;
                            
                            memcpy(current_msg, main_buffer, msg_len);
                            current_msg[msg_len] = '\0';
            
                            printf("[STM32Thread] Message: %s\n", current_msg);
            
                            uint32_t cmd_id;
                            // The STM may print debug text on the same UART, so the ACK can arrive
                            // with junk in front of it. Find the last '!' and parse the ACK from there.
                            const char* ack_start = strrchr(current_msg, '!');
                            if (ack_start && sscanf(ack_start, "!%u/DONE;", &cmd_id) == 1) {
                                pthread_mutex_lock(&context->stm32_ack_mutex);
                                context->stm32_last_ack_id = cmd_id;
                                pthread_cond_signal(&context->stm32_ack_cond);
                                pthread_mutex_unlock(&context->stm32_ack_mutex);
                                printf("[STM32Thread] ACK confirmed for ID: %u\n", cmd_id);
                            } else if (strstr(current_msg, "!CAPTURE1;") != NULL) {
                                printf("[STM32Thread] Snapshot request 1 received from STM32.\n");
                                pthread_mutex_lock(&context->task2_snap_mutex);
                                context->task2_snap_obs_id = 1;
                                pthread_cond_signal(&context->task2_snap_cond);
                                pthread_mutex_unlock(&context->task2_snap_mutex);
                            } else if (strstr(current_msg, "!CAPTURE2;") != NULL) {
                                printf("[STM32Thread] Snapshot request 2 received from STM32.\n");
                                pthread_mutex_lock(&context->task2_snap_mutex);
                                context->task2_snap_obs_id = 2;
                                pthread_cond_signal(&context->task2_snap_cond);
                                pthread_mutex_unlock(&context->task2_snap_mutex);
                            }
            
                            // Shift remaining data to the front
                            int remaining = buffer_pos - msg_len;
                            if (remaining > 0) {
                                memmove(main_buffer, main_buffer + msg_len, remaining);
                                buffer_pos = remaining;
                                main_buffer[buffer_pos] = '\0';
                            } else {
                                buffer_pos = 0;
                                main_buffer[0] = '\0';
                            }
                        }
                    } else if (bytes_read == 0) {
                        usleep(10000);
                    } else {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            perror("[STM32Thread] Error reading from serial port");
                        }
                        usleep(10000);
                    }
                }
                return NULL;
            }

// =================================================================================
// New THREAD: Livestream RESULT listener (UDP from stream_server.py)
// =================================================================================
static void* livestream_result_listener_thread(void* args) {
    (void)args;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("[LiveResult] socket()");
        return NULL;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)RESULT_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only

    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        perror("[LiveResult] bind()");
        close(sockfd);
        return NULL;
    }

    printf("[LiveResult] Listening on 127.0.0.1:%d for RESULT JSON.\n", RESULT_UDP_PORT);

    char buf[4096];
    while (1) {
        ssize_t n = recv(sockfd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) {
            usleep(10000);
            continue;
        }
        buf[n] = '\0';

        int img_id = -1;
        char obj_str[32] = {0};
        int obstacle_num = 0;

        // object_id is a string in our payload; parse it and img_id.
        if (get_json_string(buf, "object_id", obj_str, sizeof(obj_str)) == 0) {
            obstacle_num = atoi(obj_str);
        }
        // get_json_int finds first "img_id" occurrence (works for our payload)
        (void)get_json_int(buf, "img_id", &img_id);

        if (obstacle_num < 1 || obstacle_num > 2 || img_id < 0) {
            printf("[LiveResult] Ignored payload (bad object_id/img_id). obj=%d img_id=%d\n", obstacle_num, img_id);
            continue;
        }

        pthread_mutex_lock(&g_task2_result_mutex);
        g_task2_result_img_id[obstacle_num] = img_id;
        g_task2_result_ready[obstacle_num] = 1;
        pthread_cond_broadcast(&g_task2_result_cond);
        pthread_mutex_unlock(&g_task2_result_mutex);

        printf("[LiveResult] Stored RESULT for obs %d: img_id=%d\n", obstacle_num, img_id);
    }
    // unreachable
    // close(sockfd);
    // return NULL;
}
            
            
            // =================================================================================
            // Main Function (Initialization and Thread Management)
            // =================================================================================

// =================================================================================
// Main Function (Initialization and Thread Management)
// =================================================================================

int main() {
    // Register signal handler for clean exit on Ctrl+C
    signal(SIGINT, handle_sigint);

    // Disable stdout buffering to ensure logs are printed immediately from all threads
    setvbuf(stdout, NULL, _IONBF, 0);

    curl_global_init(CURL_GLOBAL_ALL); // Initialize curl once for the application lifecycle
    memset(&g_app_context, 0, sizeof(SharedAppContext));
    pthread_mutex_init(&g_app_context.lock, NULL);
    pthread_cond_init(&g_app_context.new_task_cond, NULL);
    g_app_context.state = STATE_IDLE;
    g_app_context.snap_position_count = 0; // Initialize new fields
    g_app_context.snap_position_idx = 0;   // Initialize new fields

    // Initialize STM32 ACK synchronization mechanisms
    g_app_context.stm32_last_ack_id = 0;
    pthread_mutex_init(&g_app_context.stm32_ack_mutex, NULL);
    pthread_cond_init(&g_app_context.stm32_ack_cond, NULL);

    // Initialize Image capture synchronization mechanisms
    g_app_context.last_image_capture_id = 0;
    pthread_mutex_init(&g_app_context.image_capture_mutex, NULL);
    pthread_cond_init(&g_app_context.image_capture_cond, NULL);

    // Initialize recognition-verdict synchronization (used by bullseye recovery)
    g_app_context.last_recognition_id = 0;
    pthread_mutex_init(&g_app_context.recognition_mutex, NULL);
    pthread_cond_init(&g_app_context.recognition_cond, NULL);

    // Initialize Task 2 synchronization mechanisms
    g_app_context.task2_requested = false;
    g_app_context.task2_snap_obs_id = 0;
    pthread_mutex_init(&g_app_context.task2_snap_mutex, NULL);
    pthread_cond_init(&g_app_context.task2_snap_cond, NULL);


    // Initialize Android Communication
    #if ANDROID_SIM
        printf("Android: Simulation Mode (Pipes)\n");
        g_app_context.android_fd = init_serial_port(ANDROID_PIPE_READ, BAUD_RATE);
        g_app_context.android_write_fd = init_serial_port(ANDROID_PIPE_WRITE, BAUD_RATE);
    #else
        printf("Android: Hardware Mode (%s)\n", ANDROID_HW_DEVICE);
        g_app_context.android_fd = init_serial_port(ANDROID_HW_DEVICE, BAUD_RATE);
        g_app_context.android_write_fd = g_app_context.android_fd;
    #endif

    // Initialize STM32 Communication
    #if STM32_SIM
        printf("STM32:   Simulation Mode (Pipes)\n");
        g_app_context.stm32_fd = init_serial_port(STM32_PIPE_WRITE, BAUD_RATE);
        g_stm32_ack_fd = init_serial_port(STM32_PIPE_READ, BAUD_RATE);
    #else
        printf("STM32:   Hardware Mode (%s)\n", STM32_HW_DEVICE);
        g_app_context.stm32_fd = init_serial_port(STM32_HW_DEVICE, BAUD_RATE);
        g_stm32_ack_fd = g_app_context.stm32_fd;
    #endif

    if (g_app_context.stm32_fd == -1 || g_stm32_ack_fd == -1 || g_app_context.android_fd == -1 || g_app_context.android_write_fd == -1) {
        fprintf(stderr, "Fatal: Failed to initialize communication ports. Exiting.\n");
        return 1;
    }

    // Perform initial flush to clear any stale data from previous runs (STM32 only)
    flush_serial_port(g_app_context.stm32_fd);
    if (g_stm32_ack_fd != g_app_context.stm32_fd) flush_serial_port(g_stm32_ack_fd);

    printf("--- RPi Control Centre Initialized ---\n");

    pthread_t android_tid, nav_tid, stm32_tid;
    pthread_create(&android_tid, NULL, android_listener_thread, &g_app_context);
    pthread_create(&nav_tid, NULL, navigation_executor_thread, &g_app_context);
    pthread_create(&stm32_tid, NULL, stm32_listener_thread, &g_app_context); // Create the new STM32 listener thread
    pthread_t live_tid;
    pthread_create(&live_tid, NULL, livestream_result_listener_thread, NULL);

    // Signal Android that we are ready
    send_android_ack(g_app_context.android_write_fd, "RPi-Ready");

    pthread_join(android_tid, NULL);
    pthread_join(nav_tid, NULL);
    pthread_join(stm32_tid, NULL); // Join the new STM32 listener thread
    pthread_join(live_tid, NULL);

    pthread_mutex_destroy(&g_app_context.lock);
    pthread_cond_destroy(&g_app_context.new_task_cond);
    pthread_mutex_destroy(&g_app_context.stm32_ack_mutex); // Destroy new mutex
    pthread_cond_destroy(&g_app_context.stm32_ack_cond);   // Destroy new condition variable
    pthread_mutex_destroy(&g_app_context.image_capture_mutex); // Destroy image capture mutex
    pthread_cond_destroy(&g_app_context.image_capture_cond);   // Destroy image capture condition variable
    
    // Close file descriptors
    if (g_stm32_ack_fd != -1) close(g_stm32_ack_fd);
    if (g_app_context.stm32_fd != -1 && g_app_context.stm32_fd != g_stm32_ack_fd) {
        close(g_app_context.stm32_fd);
    }
    
    if (g_app_context.android_fd != -1) close(g_app_context.android_fd);
    if (g_app_context.android_write_fd != -1 && g_app_context.android_write_fd != g_app_context.android_fd) {
        close(g_app_context.android_write_fd);
    }


    curl_global_cleanup(); // Clean up curl once at application shutdown
    return 0;
}


// =================================================================================
// Testing Instructions (Simulation Mode)
// =================================================================================
/*
To test the RPi Control Centre without physical hardware (Android/STM32), follow these steps.
The simulation uses Named Pipes (FIFOs) for serial comms and the provided Python fake servers.

**Prerequisites:**
1.  Python 3 installed.
2.  libcurl development libraries (e.g., `sudo apt-get install libcurl4-openssl-dev`).
3.  Ensure all source files (multithread_communication.c, json_parser.c, rpi_hal.c) are present.

**Step 1: Compile the RPI communication module**
Use the provided Makefile to build the test configuration:

    make test_center

This creates an executable `./test_center` with STM32_SIM=1, ANDROID_SIM=1, and RPI_TESTING defined.

**Step 2: Create Named Pipes (FIFOs)**
Create the 4 pipes required for bidirectional simulation in the `RPI` directory:

    mkfifo rpi_to_stm stm_to_rpi android_to_rpi rpi_to_android

**Step 3: Run the Fake Servers and STM32 Simulator**
Open 3 separate terminals for the simulators:

*   Terminal 1 (Fake Pathfinding Server):
    python3 fake_path_server.py

*   Terminal 2 (Fake Image Recognition Server):
    python3 fake_image_server.py

*   Terminal 3 (Fake STM32 Simulation):
    python3 fake_stm.py

*Note: Ensure the URLs in `multithread_communication.c` (PATHFINDING_SERVER_URL, IMAGE_SERVER_URL) 
point to where these servers are running (e.g., "http://localhost:5000").*

**Step 4: Run the RPI Control Centre**
Open a 4th terminal and run the compiled program:

    ./test_center

**Step 5: Simulate Android Input**
Open a 5th terminal. Send a "sendArena" command as JSON to the `android_to_rpi` pipe.

Example Command (Start Mission):
    echo "{\"cat\": \"sendArena\", \"value\": {\"obstacles\":[{\"x\": 10,\"y\": 10,\"d\": 1,\"id\": 1},{\"x\": 5,\"y\": 15,\"d\": 2,\"id\": 2}],\"robot_x\": 2,\"robot_y\": 2,\"robot_dir\": 1}}" > android_to_rpi

Example Command (Direct STM control):
    echo "{\"cat\": \"stm\", \"value\": \"<FW10>\"}" > android_to_rpi

Example Command (Stop):
    echo "{\"cat\": \"stop\"}" > android_to_rpi

*Note: 
- Coordinates (x, y) are 1-indexed for Android; Directions: 1=N, 2=E, 3=S, 4=W.
- Category "stm" expects "<CMDVALUE>" format (e.g., <FW10>, <TL90>).*

**Step 6: Observe and Control**
*   **Monitor STM32 output:** Check Terminal 3 or run `cat rpi_to_stm`.
*   **Monitor Android feedback:** Run `cat rpi_to_android` in a separate terminal to see status Acks and Image-Rec results sent by the RPi.

**Cleanup:**
1.  Press Ctrl+C in all terminals.
2.  Remove pipes: `rm rpi_to_stm stm_to_rpi android_to_rpi rpi_to_android`
*/