#include "tracker.h"
#include <stdio.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"

static inline int16_t myMin(const int16_t value1, const int16_t value2)
{
    if (value1 < value2)
    {
        return value1;
    }
    return value2;
}

static inline int16_t myMax(const int16_t value1, const int16_t value2)
{
    if (value1 > value2)
    {
        return value1;
    }
    return value2;
}

static void IRAM_ATTR boxblur_sat(const camera_fb_t *restrict fb, const target_box *restrict target_box, uint32_t *restrict sat_buffer, uint8_t *restrict blur_buffer)
{
    size_t w = fb->width;
    size_t h = fb->height;

    // Maximum reach of brief test pixels
    const size_t brief_radius = (BRIEF_PATCH_SIZE - 1) / 2;

    // Minimum and maximum coords of fb with respect to kernel size
    size_t kx_min = (target_box->x_min > KERNEL_RADIUS + brief_radius) ? target_box->x_min - KERNEL_RADIUS - brief_radius : 0;
    size_t ky_min = (target_box->y_min > KERNEL_RADIUS + brief_radius) ? target_box->y_min - KERNEL_RADIUS - brief_radius : 0;
    size_t kx_max = (target_box->x_max + KERNEL_RADIUS + brief_radius < w) ? target_box->x_max + KERNEL_RADIUS + brief_radius : w;
    size_t ky_max = (target_box->y_max + KERNEL_RADIUS + brief_radius < h) ? target_box->y_max + KERNEL_RADIUS + brief_radius : h;

    // Dimensions of sat buffer
    size_t sat_w = kx_max - kx_min;
    size_t sat_h = ky_max - ky_min;

    if (sat_w == 0 || sat_h == 0 || sat_w > w || sat_h > h)
    {
        return;
    }

    // Acquire the summed area table to the buffer
    /*
    ----------
    | D |  B |
    ----------
    | C |  A |
    ----------
    I(A) = I(C) + I(B) - I(D) + A
    */

    for (size_t y = ky_min; y < ky_max; y++)
    {
        size_t row_fb = y * w;
        size_t row_sat = (y - ky_min) * sat_w;
        for (size_t x = kx_min; x < kx_max; x++)
        {
            // Use int to avoid overflow
            int idx_fb = x + row_fb;
            int x_offset = x - kx_min;
            int idx_sat = x_offset + row_sat;
            uint32_t acc = fb->buf[idx_fb];
            // Add above
            if (y > ky_min)
            {
                acc += sat_buffer[idx_sat - sat_w];
            }
            // Add left
            if (x > kx_min)
            {
                acc += sat_buffer[idx_sat - 1];
            }
            // Subtract diagonal
            if (x > kx_min && y > ky_min)
            {
                acc -= sat_buffer[idx_sat - sat_w - 1];
            }
            sat_buffer[idx_sat] = acc;
        }
    }

    /*
    ----------
    | D |  B |
    ----------
    | C |  A |
    ----------
    A = I(A) - I(B) - I(C) + I(D)
    */

    for (size_t y = target_box->y_min; y < target_box->y_max; y++)
    {
        size_t y0 = (y > KERNEL_RADIUS) ? y - KERNEL_RADIUS : ky_min;
        size_t y1 = (y + KERNEL_RADIUS < ky_max) ? y + KERNEL_RADIUS : ky_max - 1;
        for (size_t x = target_box->x_min; x < target_box->x_max; x++)
        {
            size_t x0 = (x > KERNEL_RADIUS) ? x - KERNEL_RADIUS : kx_min;
            size_t x1 = (x + KERNEL_RADIUS < kx_max) ? x + KERNEL_RADIUS : kx_max - 1;

            uint32_t A = sat_buffer[(x1 - kx_min) + (y1 - ky_min) * sat_w];

            uint32_t B = 0;
            if (y0 > ky_min)
                B = sat_buffer[(x1 - kx_min) + (y0 - 1 - ky_min) * sat_w];

            uint32_t C = 0;
            if (x0 > kx_min)
                C = sat_buffer[(x0 - 1 - kx_min) + (y1 - ky_min) * sat_w];

            uint32_t D = 0;
            if (x0 > kx_min && y0 > ky_min)
                D = sat_buffer[(x0 - 1 - kx_min) + (y0 - 1 - ky_min) * sat_w];

            // SAT formula
            uint32_t sum = A - B - C + D;

            // Divide by area
            size_t area = (x1 - x0 + 1) * (y1 - y0 + 1);
            if (area > 0)
            {
                size_t fb_index = x + y * w;
                blur_buffer[fb_index] = (uint8_t)(sum / area);
            }
        }
    }
}

static bool IRAM_ATTR pixel_calc_fast(const camera_fb_t *restrict fb, const uint8_t threshold, const uint16_t x, const uint16_t y, const uint8_t pixel_value)

{
    size_t width = fb->width;
    uint8_t dark_streak = 0;
    uint8_t bright_streak = 0;

    // Special case test
    uint8_t n = 12;

    for (uint8_t offset_index = 0; offset_index < 16 + n; offset_index++)
    {
        uint16_t circle_x = x + FAST_OFFSET_X[offset_index % 16];
        uint16_t circle_y = y + FAST_OFFSET_Y[offset_index % 16];
        int32_t circle_index = circle_x + circle_y * (int32_t)width;

        if (circle_index < 0 || circle_index >= fb->len)
        {
            dark_streak = 0;
            bright_streak = 0;
            continue;
        }

        uint8_t circle_value = fb->buf[circle_index];

        if (circle_value > pixel_value + threshold)
        {
            dark_streak++;
            bright_streak = 0;
        }
        else if (circle_value < pixel_value - threshold)
        {
            bright_streak++;
            dark_streak = 0;
        }
        else
        {
            dark_streak = 0;
            bright_streak = 0;
        }

        if (bright_streak >= n || dark_streak >= n)
        {
            return true;
        }
    }

    return false;
}

static void IRAM_ATTR pixel_pretest_fast(const camera_fb_t *restrict fb, const uint8_t threshold, const target_box *restrict target_box, feature_vector *restrict features, const uint16_t x, const uint16_t y)
{

    size_t width = fb->width;
    size_t height = fb->height;
    uint8_t half_patch = BRIEF_PATCH_SIZE / 2;

    if (x < half_patch || x > width - half_patch || y < half_patch || y > height - half_patch)
    {
        return;
    }

    uint32_t pixel_index = x + y * (int32_t)width;
    uint8_t pixel_value = fb->buf[pixel_index];
    uint8_t brighter_count = 0;

    int32_t circle_index = x + FAST_OFFSET_X[0] + ((y + FAST_OFFSET_Y[0]) * (int32_t)width); // index of the 1st element
    uint8_t circle_value = fb->buf[circle_index];

    //  Exam pixel 1
    if (abs(circle_value - pixel_value) > threshold)
    {
        brighter_count++;
    }

    circle_index = x + FAST_OFFSET_X[8] + ((y + FAST_OFFSET_Y[8]) * (int32_t)width); // index of the 9th element
    circle_value = fb->buf[circle_index];

    //  Exam pixel 9
    if (abs(circle_value - pixel_value) > threshold)
    {
        brighter_count++;
    }

    if (brighter_count == 0)
    {
        // Not a feature
        return;
    }

    circle_index = x + FAST_OFFSET_X[12] + ((y + FAST_OFFSET_Y[12]) * (int32_t)width); // index of the 13th element
    circle_value = fb->buf[circle_index];

    //  Exam pixel 13
    if (abs(circle_value - pixel_value) > threshold)
    {
        brighter_count++;
    }

    if (brighter_count >= 2)
    {

        if (brighter_count == 2)
        {
            circle_index = x + FAST_OFFSET_X[4] + ((y + FAST_OFFSET_Y[4]) * (int32_t)width); // index of the 5th element
            circle_value = fb->buf[circle_index];

            // Exam pixel 5
            if (abs(circle_value - pixel_value) > threshold)
            {
                brighter_count++;
            }
        }

        if (brighter_count == 3)
        {
            // Full criterion test to the pixel
            if (pixel_calc_fast(fb, threshold, x, y, pixel_value))
            {
                if (features->count >= features->size)
                {
                    return;
                }
                // Is a feature
                feature feat;
                feat.x = x;
                feat.y = y;
                features->feature_arr[features->count] = feat;
                features->count++;
            }
            // Not a feature
            return;
        }
        else
        {
            // Not a feature
            return;
        }
    }
    else if (brighter_count < 2)
    {
        bool pixel_5_examined = false;

        if (brighter_count == 1)
        {
            circle_index = x + FAST_OFFSET_X[4] + ((y + FAST_OFFSET_Y[4]) * (int32_t)width); // index of the 5th element
            circle_value = fb->buf[circle_index];

            // Exam pixel 5
            if (abs(circle_value - pixel_value) > threshold)
            {
                brighter_count++;
            }
            pixel_5_examined = true;
        }

        if ((brighter_count == 0 && !pixel_5_examined) || (brighter_count == 1 && pixel_5_examined))
        {
            // Full criterion test to the pixel
            if (pixel_calc_fast(fb, threshold, x, y, pixel_value))
            {
                if (features->count >= features->size)
                {
                    return;
                }
                // Is a feature
                feature feat;
                feat.x = x;
                feat.y = y;
                features->feature_arr[features->count] = feat;
                features->count++;
            }
            // Not a feature
            return;
        }
        else
        {
            // Not a feature
            return;
        }
    }
    else
    {
        // Not a feature
        return;
    }
}

static void IRAM_ATTR fast_from_middle(const camera_fb_t *fb, const uint8_t threshold, const target_box *target_box, feature_vector *features)
{
    features->count = 0;

    uint16_t middle_x = (target_box->x_max + target_box->x_min) / 2;
    uint16_t middle_y = (target_box->y_max + target_box->y_min) / 2;

    pixel_pretest_fast(fb, threshold, target_box, features, middle_x, middle_y);

    uint16_t r = 1;
    while (1)
    {
        if (middle_x - r < target_box->x_min && middle_y - r < target_box->y_min)
        {
            break;
        }

        if (middle_x + r > target_box->x_max && middle_y + r > target_box->y_max)
        {
            break;
        }

        uint16_t min_x = myMax((uint16_t)target_box->x_min + 1, middle_x - r);
        uint16_t max_x = myMin((uint16_t)target_box->x_max - 1, middle_x + r);
        uint16_t min_y = myMax((uint16_t)target_box->y_min, middle_y - r);
        uint16_t max_y = myMin((uint16_t)target_box->y_max, middle_y + r);

        // X Sweep upper row
        for (int x = min_x; x <= max_x && min_y > target_box->y_min; x++)
        {
            pixel_pretest_fast(fb, threshold, target_box, features, x, min_y);
        }

        // X Sweep lower row
        for (int x = min_x; x <= max_x && max_y < target_box->y_max; x++)
        {
            pixel_pretest_fast(fb, threshold, target_box, features, x, max_y);
        }

        min_x = myMax((uint16_t)target_box->x_min, middle_x - r);
        max_x = myMin((uint16_t)target_box->x_max, middle_x + r);

        // Y Sweep left column
        for (int y = min_y + 1; y < max_y && min_x > target_box->x_min; y++)
        {
            pixel_pretest_fast(fb, threshold, target_box, features, min_x, y);
        }

        // Y Sweep right column
        for (int y = min_y + 1; y < max_y && max_x < target_box->x_max; y++)
        {
            pixel_pretest_fast(fb, threshold, target_box, features, max_x, y);
        }

        if (features->count >= features->size)
        {
            return;
        }

        r++;
    }
}

static void IRAM_ATTR brief_tracking(const camera_fb_t *restrict fb, feature_vector *restrict features, const uint8_t *restrict blur_buffer)
{

    for (uint32_t i = 0; i < features->count; i++)
    {
        for (uint8_t sample_index = 0; sample_index < TRACK_BRIEF_SIZE; sample_index++)
        {

            uint16_t p0_x = features->feature_arr[i].x + BRIEF_OFFSET[sample_index][0];
            uint16_t p0_y = features->feature_arr[i].y + BRIEF_OFFSET[sample_index][1];
            uint16_t p1_x = features->feature_arr[i].x + BRIEF_OFFSET[sample_index][2];
            uint16_t p1_y = features->feature_arr[i].y + BRIEF_OFFSET[sample_index][3];

            uint32_t p0_index = p0_x + p0_y * (int32_t)fb->width;
            uint32_t p1_index = p1_x + p1_y * (int32_t)fb->width;

            uint8_t p0_value = blur_buffer[p0_index];
            uint8_t p1_value = blur_buffer[p1_index];

            uint8_t byte_index = sample_index / 8;
            uint8_t bit_index = sample_index % 8;

            // Set new byte to 0
            if (bit_index == 0)
            {
                features->feature_arr[i].descriptor[byte_index] = 0;
            }
            // Shift the byte before adding new bit
            features->feature_arr[i].descriptor[byte_index] = features->feature_arr[i].descriptor[byte_index] << 1;
            // OR a 1 to the beginning of the byte if p0 > p1
            if (p0_value > p1_value)
            {
                features->feature_arr[i].descriptor[byte_index] |= 1;
            }
        }
    }
}

static uint8_t IRAM_ATTR hamming_tracking(const feature *restrict feature1, const feature *restrict feature2)
{
    uint8_t result = 0;

    uint32_t *a0 = (uint32_t *)feature1->descriptor;
    uint32_t *b0 = (uint32_t *)feature2->descriptor;

    for (int i = 0; i < 4; i++)
    {
        result += __builtin_popcount(a0[i] ^ b0[i]);
    }

    return result;
}

static void IRAM_ATTR match_tracking(feature_vector *restrict new_features, feature_vector *restrict old_features, feature_match_vector *restrict matches, const uint8_t match_threshold)
{

    uint32_t match_count = 0;

    for (uint32_t i = 0; i < new_features->count; i++)
    {

        if (match_count >= matches->size || old_features->count <= 0)
        {
            break;
        }

        uint8_t min_distance = TRACK_BRIEF_SIZE;
        uint32_t match_index = 0;

        for (uint32_t j = 0; j < old_features->count; j++)
        {
            uint8_t distance = hamming_tracking(&new_features->feature_arr[i], &old_features->feature_arr[j]);

            if (distance < min_distance)
            {
                min_distance = distance;
                match_index = j;
            }
        }

        if (min_distance < match_threshold)
        {
            matches->match_arr[match_count].distance = min_distance;
            matches->match_arr[match_count].feat_1 = new_features->feature_arr[i];
            matches->match_arr[match_count].feat_2 = old_features->feature_arr[match_index];
            match_count++;
        }
    }
    matches->count = match_count;
}

static void IRAM_ATTR calculate_new_box(target_box *box, const camera_fb_t *fb, const feature_match_vector *matches, const float match_ratio, state_vector *kf, const float dt)
{
    float match_thresh = 0.4f;
    float centroid_x = 0;
    float centroid_y = 0;
    size_t box_width = box->x_max - box->x_min;
    size_t box_height = box->y_max - box->y_min;
    size_t box_area = box_width * box_height;

    if (match_ratio > match_thresh)
    {
        float cx = (box->x_min + box->x_max) / 2.0f;
        float cy = (box->y_min + box->y_max) / 2.0f;
        float sum_x = 0;
        float sum_y = 0;
        float weight_sum = 0;

        // Calculate weighted centroid of the matches
        for (size_t i = 0; i < matches->count; i++)
        {
            float x = matches->match_arr[i].feat_1.x;
            float y = matches->match_arr[i].feat_1.y;

            // Distance from box center
            float dx = x - cx;
            float dy = y - cy;
            float dist2 = dx * dx + dy * dy;

            // weight = inverse distance (closer to center = more weight)
            static float k = 1.0f; // scaling factor for inverse distance. Higher means less effect from distance.
            float w = 1.0f / (1.0f + dist2 / ((float)box_area) * k);

            sum_x += w * x;
            sum_y += w * y;
            weight_sum += w;
        }

        centroid_x = sum_x / weight_sum;
        centroid_y = sum_y / weight_sum;
    }

    // Simplified Kalman filter.
    // Adjust alphas dynamically based on match count. Lower match % makes formula trust in predictions. Higher % makes formula more reactive to new measurements.

    static float ap_min = 0.1f;    // minimum trust in measured position
    static float ap_range = 0.7f;  // how much to increase trust as match ratio approaches 1
    static float av_min = 0.1f;   // minimum trust in newly calculated velocity
    static float av_range = 0.7f; // how much to increase trust as match ratio approaches 1

    float alpha_pos = ap_min + ap_range * match_ratio; // Trust in new measurement (higher) vs. predicted position (lower)
    float alpha_vel = av_min + av_range * match_ratio; // Trust in new velocity (higher) vs. last velocity (lower)

    if (!kf->initialized && match_ratio > match_thresh)
    {
        kf->x = centroid_x;
        kf->y = centroid_y;
        kf->vx = 0;
        kf->vy = 0;
        kf->initialized = true;
    }
    else if (!kf->initialized)
    {
        return;
    }
    else
    {
        // Predict position
        float pred_x = kf->x + kf->vx * dt;
        float pred_y = kf->y + kf->vy * dt;

        // If there are matches, use the measured position.
        if (match_ratio > match_thresh)
        {
            // Add prediction to measurement
            float nx = alpha_pos * centroid_x + (1 - alpha_pos) * pred_x;
            float ny = alpha_pos * centroid_y + (1 - alpha_pos) * pred_y;

            // Update velocity
            kf->vx = alpha_vel * (nx - kf->x) / dt + (1 - alpha_vel) * kf->vx;
            kf->vy = alpha_vel * (ny - kf->y) / dt + (1 - alpha_vel) * kf->vy;

            // Update position
            kf->x = nx;
            kf->y = ny;
        }
        // Else just predict
        else
        {
            kf->x = pred_x;
            kf->y = pred_y;
        }
    }

    // Calculate obtained box's coordinates
    int nx_min = kf->x - box_width / 2;
    int nx_max = kf->x + box_width / 2;
    int ny_min = kf->y - box_height / 2;
    int ny_max = kf->y + box_height / 2;

    if (nx_min < 0)
    {
        nx_min = 0;
        nx_max = box_width;
    }
    if (nx_max > fb->width)
    {
        nx_max = fb->width;
        nx_min = fb->width - box_width;
    }
    if (ny_min < 0)
    {
        ny_min = 0;
        ny_max = box_height;
    }
    if (ny_max > fb->height)
    {
        ny_max = fb->height;
        ny_min = fb->height - box_height;
    }

    box->x_min = nx_min;
    box->x_max = nx_max;
    box->y_min = ny_min;
    box->y_max = ny_max;
}

void tracker(camera_fb_t *fb, target_box *box, bool reset)
{
    static int fast_threshold = 30;
    static int fast_threshold_upper = 60;
    static int fast_threshold_lower = 10;
    static int match_threshold = 20;
    static int match_threshold_upper = 20;
    static int match_threshold_lower = 5;
    static int track_lost_threshold;
    static float match_ratio_threshold = 0.4f;
    static float match_ratio;
    static state_vector kf = {0}; // "Kalman" state vector, init all fields to 0
    static float track_lost_time = 0.0f;
    static float prev_time = 0.0f;

    static bool is_init = false;
    static feature_vector old_features;
    static feature_vector new_features;
    static feature_match_vector matches;

    static feature old_feature_arr[FEATURE_ARR_SIZE];
    static feature new_feature_arr[FEATURE_ARR_SIZE];
    static feature_match match_arr[FEATURE_ARR_SIZE];

    EXT_RAM_BSS_ATTR static uint32_t sat_buf[MAX_FRAME_SIZE];
    EXT_RAM_BSS_ATTR static uint8_t blur_buf[MAX_FRAME_SIZE];

    if (reset)
    {
        // RESET
        is_init = false;
        fast_threshold = 30;
        match_threshold = 20;
        kf.initialized = false;
        track_lost_time = 0.0f;
        return;
    }

    if (!is_init)
    {
        // INIT
        old_features.feature_arr = old_feature_arr;
        new_features.feature_arr = new_feature_arr;
        matches.match_arr = match_arr;
        old_features.size = FEATURE_ARR_SIZE;
        new_features.size = FEATURE_ARR_SIZE;
        matches.size = FEATURE_ARR_SIZE;
        old_features.count = 0;
        new_features.count = 0;
        matches.count = 0;
        prev_time = fb->timestamp.tv_sec + fb->timestamp.tv_usec / 1e6;

        // Calculate initial features using the box
        fast_from_middle(fb, fast_threshold, box, &old_features);
        boxblur_sat(fb, box, sat_buf, blur_buf);
        brief_tracking(fb, &old_features, blur_buf);

        is_init = true;
    }
    else
    {
        // TRACK
        int frame_width = fb->width;
        int frame_height = fb->height;

        // Calculate time difference
        float curr_time = fb->timestamp.tv_sec + fb->timestamp.tv_usec / 1e6;
        float dt = curr_time - prev_time;
        prev_time = curr_time;

        // TIMING
        static int64_t boxblur_search_accu = 0;
        static int64_t boxblur_accu = 0;
        static int64_t brief_search_accu = 0;
        static int64_t brief_accu = 0;
        static int64_t calculate_box_accu = 0;
        static int64_t fast_search_accu = 0;
        static int64_t fast_accu = 0;
        static int64_t match_accu = 0;
        static float elapsed_time = 0;
        static uint32_t frame_count = 0;
        frame_count++;

        // Scale the box for searching
        int box_width = box->x_max - box->x_min;
        int box_height = box->y_max - box->y_min;

        target_box search_box = {
            .x_min = (box->x_min - box_width / 2) > 0 ? (box->x_min - box_width / 2) : 0,
            .y_min = (box->y_min - box_height / 2) > 0 ? (box->y_min - box_height / 2) : 0,
            .x_max = (box->x_max + box_width / 2) < (frame_width - 1) ? (box->x_max + box_width / 2) : (frame_width - 1),
            .y_max = (box->y_max + box_height / 2) < (frame_height - 1) ? (box->y_max + box_height / 2) : (frame_height - 1)};

        // Find new features using the search box
        int64_t a = esp_timer_get_time();
        fast_from_middle(fb, fast_threshold, &search_box, &new_features);
        int64_t b = esp_timer_get_time();
        boxblur_sat(fb, &search_box, sat_buf, blur_buf);
        int64_t c = esp_timer_get_time();
        brief_tracking(fb, &new_features, blur_buf);
        int64_t d = esp_timer_get_time();

        // Calculate matches between the old and the new features
        int64_t e = esp_timer_get_time();
        match_tracking(&new_features, &old_features, &matches, match_threshold);
        int64_t f = esp_timer_get_time();

        // Calculate new target_box and clamp to frame bounds
        if (old_features.count > 0)
        {
            match_ratio = (float)matches.count / (float)old_features.count;
            match_ratio = (match_ratio > 1.0f) ? 1.0f : match_ratio;
        }
        else
        {
            match_ratio = 0.0f;
        }
        calculate_new_box(box, fb, &matches, match_ratio, &kf, dt);
        int64_t g = esp_timer_get_time();

        // Extract features from new target box COULD BE MOVED TO THE FOLLOWING IF BLOCK IF NOT DOING TIMING
        fast_from_middle(fb, fast_threshold, box, &new_features);
        int64_t h = esp_timer_get_time();
        boxblur_sat(fb, box, sat_buf, blur_buf);
        int64_t i = esp_timer_get_time();
        brief_tracking(fb, &new_features, blur_buf);
        int64_t j = esp_timer_get_time();

        // Reset state vector if track is lost to avoid the box jumping around due to persisting velocity.
        if (match_ratio > match_ratio_threshold)
        {
            track_lost_time = 0.0f;
            feature_vector temp = old_features;
            old_features = new_features;
            new_features = temp;
        }
        else
        {
            track_lost_time += dt;
            new_features.count = 0;
        }
        if (track_lost_time > track_lost_threshold)
        {
            kf.initialized = false; // Track lost, reset state vector
            track_lost_time = 0.0f;
        }

        // Adjust match distance threshold if matches can't be found or there are too many. Higher threshold -> more matches
        if (match_ratio < 20.0f && match_threshold < match_threshold_upper)
        {
            match_threshold += 5;
        }
        else if (match_ratio > 80.0f && match_threshold > match_threshold_lower)
        {
            match_threshold -= 5;
        }

        // Calculate saturation
        float saturation = (float)new_features.count / new_features.size;

        if (saturation > 0.8f && fast_threshold < fast_threshold_upper)
        {
            fast_threshold += 5;
        }
        else if (saturation < 0.2f && fast_threshold > fast_threshold_lower)
        {
            fast_threshold -= 5;
        }

        boxblur_search_accu += c - b;
        boxblur_accu += i - h;
        brief_search_accu += d - c;
        brief_accu += j - i;
        calculate_box_accu += g - f;
        fast_search_accu += b - a;
        fast_accu += h - g;
        match_accu += f - e;
        elapsed_time += dt;

        if (elapsed_time >= 10.0f)
        {
            uint64_t tracker_total_accu =
                fast_search_accu +
                boxblur_search_accu +
                brief_search_accu +
                match_accu +
                calculate_box_accu +
                fast_accu +
                boxblur_accu +
                brief_accu;

            printf(
                "\n=== Tracker Averages (last %.1f s, %lu frames) ===\n"
                "fast_from_middle(search): %llu us (%.3f ms)\n"
                "boxblur_sat(search):      %llu us (%.3f ms)\n"
                "brief_tracking(search):   %llu us (%.3f ms)\n"
                "match_tracking:           %llu us (%.3f ms)\n"
                "calculate_new_box:        %llu us (%.3f ms)\n"
                "fast_from_middle(box):    %llu us (%.3f ms)\n"
                "boxblur_sat(box):         %llu us (%.3f ms)\n"
                "brief_tracking(box):      %llu us (%.3f ms)\n"
                "tracker total:            %llu us (%.3f ms)\n\n",
                elapsed_time, frame_count,
                (unsigned long long)(fast_search_accu / frame_count), (fast_search_accu / frame_count) / 1000.0,
                (unsigned long long)(boxblur_search_accu / frame_count), (boxblur_search_accu / frame_count) / 1000.0,
                (unsigned long long)(brief_search_accu / frame_count), (brief_search_accu / frame_count) / 1000.0,
                (unsigned long long)(match_accu / frame_count), (match_accu / frame_count) / 1000.0,
                (unsigned long long)(calculate_box_accu / frame_count), (calculate_box_accu / frame_count) / 1000.0,
                (unsigned long long)(fast_accu / frame_count), (fast_accu / frame_count) / 1000.0,
                (unsigned long long)(boxblur_accu / frame_count), (boxblur_accu / frame_count) / 1000.0,
                (unsigned long long)(brief_accu / frame_count), (brief_accu / frame_count) / 1000.0,
                (unsigned long long)(tracker_total_accu / frame_count), (tracker_total_accu / frame_count) / 1000.0);

            // Reset accumulators
            boxblur_search_accu = 0;
            boxblur_accu = 0;
            brief_search_accu = 0;
            brief_accu = 0;
            calculate_box_accu = 0;
            fast_search_accu = 0;
            fast_accu = 0;
            match_accu = 0;
            elapsed_time = 0;
            frame_count = 0;
        }
    }
}