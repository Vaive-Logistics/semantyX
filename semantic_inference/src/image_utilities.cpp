/* -----------------------------------------------------------------------------
 * BSD 3-Clause License
 *
 * Copyright (c) 2021-2024, Massachusetts Institute of Technology.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * * -------------------------------------------------------------------------- */

#include "semantic_inference/image_utilities.h"

#include <config_utilities/config.h>

#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "semantic_inference/logging.h"

namespace semantic_inference {

float ColorConverter::convert(uint8_t input_val, size_t channel) const {
  float to_return = config.map_to_unit_range ? (input_val / 255.0f) : input_val;
  to_return = config.normalize
                  ? (to_return - config.mean[channel]) / config.stddev[channel]
                  : to_return;
  return to_return;
}

void ColorConverter::fillImage(const cv::Mat& input, cv::Mat& output) const {
  if (output.size.dims() != 3) {
    SLOG(ERROR) << "Invalid output matrix!";
    return;
  }

  // [OPTIMIZATION] Replaced pixel-by-pixel loop with vectorized OpenCV ops.
  // Original code used nested loops with .at<float>() (incl. bounds checks),
  // iterating ~W*H*3 times. New code uses cv::split + convertTo + arithmetic
  // which maps to SIMD-accelerated routines, dramatically faster on ARM/Jetson.

  const bool is_chw_order = output.size[0] == 3;
  const int rows = is_chw_order ? output.size[1] : output.size[0];
  const int cols = is_chw_order ? output.size[2] : output.size[1];

  // Resize if needed (same logic as before)
  cv::Mat img;
  if (input.cols == cols && input.rows == rows) {
    img = input;
  } else {
    cv::resize(input, img, cv::Size(cols, rows));
  }

  // Split into per-channel planes. OpenCV delivers BGR order.
  std::vector<cv::Mat> channels(3);
  cv::split(img, channels);  // channels[0]=B, channels[1]=G, channels[2]=R

  // Reorder channels to match target color space
  if (config.rgb_order) {
    std::swap(channels[0], channels[2]);  // BGR → RGB
  }
  // channels[c] is now the c-th channel in the target order

  const float inv255 = 1.0f / 255.0f;

  if (is_chw_order) {
    // CHW layout: output is a 3D mat [3, H, W] — each channel is a contiguous
    // HxW block. We wrap each block as a 2D mat and write directly into it.
    for (int c = 0; c < 3; ++c) {
      cv::Mat dst(rows, cols, CV_32F, output.ptr<float>(c));
      channels[c].convertTo(dst, CV_32F);
      if (config.map_to_unit_range) {
        dst *= inv255;
      }
      if (config.normalize) {
        dst -= config.mean[c];
        dst *= (1.0f / config.stddev[c]);
      }
    }
  } else {
    // HWC layout: output is a 3D mat [H, W, 3] — channels are interleaved.
    // Process each channel independently, then merge into a temporary 2D
    // 3-channel mat (identical memory layout to [H, W, 3]) and memcpy out.
    std::vector<cv::Mat> float_channels(3);
    for (int c = 0; c < 3; ++c) {
      channels[c].convertTo(float_channels[c], CV_32F);
      if (config.map_to_unit_range) {
        float_channels[c] *= inv255;
      }
      if (config.normalize) {
        float_channels[c] -= config.mean[c];
        float_channels[c] *= (1.0f / config.stddev[c]);
      }
    }
    cv::Mat merged(rows, cols, CV_32FC3);
    cv::merge(float_channels, merged);
    // A 3D HWC mat [H,W,3] of CV_32FC1 and a 2D mat [H,W] of CV_32FC3 share
    // identical memory layout — plain memcpy is safe here.
    std::memcpy(output.data, merged.data,
                static_cast<size_t>(rows) * cols * 3 * sizeof(float));
  }
}

void declare_config(ColorConverter::Config& config) {
  using namespace config;
  name("ColorConverter::Config");
  field(config.mean, "mean");
  field(config.stddev, "stddev");
  field(config.map_to_unit_range, "map_to_unit_range");
  field(config.normalize, "normalize");
  field(config.rgb_order, "rgb_order");
}

float DepthConverter::convert(float input_val) const {
  if (!config.normalize) {
    return input_val;
  }

  const float new_value = (input_val - config.mean) / config.stddev;
  if (new_value < 0.0f) {
    return 0.0f;
  }

  return new_value;
}

void DepthConverter::fillImage(const cv::Mat& input, cv::Mat& output) const {
  if (output.size.dims() != 2) {
    SLOG(ERROR) << "Invalid output matrix!";
    return;
  }

  // [OPTIMIZATION] Replaced pixel-by-pixel loop with vectorized OpenCV ops.
  // Original iterated every pixel with .at<float>(); new code uses convertTo
  // and element-wise arithmetic backed by SIMD on ARM Cortex-A / Jetson CPU.

  const bool size_ok = input.cols == output.cols && input.rows == output.rows;
  cv::Mat img;
  if (size_ok) {
    img = input;
  } else {
    cv::resize(input, img, cv::Size(output.cols, output.rows), 0, 0, cv::INTER_NEAREST);
  }

  // Ensure float input
  cv::Mat float_img;
  if (img.type() == CV_32F) {
    float_img = img;
  } else {
    img.convertTo(float_img, CV_32F);
  }

  if (!config.normalize) {
    float_img.copyTo(output);
  } else {
    // normalize: (x - mean) / stddev, then clamp negative values to 0.
    // Original loop did this per-pixel; cv::subtract + multiply + max are SIMD.
    cv::subtract(float_img, config.mean, output);
    output *= (1.0f / config.stddev);
    cv::max(output, 0.0f, output);
  }
}

void declare_config(DepthConverter::Config& config) {
  using namespace config;
  name("DepthConverter::Config");
  field(config.mean, "mean");
  field(config.stddev, "stddev");
  field(config.normalize, "normalize");
}

cv::Mat DepthLabelMask::maskLabels(const cv::Mat& labels, const cv::Mat& depth) const {
  cv::Mat resized_depth;
  cv::resize(depth,
             resized_depth,
             cv::Size(labels.cols, labels.rows),
             0,
             0,
             cv::INTER_NEAREST);

  cv::Mat mask;
  cv::inRange(resized_depth, config.min_depth, config.max_depth, mask);

  cv::Mat masked_labels;
  cv::bitwise_or(labels, labels, masked_labels, mask);
  return masked_labels;
}

void declare_config(DepthLabelMask::Config& config) {
  using namespace config;
  name("DepthLabelMask::Config");
  field(config.min_depth, "min_depth");
  field(config.max_depth, "max_depth");
}

std::string getLabelPercentages(const cv::Mat& labels) {
  std::map<int32_t, size_t> counts;
  std::vector<int32_t> unique_classes;
  for (int r = 0; r < labels.rows; ++r) {
    for (int c = 0; c < labels.cols; ++c) {
      int32_t class_id = 0;
      if (labels.type() == CV_32SC1) {
        class_id = labels.at<int32_t>(r, c);
      } else if (labels.type() == CV_16SC1) {
        class_id = labels.at<int16_t>(r, c);
      } else {
        return "invalid type: " + std::to_string(labels.type());
      }

      if (!counts.count(class_id)) {
        counts[class_id] = 0;
        unique_classes.push_back(class_id);
      }

      counts[class_id]++;
    }
  }

  double total = static_cast<double>(labels.rows * labels.cols);
  std::sort(unique_classes.begin(),
            unique_classes.end(),
            [&](const int32_t& lhs, const int32_t& rhs) {
              return counts[lhs] > counts[rhs];
            });

  std::stringstream ss;
  ss << " Class pixel percentages:" << std::endl;
  for (const int32_t id : unique_classes) {
    ss << "  - " << id << ": " << static_cast<double>(counts[id]) / total * 100.0 << "%"
       << std::endl;
  }

  return ss.str();
}

}  // namespace semantic_inference
