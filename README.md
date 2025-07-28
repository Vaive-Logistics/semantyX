# SemantyX

This project builds on MIT’s [semantic_inference](https://github.com/MIT-SPARK/semantic_inference) library to enable semantic segmentation of RGB camera input within the **ROS 2 Humble** ecosystem. To streamline integration, the original dependency on [Ianvs](https://github.com/MIT-SPARK/Ianvs) has been removed.

A lightweight and intuitive **ROS 2 wrapper** has been developed to simplify usage and deployment. Additionally, minor modifications were made to the standalone `semantic_inference` C++ library to support easier label group associations via **YAML configuration**.


## Installation
_The minimal steps in order to build and run this project are explained here. For more detailed information please read the [official README](https://github.com/MIT-SPARK/semantic_inference/blob/main/README.md) and the [Closed Set installation](https://github.com/MIT-SPARK/semantic_inference/blob/main/docs/closed_set.md#setting-up)._

### Getting Dependencies

  1. Add the CUDA repositories [here](https://developer.nvidia.com/cuda-downloads) by installing the `deb (network)` package or

```shell
# make sure you pick the correct ubuntu version!
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
```

  2. Check what version of CUDA TensorRT is built against:

```console
$ apt search nvinfer | grep cuda
libnvinfer-bin/unknown 10.4.0.26-1+cuda12.6 amd64
libnvinfer-dev/unknown,now 10.4.0.26-1+cuda12.6 amd64
...
```

  3.  Install TensorRT and CUDA if necessary:

```shell
# use the corresponding version number from the previous step or omit nvcc if already installed
sudo apt install libnvinfer-dev libnvonnxparsers-dev libnvinfer-plugin-dev cuda-nvcc-12-6
```

### Building

Once CUDA and TensorRT dependencies are installed:
```bash
cd /path/to/your/workspace/src/
git clone https://github.com/MIT-SPARK/config_utilities.git
git clone https://github.com/Vaive-Logistics/semantyX.git
cd ..
colcon build
```

You can run the following to validate that `semantic_inference` built correctly:
```shell
colcon test --packages-select semantic_inference
```

### Get Models

Running dense 2D semantic segmentation requires obtaining a pre-trained model (onnx).
Several pre-exported models live [here](https://drive.google.com/drive/folders/1GrmgFDFCssDxKe_Nyx8PPTK1pRMA0gEO?usp=sharing).
By default, the code uses [this](https://drive.google.com/file/d/1XRcsyLSvqqhqNIaOI_vmqpUpmBT6gk9-/view?usp=drive_link) model.

By default, the closed set node looks under the directory `$HOME/.semantic_inference` for models (this works on Linux or as long as `HOME` is set).
It is possible to change this directory by specifying the `SEMANTIC_INFERENCE_MODEL_DIR` environment variable.
To use a specific downloaded model, use the argument `model_file:=MODEL_FILE` when running the appropriate launch file (where `MODEL_NAME` is the filename of the model relative to the configured model directory).
Specifying an absolute filepath will override the default model directory.

Note that the pipeline as implemented works with any pre-trained model exported to [onnx](https://onnx.ai/) as long as the model takes 32-bit float 3-channel tensors as input and outputs labels in a single-channel tensor as integers.
The pipeline can optionally rescale and offset the input tensor.ç

## Quick Start

Once the project is built, you can run the basic `segmentator_node` via:
```bash
ros2 launch semantic_inference_ros semantic_inference.launch.yaml
```