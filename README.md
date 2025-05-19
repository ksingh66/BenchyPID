# BenchyPID

An object detection and tracking system that uses YOLOv5 for object recognition and Arduino-controlled pan-tilt-zoom servos to track objects in real-time.

## Overview + Demo

Demo video: https://www.youtube.com/watch?v=3AuYJmLxwaw

BenchyPID combines computer vision with hardware control to create a system that can:

- Detect objects using YOLOv5 models
- Track the detected objects in real-time
- Control a pan-tilt servo mechanism via Arduino
- Stream video with visual feedback
- Optionally zoom into detected objects

## Requirements

### Hardware

- Arduino board (connected via USB)
- Pan-tilt servo mechanism
- Webcam or video source
- Computer with Python support (GPU recommended for better performance)

### Software

- Python 3.6+
- YOLOv5
- OpenCV
- PyTorch
- Arduino IDE
- Easiest to run on RaspberryPiOS

## Installation

- Check our this video for a guide : https://www.youtube.com/watch?v=vcFRip1ln-0&feature=youtu.be
  

### 1. Clone the Repository

```bash
git clone https://github.com/ksingh66/BenchyPID.git
cd BenchyPID
```

### 2. Clone YOLOv5

```bash
git clone https://github.com/ultralytics/yolov5.git
```

### 3. Move the Tracking Script

```bash
cp ptz_tracking.py yolov5/
```

### 4. Set Up Python Environment

```bash
# Create and activate virtual environment
python -m venv environment
source environment/bin/activate  # On Windows, use: environment\Scripts\activate

# Install requirements
pip install -r requirements.txt
```

### 5. Set Up Arduino

1. Open Arduino IDE
2. Create a new sketch and copy the contents of arduino.c into it
3. Connect your Arduino board via USB
4. Select the correct board and port in the Arduino IDE
5. Upload the code to your Arduino

## Usage

1. Activate the Python virtual environment:

```bash
source environment/bin/activate  # On Windows, use: environment\Scripts\activate
```

2. Navigate to the YOLOv5 directory:

```bash
cd yolov5
```

3. Run the tracking script:

```bash
python ptz_tracking.py --source 0 --weights ../best.pt --view-img --zoom
```

### Command-Line Arguments

| Argument          | Description                                        | Default             |
| ----------------- | -------------------------------------------------- | ------------------- |
| `--weights`       | Path to YOLOv5 model weights                       | `../best.pt`        |
| `--source`        | Video source (0 for webcam, or path to video file) | `0`                 |
| `--data`          | Path to data configuration                         | `data/coco128.yaml` |
| `--imgsz`         | Input image size                                   | `320`               |
| `--conf-thres`    | Confidence threshold                               | `0.25`              |
| `--iou-thres`     | IoU threshold for NMS                              | `0.45`              |
| `--max-det`       | Maximum number of detections                       | `10`                |
| `--device`        | Device to run on (cuda device or cpu)              | `''`                |
| `--view-img`      | Show results window                                | `False`             |
| `--classes`       | Filter by class                                    | `None`              |
| `--agnostic-nms`  | Class-agnostic NMS                                 | `False`             |
| `--arduino`       | Arduino serial port                                | `/dev/ttyUSB0`      |
| `--baudrate`      | Arduino baudrate                                   | `115200`            |
| `--debug`         | Print debug information                            | `False`             |
| `--move-interval` | Movement interval in seconds                       | `0.05`              |
| `--min-movement`  | Minimum pixel movement to send command             | `2`                 |
| `--zoom`          | Zoom into the detected object                      | `False`             |

## Controls

When the view window is active, you can use the following keyboard controls:

- `q` - Quit the application
- `t` - Tune Arduino PID values
- `r` - Reset servo positions to center

## Troubleshooting

- **Arduino connection issues**: Ensure the correct port is specified with `--arduino` argument. On Linux, it's typically `/dev/ttyUSB0` or `/dev/ttyACM0`. On Windows, it's typically `COM3` or similar.
- **Camera not found**: Make sure your webcam is properly connected and recognized. Try changing the `--source` parameter.
- **Model not loading**: Verify the path to your weights file is correct. The default is `../best.pt` relative to the YOLOv5 directory.
- **Low performance**: Try reducing the image size with `--imgsz` or use a more powerful computer with GPU support.

## How This Works

1. The script captures video from the specified source.
2. Each frame is processed by the YOLOv5 model to detect objects.
3. The highest-confidence detection is selected for tracking.
4. The pan-tilt error is calculated as the difference between the center of the frame and the center of the detected object.
5. Commands are sent to the Arduino to adjust servo positions.
6. The Arduino uses PID control to smoothly track the object; by default this is a P controller only but you may add D and I.

## Customization

- **Use your own YOLOv5 model**: Train your own model and specify it with `--weights`
- **Adjust PID values**: Press 't' during operation to tune the PID controller
- **Modify tracking parameters**: Adjust `--move-interval` and `--min-movement` to change tracking behavior
- **Camera exposure**: The script attempts to set camera exposure for optimal performance, but you may need to adjust exposure values in the main function
