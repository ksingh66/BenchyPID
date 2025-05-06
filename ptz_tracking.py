#!/usr/bin/env python3
# benchy_tracker.py - Optimized YOLOv5 tracking for Raspberry Pi 5

import argparse
import cv2
import numpy as np
import serial
import time
import torch
import torch.backends.cudnn as cudnn
import os
import atexit  # Add this for cleanup on exit
import signal  # Add this for handling signals

from pathlib import Path
import sys

# Add YOLOv5 directory to path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# Import YOLOv5 modules
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import check_img_size, non_max_suppression
from utils.torch_utils import select_device, time_sync

# Try different imports based on YOLOv5 version
try:
    from utils.general import scale_coords
except ImportError:
    try:
        from utils.general import scale_boxes as scale_coords
    except ImportError:
        print("Warning: Could not import scale_coords or scale_boxes. Detection may not work correctly.")

try:
    from utils.plots import Annotator, colors
except ImportError:
    print("Warning: Could not import Annotator. Will use basic OpenCV drawing instead.")
    Annotator = None
    colors = lambda x, y: (0, 255, 0)

class PTZController:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.serial = None
        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Connected to Arduino on {port}")
            
            # Wait for READY signal from Arduino
            ready = False
            start_time = time.time()
            timeout = 5  # seconds
            
            while not ready and (time.time() - start_time) < timeout:
                if self.serial.in_waiting:
                    response = self.serial.readline().decode('utf-8').strip()
                    print(f"Arduino says: {response}")
                    if response == "READY":
                        ready = True
                        print("Arduino ready for commands")
            
            if not ready:
                print("Warning: Arduino did not send READY signal, continuing anyway")
                
        except Exception as e:
            print(f"Error connecting to Arduino: {e}")
            self.serial = None
        
        # Frame dimensions
        self.frame_width = 640
        self.frame_height = 480
        self.center_x = self.frame_width / 2
        self.center_y = self.frame_height / 2
        
        # Current servo positions
        self.current_pan = 90
        self.current_tilt = 90
        
        # Control parameters
        self.last_move_time = time.time()
        self.move_interval = 0.05  # Send commands every 50ms
        self.min_movement = 2      # Minimum movement threshold in pixels
        
        # Register cleanup function at exit
        atexit.register(self.cleanup)
    
    def cleanup(self):
        """Send servos to center position and close serial connection"""
        if self.serial is not None:
            try:
                print("Cleanup: Resetting servos to center position")
                # Send reset command
                self.serial.write("RESET\n".encode())
                time.sleep(0.5)  # Wait for reset
                self.serial.close()
                print("Serial connection closed")
            except:
                pass
            
    def send_target_position(self, target_x, target_y, debug=False):
        """Send target position directly to Arduino without local PID calculation"""
        if self.serial is None:
            print("No Arduino connection!")
            return False
            
        # Only send updates at specified interval
        current_time = time.time()
        if current_time - self.last_move_time < self.move_interval:
            return False
        
        # Calculate target error
        error_x = target_x - self.center_x
        error_y = target_y - self.center_y
        
        # Skip small movements
        if abs(error_x) < self.min_movement and abs(error_y) < self.min_movement:
            return False
        
        # Map image coordinates to servo angles
        # Invert X axis so right is decreasing angle (adjust if needed)
        target_pan = int(180 - (target_x / self.frame_width * 180))
        # Map Y axis (adjust if needed)
        target_tilt = int(target_y / self.frame_height * 180)
        
        # Constrain to valid range
        target_pan = max(0, min(180, target_pan))
        target_tilt = max(0, min(180, target_tilt))
        
        # Update move time
        self.last_move_time = current_time
        
        # Send command to Arduino
        command = f"PT:{target_pan},{target_tilt}\n"
        
        if debug:
            print(f"\nTarget: ({target_x:.1f}, {target_y:.1f}), Error: ({error_x:.1f}, {error_y:.1f})")
            print(f"Sending: Pan={target_pan}, Tilt={target_tilt}")
        
        try:
            self.serial.write(command.encode())
            
            # Read response with timeout
            start_time = time.time()
            while self.serial.in_waiting == 0 and time.time() - start_time < 0.1:
                time.sleep(0.01)
                
            if self.serial.in_waiting:
                response = self.serial.readline().decode('utf-8').strip()
                if debug:
                    print(f"Arduino response: {response}")
                
                # Try to parse current position from Arduino response
                if response.startswith("PT:"):
                    try:
                        parts = response.split(':')[1].split(',')
                        self.current_pan = int(parts[0])
                        self.current_tilt = int(parts[1])
                    except:
                        pass
                        
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False

# Global PTZ controller for signal handlers
global_ptz = None

def signal_handler(sig, frame):
    """Handle Ctrl+C and other signals to gracefully exit"""
    print("Exiting gracefully...")
    if global_ptz is not None:
        global_ptz.cleanup()
    cv2.destroyAllWindows()
    sys.exit(0)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='../best.pt', help='model path')
    parser.add_argument('--source', type=str, default='0', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--data', type=str, default='data/coco128.yaml', help='dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[320], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=10, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--arduino', type=str, default='/dev/ttyACM0', help='Arduino port')
    parser.add_argument('--baudrate', type=int, default=115200, help='Arduino baudrate')
    parser.add_argument('--debug', action='store_true', help='Print debug information')
    parser.add_argument('--move-interval', type=float, default=0.05, help='Movement interval in seconds')
    parser.add_argument('--min-movement', type=int, default=2, help='Minimum pixel movement to send command')
    opt = parser.parse_args()
    
    # Handle imgsz
    if len(opt.imgsz) == 1:
        opt.imgsz = [opt.imgsz[0], opt.imgsz[0]]
        
    # Print args
    print(f"Arguments: {vars(opt)}")
    return opt

def main(opt):
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only on CUDA
    
    # Load model
    model = DetectMultiBackend(opt.weights, device=device, data=opt.data)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(opt.imgsz, s=stride)  # check image size
    
    # Half precision
    half &= pt and device.type != 'cpu'  # half precision only on CUDA and PyTorch model
    if pt:
        model.model.half() if half else model.model.float()
    
    # Dataloader (optimized for Raspberry Pi)
    source = str(opt.source)
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))
    
    if webcam:
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt)
        bs = 1  # batch_size
    
    # Initialize PTZ controller
    global global_ptz
    ptz = PTZController(port=opt.arduino, baudrate=opt.baudrate)
    global_ptz = ptz  # Set global reference for signal handlers
    
    ptz.move_interval = opt.move_interval
    ptz.min_movement = opt.min_movement
    
    # Run inference
    model.warmup(imgsz=(1 if pt else bs, 3, *imgsz))  # warmup
    
    # FPS calculation variables
    fps_start_time = time.time()
    fps_frame_count = 0
    fps = 0
    
    try:
        for path, im, im0s, vid_cap, s in dataset:
            # FPS calculation
            fps_frame_count += 1
            if time.time() - fps_start_time >= 1.0:
                fps = fps_frame_count / (time.time() - fps_start_time)
                fps_frame_count = 0
                fps_start_time = time.time()
                print(f"FPS: {fps:.1f}")
            
            # Preprocess image
            im = torch.from_numpy(im).to(device)
            im = im.half() if half else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            
            # Inference
            pred = model(im, augment=False, visualize=False)
            
            # NMS
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, 
                                      getattr(opt, 'agnostic_nms', False), max_det=opt.max_det)
            
            # Process detections
            for i, det in enumerate(pred):  # per image
                if webcam:  # batch_size >= 1
                    p, im0, frame = path[i], im0s[i].copy(), dataset.count
                else:
                    p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)
                
                # Get frame dimensions
                h, w = im0.shape[:2]
                ptz.frame_width = w
                ptz.frame_height = h
                ptz.center_x = w / 2
                ptz.center_y = h / 2
                
                # Try to use Annotator if available
                has_annotator = False
                if Annotator is not None:
                    try:
                        annotator = Annotator(im0, line_width=2, example=str(names))
                        has_annotator = True
                    except:
                        has_annotator = False
                
                # Draw center crosshairs
                cv2.line(im0, (int(ptz.center_x-10), int(ptz.center_y)), 
                        (int(ptz.center_x+10), int(ptz.center_y)), (0, 0, 255), 2)
                cv2.line(im0, (int(ptz.center_x), int(ptz.center_y-10)), 
                        (int(ptz.center_x), int(ptz.center_y+10)), (0, 0, 255), 2)
                
                # Draw FPS on frame
                cv2.putText(im0, f"FPS: {fps:.1f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    try:
                        det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()
                    except Exception as e:
                        print(f"Error scaling coordinates: {e}")
                        continue
                    
                    # Find benchy with highest confidence
                    best_det = None
                    best_conf = 0
                    best_cls = -1
                    
                    for *xyxy, conf, cls in reversed(det):
                        c = int(cls)
                        # Use highest confidence detection
                        if conf > best_conf:  
                            best_conf = conf
                            best_det = xyxy
                            best_cls = c
                    
                    # Process best detection
                    if best_det is not None:
                        xyxy = best_det
                        x1, y1, x2, y2 = [int(c) for c in xyxy]
                        
                        # Calculate center of bounding box
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        
                        # Draw bounding box
                        c = best_cls
                        label = f"{names[c]} {best_conf:.2f}"
                        
                        if has_annotator:
                            annotator.box_label(xyxy, label, color=colors(c, True))
                        else:
                            # Fallback drawing method
                            cv2.rectangle(im0, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.putText(im0, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Draw center of detection
                        cv2.circle(im0, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
                        
                        # Draw line from center to target
                        cv2.line(im0, (int(ptz.center_x), int(ptz.center_y)), 
                                 (int(center_x), int(center_y)), (255, 0, 0), 2)
                        
                        # Draw error values
                        error_x = center_x - ptz.center_x
                        error_y = center_y - ptz.center_y
                        cv2.putText(im0, f"Err X: {error_x:.1f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cv2.putText(im0, f"Err Y: {error_y:.1f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # Send target position to Arduino
                        ptz.send_target_position(center_x, center_y, debug=opt.debug)
                        
                        # Show servo positions
                        cv2.putText(im0, f"Pan: {ptz.current_pan}", (20, h-30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                        cv2.putText(im0, f"Tilt: {ptz.current_tilt}", (20, h-60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                
                # Stream results
                im0_result = annotator.result() if (has_annotator and annotator is not None) else im0
                if opt.view_img:
                    cv2.imshow(str(p), im0_result)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):  # Press 'q' to quit
                        print("Exiting...")
                        break
                    elif key == ord('t'):  # Press 't' to tune Arduino PID directly
                        # Tune both servos with the same values
                        p = float(input("Enter P value (0.1-0.5): "))
                        i = float(input("Enter I value (0.0-0.01): "))
                        d = float(input("Enter D value (0.01-0.1): "))
                        
                        # Tune pan servo
                        command = f"TUNE:0,{p:.4f},{i:.4f},{d:.4f}\n"
                        print(f"Sending pan PID tune command: {command.strip()}")
                        if ptz.serial:
                            ptz.serial.write(command.encode())
                            time.sleep(0.1)
                        
                        # Tune tilt servo
                        command = f"TUNE:1,{p:.4f},{i:.4f},{d:.4f}\n"
                        print(f"Sending tilt PID tune command: {command.strip()}")
                        if ptz.serial:
                            ptz.serial.write(command.encode())
                    elif key == ord('r'):  # Press 'r' to reset
                        if ptz.serial:
                            ptz.serial.write("RESET\n".encode())
                            print("Reset command sent")
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Always cleanup properly
        print("Cleaning up...")
        ptz.cleanup()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        from utils.general import check_requirements
        check_requirements(exclude=('tensorboard', 'thop'))
    except:
        print("Skipping requirements check...")
    opt = parse_args()
    main(opt)