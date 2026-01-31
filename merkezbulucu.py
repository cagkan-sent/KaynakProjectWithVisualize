import VsxProtocolDriver
from Sr2DParameterIdentifier import Sr2D_parameters
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons
import urllib.request
import urllib.parse
import statistics
import cv2
from collections import Counter
import time
import subprocess
import sys
import os


x_resolution=0
surface_z=0
rect=None
holeZlimit=2 #surface_z den daha ilerideki noktaları invalid yap
surfaceline=None
numofinvalidreadings=0
segments=[]
pipe_line_points =None
pipe_segment_info=None
debug=False
startpipesearch=False
is_backward = False
keep_running = True
imagetransfer=True
segment_slope_threshold=0.1

# ---- STOP FILTER (NOISE + N-FRAME CONFIRM) ----
MIN_NOISE_LEN = 6.0          # 5-6mm altı segment = gürültü, çıkışa sayma
PIPE_LOST_FRAMES = 4         # kaç frame üst üste "kayıp" olursa çık
pipe_lost_counter = 0        # sayaç

def on_close(event):
    global keep_running
    keep_running = False

def reset_parameters():
    global segments, surfaceline, pipe_segment_info, numofinvalidreadings, pipe_line_points, pipe_image, maxsegmentlength, segment_line, motortimer, motor_timeout, surface_z, rect, x_resolution, pipe_lost_counter
    segments=[]
    surfaceline=None
    pipe_segment_info=None
    numofinvalidreadings=0
    pipe_line_points=None
    pipe_image=None
    maxsegmentlength=0
    segment_line=None
    motortimer=0
    motor_timeout=20
    surface_z=0
    rect=None
    x_resolution=0
    pipe_lost_counter=0

def on_button_clicked(event):
    global startpipesearch
    reset_parameters()
    startpipesearch=True  

def on_check_clicked(label):
    global is_backward
    if label == 'Geri':
        is_backward = not is_backward
        print(f"Direction changed: {'Backward' if is_backward else 'Forward'}")

def send_command(commandstr):
    base_url = "http://192.168.8.10/command"
    params = {
        "commandText": commandstr,
        "PAGEID": "0"
    }
    
    # Encode parameters
    query_string = urllib.parse.urlencode(params)
    url = f"{base_url}?{query_string}"
    
    try:
        print(f"Sending request to: {url}")
        with urllib.request.urlopen(url, timeout=10) as response:
            pass # Command sent successfully
    except Exception as e:
        print(f"Error: {str(e)}")

pipe_radius = 8.2
roi_min_x = -(pipe_radius + 1)
roi_max_x = (pipe_radius + 1)
roi_width = roi_max_x - roi_min_x

def main():
    #send_command("G91")
    #send_command("G1 X100 F5000")
    # First discover devices via UDP and use the ip of the first device found
    ret, sensors = VsxProtocolDriver.Sensor.GetUdpDeviceList()

    if ret == VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS and len(sensors) > 0:
        # create a new VsxProtocolDriver instance
        dev = sensors[0]
        print("Device found: {0}({1})".format(dev["sensorType"], dev["ipAddress"]))
        sensor = VsxProtocolDriver.Sensor.InitTcpSensor(dev["ipAddress"], "")

    else:  # use fix ip address
        # create a new VsxProtocolDriver instance with fix ip address
        sensor = VsxProtocolDriver.Sensor.InitTcpSensor("192.168.2.3", "")

    # Connect with device
    ret = sensor.Connect()
    if ret != VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print(f"Unable to connect with device: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return

    # Get the current device information
    ret, deviceInformation = sensor.GetDeviceInformation()

    if ret == VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print("Connected with device {0}, IP: {1}".format(deviceInformation["sensorType"],
                                                          deviceInformation["ipAddress"]))
        print()
    else:
        print(f"Unable to receive device information: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return

    sensorType = deviceInformation["sensorType"]

    if sensorType == "SMARTRUNNER":
        run_example(sensor)
    else:
        print("Wrong type of sensor connected for this demo")
    ret = sensor.SetSingleParameterValue(1, Sr2D_parameters.Autotrigger_0x510001.value["ConfigId"], 1,
                                         Sr2D_parameters.Autotrigger_0x510001.value["ParamId"], "false")
    if ret != VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print(f"Error disabling trigger: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return
    # Disconnect device
    sensor.Disconnect()


def run_example(sensor: VsxProtocolDriver.Sensor):
    # Print the SR2D parameters

    print("--- Device contains the following parameters: ---")
    for p in list(Sr2D_parameters):
        ret, value = sensor.GetSingleParameterValue(1, p.value["ConfigId"], 1, p.value["ParamId"])
        if ret == VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
            print("Id: {0}, Name: {1}, Value: {2}".format(p.value["ParamId"], p.name, value))
        else:
            print("Reading parameter value from device failed")
            return
    print("ROI Min X: {0}, Max X: {1}, Width: {2}".format(roi_min_x,roi_max_x,roi_width))
    # Start autotrigger and enable trigger, see manual for config Id and parameter Id
    ret = sensor.SetSingleParameterValue(1, Sr2D_parameters.Autotrigger_0x510001.value["ConfigId"], 1,
                                         Sr2D_parameters.Autotrigger_0x510001.value["ParamId"], "true")
    sensor.SetSingleParameterValue(1, Sr2D_parameters.ROIMinX_0xC90001.value["ConfigId"], 1,
                                         Sr2D_parameters.ROIMinX_0xC90001.value["ParamId"], -1*(pipe_radius/2 + 3))
    sensor.SetSingleParameterValue(1, Sr2D_parameters.ROIMaxX_0xC90002.value["ConfigId"], 1,
                                         Sr2D_parameters.ROIMaxX_0xC90002.value["ParamId"], 1*(pipe_radius/2 + 3))
    if ret != VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print(f"Error enabling trigger: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return

    # Reconfigure grabber queue to queue 3 buffers containing line data
    ret = sensor.ResetDynamicContainerGrabberEx(3, VsxProtocolDriver.Strategy.DROP_OLDEST)
    if ret != VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print(f"Error configuring grabber: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return
    transferimageactive="True" if imagetransfer else "False"
    ret = sensor.SetSingleParameterValue(1, Sr2D_parameters.ImageTransferActive.value["ConfigId"], 1,
                                         Sr2D_parameters.ImageTransferActive.value["ParamId"], transferimageactive)
    if ret != VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS:
        print(f"Error configuring grabber: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
        return
    fig, ax = plt.subplots(figsize=(10, 6))
    fig.canvas.mpl_connect('close_event', on_close)
    plt.subplots_adjust(bottom=0.2)
    ax.set_title("Captured Lines")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Z (mm)")
    ax.grid(True)

    # Set up the button
    ax_button = plt.axes([0.7, 0.05, 0.2, 0.075])
    btn = Button(ax_button, 'Boru Aramayi Başlat')
    btn.on_clicked(on_button_clicked)

    # Set up the checkbox
    ax_check = plt.axes([0.55, 0.05, 0.1, 0.075])
    check = CheckButtons(ax_check, ['Geri'], [is_backward])
    check.on_clicked(on_check_clicked)

    all_lines_data = [] # Initialize inside loop to clear on each iteration
    send_command("G91")
    line_buffer = [] # Buffer for Z-smoothing
    segment_buffer = [] # Buffer for segment median filtering
    boruyageldim=False
    kacincidefabuldum=0 
    while keep_running:
        boruyageldim=borubuldum(sensor, line_buffer, segment_buffer, ax)
    # stop auto-trigger
   
    startpipesearch=False
    # Plot all captured lines


def calculate_linewidth(line_points):
    valid_xs = []
    for i in range(len(line_points)):
        if line_points[i]["z"] != 0:
            valid_xs.append(line_points[i]["x"])
            
    if valid_xs:
        return max(valid_xs) - min(valid_xs)
    
    # Fallback default
    return 0

def calculate_resolution(line_points):
    valid_diffs = []
    for i in range(len(line_points) - 1):
        # Check if both current and next point are valid (z != 0)
        if line_points[i]["z"] != 0 and line_points[i+1]["z"] != 0:
            diff = abs(line_points[i+1]["x"] - line_points[i]["x"])
            valid_diffs.append(diff)
            
    if valid_diffs:
        return statistics.median(valid_diffs)
    
    # Fallback/Default if calculation fails
    return 0


def isValidLine(line_points):
    invalid_count = sum(1 for p in line_points if p["x"] == 0 and p["z"] == 0)
    return invalid_count < 2*len(line_points) / 3

def filter_valid_noise(raw_xs, raw_zs):
    if not raw_xs or not raw_zs:
        return [], []
    xs, zs = list(raw_xs), list(raw_zs)
    count = len(raw_zs)
    zs_copy = list(raw_zs) # Copy to check original neighbors
    for i in range(count):
        if zs[i] != 0:
            left_invalid = (i == 0) or (zs_copy[i-1] == 0)
            right_invalid = (i == count - 1) or (zs_copy[i+1] == 0)
            if (left_invalid and right_invalid):
                zs[i] = 0
                xs[i] = 0
    return xs,zs

def find_invalid_segments(xs, zs, roi_start, roi_end, surface_z):
    segments = []
    
    current_segment_start_idx = -1
    
    # Iterate through ROI
    for i in range(roi_start, roi_end + 1):
        # Start of invalid segment?
        if zs[i] == 0:
            if current_segment_start_idx == -1:
                current_segment_start_idx = i
                
        # End of invalid segment? (Either valid point found OR reached ROI end)
        if current_segment_start_idx != -1:
            is_end = False
            if i == roi_end:
                 is_end = True
            elif zs[i+1] != 0:
                 is_end = True
            
            if is_end:
                # Find prev valid
                prev_valid_idx = -1
                for k in range(current_segment_start_idx - 1, -1, -1):
                    if zs[k] != 0:
                        prev_valid_idx = k
                        break
                        
                # Find next valid
                next_valid_idx = -1
                for k in range(i + 1, len(zs)):
                    if zs[k] != 0:
                        next_valid_idx = k
                        break
                
                if prev_valid_idx != -1 and next_valid_idx != -1:
                    if abs(zs[prev_valid_idx] - surface_z) <= 10 and abs(zs[next_valid_idx] - surface_z) <= 10:
                        length = xs[next_valid_idx] - xs[prev_valid_idx]
                        if length > 0:
                            slope = abs(zs[next_valid_idx] - zs[prev_valid_idx]) / length
                            if slope <= segment_slope_threshold:
                                start_pt = (xs[prev_valid_idx], zs[prev_valid_idx])
                                end_pt = (xs[next_valid_idx], zs[next_valid_idx])
                                segments.append((length, start_pt, end_pt, prev_valid_idx, next_valid_idx))
                            
                current_segment_start_idx = -1 # 
            
    max_segment_info=None     
    if segments:
        max_segment_info = max(segments, key=lambda item: item[0])
    return max_segment_info


maxsegmentlength=0
segment_line=None
pipe_image=None
motortimer=0
mtrspd=300
mtrposition=100
motor_timeout=20


def borubuldum(sensor: VsxProtocolDriver.Sensor, line_buffer, segment_buffer, ax):
    global surface_z,x_resolution,pipe_image,surfaceline,is_backward,segment_line,current_linewidth,rect,numofinvalidreadings,maxsegmentlength,pipe_line_points,pipe_segment_info,mtrposition,mtrspd,motor_timeout,motortimer,startpipesearch, pipe_lost_counter
    boruvar=False
   
    ret, container = sensor.GetDataContainer(1000)
    if ret == VsxProtocolDriver.StatusCode.VSX_STATUS_SUCCESS and container is not None:
        # now point cloud data is inside of the calibratedA, calibratedB and calibratedC messages
        ret, lines, metadata = container.GetLine("Line")
        if imagetransfer:
            ret, image, metadata = container.GetImage("Image")
        
        if lines is not None and metadata is not None:
            for line in lines:
                line_points = [{"x": p.x, "z": p.z} for p in line]
                raw_xs = [p["x"] for p in line_points]
                raw_zs = [p["z"] for p in line_points]

            
                if not isValidLine(line_points):
                    numofinvalidreadings+=1
                    if numofinvalidreadings>50:
                        print("UYARI: Cok fazla hatali okuma (Lazer cok yakin veya uzak olabilir).")
                        # exit()
                    continue # Skip this line if majority is invalid
                numofinvalidreadings=0
                
              
                if surface_z==0:
                        x_resolution=calculate_resolution(line_points)
                        current_linewidth = calculate_linewidth(line_points)
                        rounded_zs = [round(z, 1) for x, z in zip(raw_xs, raw_zs) if z != 0 and roi_min_x <= x <= roi_max_x]
                        most_common = Counter(rounded_zs).most_common(1)
                        if most_common:
                            surface_z = most_common[0][0]
                            print(f"surface z: {surface_z},xres: {x_resolution},linewidth: {current_linewidth}")
                
                if surface_z > 0:
                        roi_start_idx = int(len(line_points)/2 - (roi_width/2)/x_resolution)
                        roi_end_idx = int(len(line_points)/2 + (roi_width/2)/x_resolution)
                        
                        # Pre-process: Treat points deeper than holeZlimit as invalid (holes)
                        for k in range(len(line_points)):
                            if raw_zs[k] - surface_z > holeZlimit:
                                raw_zs[k] = 0
                                raw_xs[k] = 0
                        
                        #pepper-salt noise filtering
                        filtered_xs,filtered_zs=filter_valid_noise(raw_xs, raw_zs)
                        
                        # Find Invalid Segments
                        segment_info = find_invalid_segments(filtered_xs, filtered_zs, roi_start_idx, roi_end_idx, surface_z)
                        #Plot
                        ax.clear()
                        ax.scatter(filtered_xs, filtered_zs, c='b', s=1)
                        roi_min_z = surface_z - 5
                        roi_height = 10

                        if startpipesearch and motortimer==0:
                                direction = "" if not is_backward else "-"
                                send_command(f"G1 X{direction}{mtrposition} F{mtrspd}")
                                motortimer=time.perf_counter()
                        if motortimer>0 and time.perf_counter()-motortimer>motor_timeout:
                                motortimer=0
                                
                        if segment_info and startpipesearch:
                            
                            length, start_pt, end_pt, s_idx, e_idx = segment_info
                            if length>pipe_radius*2 and length<pipe_radius*2 + 1:
                                if length>maxsegmentlength:
                                    maxsegmentlength=length
                                    pipe_segment_info=segment_info
                                    pipe_line_points=line_points
                                    if imagetransfer:
                                        pipe_image=image
                                      
                                   
                            elif pipe_segment_info and not debug:
                                currlength = length

                                # 1) Çok küçük segment => gürültü. Asla çıkışa sayma.
                                if currlength < MIN_NOISE_LEN:
                                    # Gürültü say: sayaç artırma
                                    pass
                                else:
                                    # 2) Boru kayıp koşulu: çap bandının altına düştüyse sayaç artır
                                    if currlength < pipe_radius*2 - 1:
                                        pipe_lost_counter += 1
                                    else:
                                        # tekrar normale döndüyse sayaç sıfırla
                                        pipe_lost_counter = 0

                                # 3) N frame üst üste kayıpsa çık
                                if pipe_lost_counter >= PIPE_LOST_FRAMES:
                                    pipe_lost_counter = 0

                                    length, start_pt, end_pt, start_idx, end_idx = pipe_segment_info
                                    scatter_xs = [p["x"] for p in pipe_line_points]
                                    scatter_zs = [p["z"] for p in pipe_line_points]
                                    ax.clear()
                                    ax.scatter(scatter_xs, scatter_zs, c='y', s=5, zorder=10)

                                    boruvar = True
                                    startpipesearch = False
                                    print(f"Pipe Segment: Length={length:.4f}, StartValid={start_pt}, EndValid={end_pt}, "
                                          f"Stopped after {PIPE_LOST_FRAMES} frames, last={currlength:.4f}")
                        # Set fixed limits to prevent autoscaling jitter
                        # Observed Surface Z is approx 252mm. ROI X is approx +/- 8mm.
                        rect = plt.Rectangle((roi_min_x, roi_min_z), roi_width, roi_height, 
                                             linewidth=2, edgecolor='g', facecolor='none', linestyle='--')
                        surfaceline = plt.Line2D([roi_min_x, roi_max_x], [surface_z, surface_z], color='g', linewidth=1)
                        ax.add_patch(rect)
                        ax.add_line(surfaceline)
                        if pipe_segment_info:
                            length, start_pt, end_pt, start_idx, end_idx = pipe_segment_info
                            segment_line = plt.Line2D([start_pt[0], end_pt[0]], [start_pt[1], end_pt[1]], color='r', linewidth=2)
                            ax.add_line(segment_line)   
                            mid_x = (start_pt[0] + end_pt[0]) / 2
                            mid_z = (start_pt[1] + end_pt[1]) / 2
                            circle = plt.Circle((mid_x, mid_z), length/2, color='r', fill=False, linewidth=2)
                            ax.add_patch(circle)
                            ax.text(mid_x, mid_z, f"{length:.2f}", ha='center', va='bottom', fontsize=8, color='black')
                            if boruvar:
                                if imagetransfer:
                                    rotated_image = cv2.rotate(pipe_image, cv2.ROTATE_90_CLOCKWISE)
                                    flipped_image=cv2.flip(rotated_image,1)
                                    cv2.imshow("pipe_image",flipped_image)
                                    cv2.imwrite("pipe_image.png", flipped_image)
                                    
                                    # Run Visualize.py
                                    script_dir = os.path.dirname(os.path.abspath(__file__))
                                    visualize_script = os.path.join(script_dir, "Visualize.py")
                                    subprocess.Popen([sys.executable, visualize_script])
                        ax.set_ylim(surface_z-30, surface_z+30) 
                        ax.set_xlim(-current_linewidth/2, current_linewidth/2)
                        ax.set_title(f"SurfZ: {surface_z:.2f} XRes: {x_resolution:.2f} Width: {current_linewidth:.2f},PipeSearch: {startpipesearch}, Geri: {is_backward}")
                        ax.grid(True)
                        plt.draw()
                        plt.pause(0.1)
                        if boruvar:
                            plt.show()
                           
        else:
            print(f"Grabbing error: {VsxProtocolDriver.Sensor.GetErrorText(ret)}")
   
    return boruvar

if __name__ == "__main__":
    main()
