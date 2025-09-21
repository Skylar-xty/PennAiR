# PennAiR

# Running Instructions
For Q1,
```bash
python Q1.py --input PennAir_2024_App_Static.png --output annotatedQ1.png
```
For Q2,
```bash
python Q2.py --input PennAir_2024_App_Dynamic.mp4 \
             --output annotatedQ2.mp4 \
             --csv detections.csv \
             --preview 1   
```    
For Q3,
```bash
python Q3.py
```
For Q4,
```bash
python Q4.py --input PennAir_2024_App_Static.png --output annotatedQ4.png
```
For Q5,
```bash
cd ros1_ws
catkin_make
source devel/setup.bash
roslaunch detector_ros shape_detector.launch
```
# Results
├── annotatedQ1.png  
├── annotatedQ2.mp4  
├── annotatedQ3.mp4  
├── annotatedQ4.png  
├── Q5.png
# Q1

## 1. Load Image & Convert
- Read the input image.
- Convert from **BGR → HSV** (HSV is more robust for color segmentation).

## 2. Build Color Masks
- Define HSV ranges for:
  - Red (two separate bands, later merged)
  - Yellow
  - Blue
  - Magenta
  - Bright green (tuned to avoid grassy background)
- Apply `cv2.inRange` to get binary masks.

## 3. Clean Masks
- Use morphological operations:
  - **Close** (fill small holes).
  - **Open** (remove speckles).
- Kernel: elliptical (7×7).

## 4. Merge Red Masks
- Combine the two red bands into a single **red** mask.

## 5. Find Candidate Blobs
- Extract contours using `cv2.findContours`.
- Ignore very small blobs (`area < 600`).

## 6. Classify Shape
- Compute perimeter (`arcLength`) and polygon approximation (`approxPolyDP`).
- Based on number of vertices:
  - 3 → triangle  
  - 4 → quadrilateral  
  - 5 → pentagon  
- Otherwise, compute **circularity**:
  
  $$
  C = \frac{4 \pi \cdot \text{Area}}{\text{Perimeter}^2}
  $$
  
  - If `C > 0.75` → circle  
  - Else → generic polygon  
- Additional filter: ignore blobs with `area < 200`.

## 7. Locate Centers
- Compute centroid from image moments (`m10/m00`, `m01/m00`).
- Fallback: min enclosing circle center.

## 8. Annotate
- Draw white contour.
- Draw a red dot at the centroid.
- Add text label with shape type.

## 9. Output
- Save annotated image.
- Print results (sorted left → right by x-coordinate):
  - Shape type
  - Center coordinates
  - Area
  - Color mask used

---

**Pipeline Summary:**  
Color segmentation (HSV) → Noise cleanup → Contour extraction → Shape classification (geometry + circularity) → Centroid marking → Annotate & report.

# Q2
The only difference from Q1 is retrieving resolution and FPS to configure `VideoWriter`, and convert each frame to HSV for robust color segmentation.

# Q3
This algorithm combines **Canny edges**, an iterative **edge-growing process**, and **region filtering** to isolate meaningful shapes while ignoring noise or full-frame borders.  

The results are displayed in a 2×2 grid per frame:
- Top-left: original frame (resized)  
- Top-right: initial edges (Canny)  
- Bottom-left: grown/cleaned edges  
- Bottom-right: final large-region detection  



## 1. Grayscale Conversion
- Input frame → grayscale (for edge detection).

## 2. Initial Edge Detection (Canny)
- Apply `cv2.Canny` with thresholds `(70, 170)`.
- Produces a binary edge map of potential boundaries.

## 3. Edge Growing (Iterative Morphology + Cleanup)
- Iteratively refine edges up to `max_iterations = 5`:
    - Apply **morphological CLOSE** with a 5×5 kernel (fills small gaps).
    - Remove small connected components (`min_size=30`) using `skimage.morphology.remove_small_objects`.
    - Stop early if no further changes occur.
- Output: a more continuous, denoised edge map.

## 4. Large Region Detection
- Invert the edge map (so objects are filled white).
- Find external contours with `cv2.findContours`.
- Filter out unwanted contours:
    - **Too small**: area `< 100`.  
    - **Too large**: bounding box width/height > 95% of frame (removes border-like contours).
- Draw remaining contours in **green** on a copy of the original frame.

## 5. Visualization
- Convert edge maps to BGR for side-by-side display.
- Show all stages in a combined window.

## 6. Interaction
- Press **`q`** to quit video playback.  


# Q4
The main image process is the same as Q1 and Q2, the difference is the depth estimation.

## 1. First Pass (Collect Candidates)
- Store each detection: shape, mask color, center `(cx,cy)`, area, and contour.  
- For circles, also record enclosing circle radius in pixels (used for depth).  

## 2. Depth Plane Estimation
- If at least one circle is found:  
    - Pick the **largest circle** (most stable).  
    - Compute plane depth using:
        
  $$
  Z = \frac{f_{\text{mean}} \cdot D_{\text{real}}}{d_{\text{px}}}
  $$
  
    where:  
    - $` f_{\text{mean}} = (f_x+f_y)/2 `$  
    - $` D_{\text{real}} = 20\ \text{in} `$  (known circle diameter)  
    - $` d_{\text{px}} = 2r_{px} `$  (diameter in pixels)  
- If no circle is found: depth `Z` is unknown.  

## 3. Back-Projection to 3D
- For each shape center `(u,v)` and plane depth `Z`:
    $$
    X = \frac{(u-c_x)}{f_x} \cdot Z, \quad
    Y = \frac{(v-c_y)}{f_y} \cdot Z, \quad
    Z = Z
    $$ 
- Output `(X,Y,Z)` in inches.  

## 4. Second Pass (Annotation)
- Draw contours (white) and center dots (red).  
- Add text label:  
    - If depth is known: `"shape  (X,Y,Z) in"`.  
    - Otherwise: `"shape"`.  
- Overlay a note with estimated plane depth or `"unknown (no circle)"`.  

## 5. Console Output
- Print all detections sorted left to right (by `cx`):  
    shape type, pixel center, area, and originating mask.  

## 6. Save Result
- Save annotated image to `--output`.  

# Q5

## 1. Nodes in Use
- video_streamer.py
    - Role: Publisher
    - Responsibility: Read the video file frame-by-frame and publish each frame as a ROS message. It continuously sends data.
- shape_detector_node.py
    - Role: Subscriber and Publisher
    - Responsibility:
        1. It subscribes to the `/camera/image_raw` topic to receive the raw video frames.
        2. After processing each frame, it publishes two different kinds of results: An annotated image to the `/camera/image_annotated` topic. Structured detection data to the `/shape_detections` topic.
##  2. Messages in Use
- `sensor_msgs/Image`
- `detector_ros/ShapeDetection.msg`
    - Acts as a building block for the `ShapeDetections` message.
- `detector_ros/ShapeDetections.msg`
