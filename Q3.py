import cv2
import numpy as np
from skimage.morphology import remove_small_objects

def get_initial_edges(image, canny_low=70, canny_high=170):
    """
    步骤 1: 使用Canny算子获取初始边缘点。
    """
    return cv2.Canny(image, canny_low, canny_high)

def edge_growing(initial_edges, max_iterations=5):
    """
    步骤 2: 迭代“边缘生长”过程。
    """
    grown_edges = initial_edges.copy()
    
    for i in range(max_iterations):
        kernel = np.ones((5, 5), np.uint8)
        closed_edges = cv2.morphologyEx(grown_edges, cv2.MORPH_CLOSE, kernel)
        # cleaned_edges = closed_edges
        cleaned_edges = remove_small_objects(closed_edges.astype(bool), min_size=30).astype(np.uint8) * 255
        
        if np.array_equal(cleaned_edges, grown_edges):
            break
            
        grown_edges = cleaned_edges
        
    return grown_edges

def find_large_regions(edges_image, original_image, min_area=100):
    """
    步骤 3: 在生长后的边缘图中寻找并标识出“大区域”。
    """
    result_image = original_image.copy()
    # 获取原始图像的尺寸，用于对比
    img_h, img_w = original_image.shape[:2]
    inverted_edges = cv2.bitwise_not(edges_image)
    contours, _ = cv2.findContours(inverted_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    large_contours = []
    for contour in contours:
        contour_area = cv2.contourArea(contour)
        
        # --- 过滤条件 1: 按最小面积过滤 ---
        if contour_area < min_area:
            continue
            
        # --- 过滤条件 2: 过滤掉与整个画面一样大的轮廓 (新增加的逻辑) ---
        # 计算轮廓的边界框
        x, y, w, h = cv2.boundingRect(contour)
        # 如果轮廓的宽度或高度非常接近图像的宽度或高度，就认为是边框轮廓，跳过它
        if w > img_w * 0.95 or h > img_h * 0.95:
            continue
            
        large_contours.append(contour)
            
    cv2.drawContours(result_image, large_contours, -1, (0, 255, 0), 2)
    return result_image

def main_video(video_path):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"错误：无法打开视频文件: {video_path}")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("视频播放完毕或读取失败。")
            break

        # --- 对当前帧应用完整的算法流程 ---
        
        frame_resized = resize_image(frame)
        if frame_resized is None:
            continue
            
        frame_gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
        
        initial_edges = get_initial_edges(frame_gray)
        grown_edges = edge_growing(initial_edges)
        
        # 调用修改后的函数
        final_result = find_large_regions(grown_edges, frame_resized)

        # --- 可视化 ---
        initial_edges_bgr = cv2.cvtColor(initial_edges, cv2.COLOR_GRAY2BGR)
        grown_edges_bgr = cv2.cvtColor(grown_edges, cv2.COLOR_GRAY2BGR)

        top_row = np.hstack((frame_resized, initial_edges_bgr))
        bottom_row = np.hstack((grown_edges_bgr, final_result))
        combined_display = np.vstack((top_row, bottom_row))

        cv2.imshow("Edge Growing Segmentation for Video", combined_display)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            print("User requested exit.")
            break

    cap.release()
    cv2.destroyAllWindows()

def resize_image(image, width=400):
    """辅助函数，用于缩放图像以便显示。"""
    if image is None or image.shape[1] == 0: return None
    (h, w) = image.shape[:2]
    r = width / float(w)
    return cv2.resize(image, (width, int(h * r)), interpolation=cv2.INTER_AREA)

if __name__ == '__main__':
    video_file = "PennAir_2024_App_Dynamic_Hard.mp4" 
    main_video(video_file)


