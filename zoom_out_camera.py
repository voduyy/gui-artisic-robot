import cv2

# Mở camera USB
cap = cv2.VideoCapture(1)  # Đổi 0 nếu bạn dùng camera thứ 2, 3...

# Tỷ lệ zoom out (ví dụ: 0.5 = thu nhỏ 50%)
zoom_scale = 0.5

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Thu nhỏ hình ảnh (zoom out)
    height, width = frame.shape[:2]
    small_frame = cv2.resize(frame, (int(width * zoom_scale), int(height * zoom_scale)))

    # Đặt khung ảnh đã thu nhỏ vào giữa màn hình đen (pad thêm cho đủ kích cỡ gốc)
    padded_frame = cv2.copyMakeBorder(
        small_frame,
        top=(height - small_frame.shape[0]) // 2,
        bottom=(height - small_frame.shape[0]) // 2,
        left=(width - small_frame.shape[1]) // 2,
        right=(width - small_frame.shape[1]) // 2,
        borderType=cv2.BORDER_CONSTANT,
        value=[0, 0, 0]
    )

    # Hiển thị
    cv2.imshow("Zoom Out Camera", padded_frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
