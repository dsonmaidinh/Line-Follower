# Line follower STM32

Dự án điều khiển xe robot theo vạch sử dụng vi điều khiển STM32F103 và HAL Library.

## Mô tả

- Hệ thống đọc cảm biến vạch và điều khiển động cơ để xe chạy theo dải vạch đen.
- Firmware được phát triển trong thư mục `firmware`.
- Dự án bao gồm mã nguồn STM32Cube/MDK-ARM và cấu hình phần cứng cơ bản.

## Cấu trúc thư mục

- `firmware/` - mã nguồn chính của dự án.
- `docs/` - thư mục tài liệu, hiện chứa GIF demo.

## Video demo

![Line follower demo](docs/LineFollower.gif)

## Hướng dẫn

1. Mở `firmware/Line_follower.ioc` bằng STM32CubeMX để xem cấu hình phần cứng.
2. Mở `firmware/MDK-ARM/Line_follower.uvprojx` bằng Keil µVision để xây dựng và nạp firmware.

## Chú ý

- Nếu thay đổi cấu hình phần cứng, nhớ cập nhật lại project CubeMX và tái tạo mã.
- Tối ưu hoá PID/dòng điện có thể cần tùy chỉnh thêm tùy từng xe robot cụ thể.
