# Kịch bản demo 10 phút

1. **0:00–1:00 — Bài toán và người dùng.** Kỹ sư Vin Robotics/VinFast cần kiểm chứng robot gắp linh kiện trên băng chuyển động, lắp đúng slot và phát hiện xe thiếu linh kiện. Mô hình F1 thu nhỏ dùng để thử thuật toán trong sim.
2. **1:00–3:00 — Một tay gắp–đặt.** Chạy `python -m f1_assembly.demo_pick --seconds 60 --seed 0`. Giải thích camera → OBB → tọa độ thế giới → encoder tracker → Panda. Chỉ ra detector oracle và grasp-assist.
3. **3:00–5:00 — Dây chuyền.** Chạy `python -m f1_assembly.main --seconds 120 --seed 0`; chỉ các trạm, trạng thái băng STOP/RUN, log lắp thành công/thất bại và QC. Nếu GUI trục trặc, mở video `demo_pick_tracker.mp4`; video chỉ minh họa vòng một tay. Dùng log thô để trình bày kết quả dây chuyền.
4. **5:00–6:00 — Số đo.** Mở REPORT.md và evidence/. Giải thích sai số Euclid giữa pose tracker với ground truth tại thời điểm gắp; nói rõ mẫu đo có điều kiện trên những lần gắp thành công.
5. **6:00–7:00 — Ca lỗi, trụ cột và impact.** Chỉ dòng lỗi cụ thể trong log, trình bày giả thuyết nguyên nhân và phép thử tiếp theo. Chỉ module hình học/thời gian/bất định. Phép tính tiết kiệm thời gian là giả định minh họa, không phải ROI đã đo tại Vin.
6. **7:00–10:00 — Hỏi đáp.** Vì sao không dùng pose thật để điều khiển? Oracle segmentation khác model RGB ở đâu? Encoder trượt gây sai số thế nào? Tại sao sim chưa đủ chứng minh sim2real? Cần hiệu chuẩn hand-eye, dữ liệu thực, đánh giá detector, mô hình tiếp xúc, giới hạn lực và E-stop trước phần cứng thật.
