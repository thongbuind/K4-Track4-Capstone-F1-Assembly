# Báo cáo Capstone: F1 Assembly

Ngày thực hiện: 16/10/2026. Đơn vị liên hệ: Vin Robotics và bài toán lắp ráp linh kiện trong hệ sinh thái VinFast. Trưởng nhóm: **Bùi Đức Thông**. Danh sách nhóm được ghi tại README.

## 1. Bài toán, người dùng và sản phẩm

Người dùng là kỹ sư tự động hóa cần thử chuỗi nhận biết vị trí linh kiện, dự đoán chuyển động, gắp–đặt và kiểm tra thiếu bộ phận trước khi chuyển sang robot thật. Sản phẩm là mô phỏng PyBullet có băng chuyền, tay Franka Panda, camera, tracker và máy QC. Xe F1 thu nhỏ gồm thân xe, cánh trước, cánh sau và bốn bánh dùng làm vật mẫu.

Một demo một tay (`demo_pick`) kiểm chứng vòng gắp–đặt linh kiện đang chạy. Demo dây chuyền (`main`) phối hợp sáu trạm lắp và một camera QC trên băng chính. Tay chuyển xe số 07 chưa triển khai. Các xe sau QC đi tiếp về Surplus, không có tuyến phân loại PASS/FAIL vật lý hoàn chỉnh.

## 2. Kiến thức Track 4 và lý do thiết kế

**Hình học.** `Camera` dựng nội tham số từ FOV, ma trận camera từ view matrix. Pixel OBB được chiếu thành tia rồi giao với mặt phẳng ngang tại chiều cao biết trước của linh kiện. Tracker bù độ lệch tâm OBB của vật không đối xứng. Slot của thân xe được đổi từ hệ cục bộ sang thế giới trước khi tính đích đặt. Cách này cho tọa độ mét có thể dùng cho IK; chỉ bám tâm pixel không đủ xác định vị trí đầu gắp trong thế giới.

**Thời gian.** Physics chạy 240 Hz, điều khiển 60 Hz, camera đầu vào 5 Hz. Tracker lưu timestamp ảnh, dùng tích phân encoder để tính độ dịch chuyển kể từ lần đo, rồi dự đoán pose ở thời điểm điều khiển. Camera mỗi cặp trạm chạy 2 Hz để cập nhật thân xe sau va chạm. Các camera trạm chụp so le. Encoder phù hợp với băng có quỹ đạo biết trước; mô hình đơn giản, dễ giải thích và tạo ca lỗi khi băng trượt. Chưa dùng Kalman và chưa ước lượng bias encoder.

**Bất định và biểu diễn.** Detector tạo OBB bằng PCA từ pixel segmentation và thêm nhiễu tâm 2 px, nhiễu góc 3 độ. Vật bị che quá mức hoặc có quá ít pixel bị bỏ qua. Track biểu diễn bằng tọa độ dọc băng `s`, độ lệch ngang `d`, góc tương đối `psi`, trạng thái và thời điểm đo. Phép đo được pha với dự đoán; đây không phải phân bố xác suất hay bộ lọc covariance đầy đủ. Confidence 0.99 do oracle gán là giá trị mô phỏng, chưa được hiệu chuẩn từ dữ liệu thực.

**Bằng chứng.** `evaluate.py` chạy tiến trình riêng cho mỗi cấu hình, giữ nguyên stdout/stderr trong `evidence/*.txt`, lưu lệnh, thời gian thực và phiên bản thư viện trong `environment.json`. Seed là 0 để dễ tái lập. Đây là kiểm chứng trên một seed, không chứng minh độ ổn định trên toàn bộ điều kiện.

## 3. Phương pháp đo

Chạy `python evaluate.py` từ thư mục gốc sau khi cài requirements. Ba ca: gắp–đặt 60 giây mô phỏng không trượt, gắp–đặt 60 giây trượt 20%, dây chuyền 120 giây. Mặc định dùng grasp-assist; ca dây chuyền dùng stop-and-go và camera trạm. Tốc độ danh định là 0.05 m/s. Tất cả cùng seed 0.

- **Sai số dự đoán tại lúc gắp:** `1000 * hypot(x_tracker-x_true, y_tracker-y_true)`, đơn vị mm; báo trung bình, cực đại và cỡ mẫu. Ground truth chỉ dùng đối chiếu phép đo ở đây. Chỉ những lần phát sinh sự kiện `grasped` mới có mẫu, nên sai số nhỏ không có nghĩa tỷ lệ gắp thành công cao.
- **Số gắp và số thả:** `grasped` và `placed` là số sự kiện. `placed` chỉ xác nhận robot đã thả, không xác nhận đặt chính xác vào slot hay không làm rơi vật.
- **Lắp thành công:** `try_attach` kiểm tra khoảng cách không quá 15 mm, góc không quá 15 độ đối với slot có kiểm tra hướng, sau đó tạo constraint mô phỏng.
- **QC:** camera riêng dò đủ sáu slot trong dung sai 20 mm; đối chiếu với số constraint thật. Tỷ lệ khớp QC không phải accuracy trên phân bố PASS/FAIL cân bằng. Nếu không có xe đến QC, không thể tính tỷ lệ QC.
- **Detector ms/ảnh:** chỉ đo hàm `detect`, không bao gồm render, tracker, điều khiển hay toàn bộ pipeline. Thời gian mô phỏng và thời gian chạy thực là hai đại lượng khác nhau.

## 4. Kết quả thực nghiệm

Hai ca một tay đều chạy 60 giây mô phỏng, seed 0 và sinh 10 linh kiện:

| Chỉ số | [Không trượt](evidence/pick_baseline.txt) | [Trượt 20%](evidence/pick_slip_20pct.txt) |
|---|---:|---:|
| Sự kiện gắp | 9 | 6 |
| Sự kiện thả | 7 | 6 |
| Sự kiện lỗi | 2 | 3 |
| Sai số dự đoán trung bình lúc gắp | **6.6 mm** | **24.3 mm** |
| Sai số lớn nhất | 17.6 mm | 37.9 mm |
| Số mẫu sai số | 9 | 6 |
| Detector trung bình, 300 ảnh/ca | 13.2 ms/ảnh | 14.1 ms/ảnh |
| Thời gian chạy thực | 83.929 s | 78.992 s |

Sai số trung bình có điều kiện trên các lần gắp tăng khoảng **3.68 lần** khi tiêm trượt 20%. Số sự kiện thả/số linh kiện sinh lần lượt là 7/10 (70%) và 6/10 (60%); đây không phải tỷ lệ đặt đúng hình học. Một seed và cỡ mẫu nhỏ chưa đủ kết luận về độ ổn định hay ý nghĩa thống kê. Thời gian thực được lấy từ [environment.json](evidence/environment.json); video có sẵn không được dùng để suy ra số đo.

**Ca dây chuyền 120 giây mô phỏng**, seed 0, stop-and-go, có camera trạm: [log đầy đủ](evidence/assembly_baseline.txt). Sinh 6 thân xe; có 14 lần gắp, 4 lần gắn linh kiện thành công và 9 lần gắn không đạt. Trong 13 lần kiểm tra lắp, tỷ số đạt dung sai là **4/13 ≈ 30.8%**. Sai số dự đoán lúc gắp trung bình 15.2 mm, cực đại 27.1 mm (n=14). Lệch khi thả vào slot trung bình **40.2 mm**, cực đại **116.4 mm** (n=13). Thời gian chạy thực là **336.725 s**.

Chưa có xe đủ sáu bộ phận (`assembled=0`). Số slot đã lắp trên sáu xe còn trong map là `[0,0,1,2,1,0]`. Chỉ một xe đi qua QC, bị đánh FAIL do thiếu cả sáu bộ phận, khớp ground truth 0/6. Mẫu QC 1 xe không đủ để ước lượng accuracy. Một số xe chưa chạy hết dây chuyền khi dừng ở 120 giây, nên không diễn giải số xe hoàn chỉnh như tỷ lệ thành công của toàn bộ sản xuất. Bằng chứng hiện tại chứng minh vòng gắp–đặt một tay hoạt động và các trạm có thể gắn một số linh kiện trong sim, **chưa chứng minh dây chuyền lắp hoàn chỉnh xe ổn định**.

## 5. Phân tích ca lỗi và phép thử tiếp theo

Ca trượt 20% làm vận tốc băng thật là 0.04 m/s trong khi encoder báo quanh 0.05 m/s. Chỉ riêng sai lệch vận tốc tạo chênh lệch tích lũy lý tưởng khoảng 10 mm mỗi giây kể từ phép đo cuối, trước khi xét cập nhật camera, tiếp xúc và va chạm. Đây là cơ chế lỗi được cài trong `Belt.speeds`, không phải kết quả đo sai số thực tế.

Trong [log trượt](evidence/pick_slip_20pct.txt), lần đầu có `pick_miss` tại t=5.73 s; hai lần khác tại 57.23 và 59.68 s. Sai số tại những lần gắp thành công tăng lên 24.3 mm trung bình và 37.9 mm cực đại, phù hợp với hạn chế của dự đoán từ encoder khi băng thật chạy chậm hơn. `pick_miss` được phát khi độ mở kẹp dưới 4 mm hoặc không tìm thấy vật giữa hai ngón; log hiện tại không tách hai nhánh này, nên không khẳng định mọi lần hụt đều chỉ do encoder.

**Ca lỗi đã quan sát:** trong [log cơ sở](evidence/pick_baseline.txt), tại **t=45.07 s** có `grasped`, rồi tại **t=45.22 s** có `dropped_in_lift`; lặp lại tại 57.13/57.27 s. Trong `PickPlaceController.update`, trạng thái LIFT phát lỗi khi độ mở kẹp nhỏ hơn 4 mm, bỏ constraint giữ vật và chuyển sang rút tay. Vì vậy có thể giải thích chắc chắn vì sao bộ điều khiển dừng chu kỳ: điều kiện kiểm tra giữ vật không đạt sau khi gắp. Va chạm lúc nâng, vật lệch trong kẹp hoặc điều kiện độ mở chưa phù hợp với grasp-assist là các giả thuyết cần kiểm chứng thêm bằng log contact/độ mở và replay, chưa phải nguyên nhân vật lý đã được chứng minh.

Bộ đếm `world.dropped=0` không phủ nhận hai lỗi trên: world chỉ đếm vật rơi qua ngưỡng gần sàn, còn `dropped_in_lift` là sự kiện do controller suy luận từ độ mở kẹp. Sai số trung bình 6.6 mm vẫn đi cùng hai lỗi nâng vật, cho thấy một số đo tracking không đủ kết luận toàn bộ thao tác thành công.

**Ca lỗi lắp ráp:** tại t=50.53 s, trạm 02 thả cánh sau lệch **23.7 mm**, góc **2.9 độ**, bị từ chối gắn. Sai số góc nằm trong ngưỡng 15 độ nhưng khoảng cách vượt ngưỡng 15 mm, đủ giải thích kết quả `HỎNG` theo điều kiện `try_attach`. Tại t=81.48 s, trạm 04 có sai lệch vị trí tới 116.4 mm. Các kết quả này cho thấy điều khiển đặt/ước lượng slot là điểm cần cải thiện ngoài tracking lúc gắp. Cần log pose thân xe, pose đầu gắp và va chạm để tách lỗi dự đoán khỏi lỗi điều khiển, thay vì nới dung sai để làm đẹp tỷ lệ lắp.

Hướng cải thiện: cập nhật vật rời bằng camera gần tay; ước lượng bias encoder hoặc hiệu chỉnh vận tốc bằng thị giác; lưu log trạng thái điều khiển và ảnh trước/sau lỗi; kiểm thử nhiều seed, nhiều tốc độ và mức che khuất; đánh giá riêng không dùng grasp-assist. Không suy ra cải thiện khi chưa đo lại.

## 6. Tác động ước lượng và giới hạn sim2real

Ví dụ minh họa cho một trạm thử nghiệm: giả sử kiểm tra thiếu linh kiện cho 500 sản phẩm/ngày đang tốn 5 giây/sản phẩm, hệ thống được kiểm chứng thực tế có thể hỗ trợ giảm còn 2 giây, thì thời gian giảm là `500 * (5-2) = 1500 giây = 25 phút/ngày`, tương đương khoảng 9.17 giờ trong 22 ngày làm việc. Các số 500, 5 giây, 2 giây và 22 ngày là giả định, không phải dữ liệu Vin hay kết quả của mô phỏng. Chưa đo chi phí, recall lỗi thiếu linh kiện, phế phẩm hay ROI.

Giá trị hiện tại là thử thuật toán và tìm lỗi trước phần cứng. Muốn triển khai cần camera được hiệu chuẩn, hand-eye calibration, detector học từ RGB được đánh giá trên dữ liệu thực, thử domain gap ánh sáng/vật liệu/che khuất, mô hình ma sát và tiếp xúc thực, đánh giá chu kỳ sản xuất cùng dừng khẩn cấp và giới hạn lực.

Detector mặc định dùng segmentation/nhãn simulator; tiếp xúc, nhận vật trong kẹp và cơ chế gắn linh kiện cũng dùng trạng thái vật lý của simulator. Dù đích điều khiển mặc định đến từ tracker, hệ thống chưa chứng minh nhận dạng thuần RGB hoặc gắp bằng ma sát hoàn toàn. Không tuyên bố đã đạt độ chính xác công nghiệp hay đã triển khai tại Vin.

## 7. Dữ liệu và bàn giao

Thí nghiệm sinh dữ liệu hình học trong PyBullet, không thu hình bệnh nhân, sinh viên hay người thật. [Video dự phòng](demo_pick_tracker.mp4) dài khoảng 85.67 giây, codec H.264, 2048×1536; [ảnh minh họa](evidence/demo_preview.png) được trích từ video. Video chỉ minh họa demo một tay. [DEMO.md](DEMO.md) cung cấp kịch bản 1 phút bài toán, 4 phút demo, 2 phút bằng chứng/impact và 3 phút hỏi đáp.
