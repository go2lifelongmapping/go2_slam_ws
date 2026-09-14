# Định dạng điểm riêng của Livox.
# File này CHỈ tồn tại để thoả mãn #include của FAST-LIO.
# Dùng Ouster thì không message nào ở đây được publish cả.
uint32 offset_time      # thời gian lệch so với timebase (ns)
float32 x               # trục X, đơn vị m
float32 y               # trục Y, đơn vị m
float32 z               # trục Z, đơn vị m
uint8 reflectivity      # độ phản xạ 0~255
uint8 tag               # tag riêng của Livox
uint8 line              # chỉ số tia laser
