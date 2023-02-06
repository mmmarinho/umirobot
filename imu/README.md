1. Install the `FastIMU` library using the Arduino Library Manager. I tested it with library version `1.0.3`.

2. Upload the `fast_imu.ino` sketch to your Arduino that reads the `MPU6500`. It is based on one of the examples with only a small modification.

3. You can test the communication with the computer running `_imu_glove_comm.py`. Remember to adjust the port to match you Arduino's COM port.

4. You can run the `_imu_filter.py` file together with a CoppeliaSim compatible scene such as `scenes/imu_frame_test.ttt` to check the IMU's orientation readings. 