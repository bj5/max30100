import max30100
mx30 = max30100.MAX30100()
mx30.refresh_temperature()
temp = mx30.get_temperature()
print('TEMP=%d'%temp)
reg = mx30.get_registers()
mx30.enable_spo2()
mx30.read_sensor()
mx30.ir, mx30.red
