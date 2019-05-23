from FTSensorInterface import FTSensorInterface
import time

ftsi = FTSensorInterface()

for i in range(5):
	time.sleep(1)
	print("{} :".format(i+1))
	print(ftsi.getForce())
	print(ftsi.getTorque())
