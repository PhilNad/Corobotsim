#The ForceTorqueSensorInterface.so file needs to be on the PYTHON_PATH
import ForceTorqueSensorInterface as ftsi
import threading
import time

class FTSensorInterface:

	force 	= [0,0,0]
	torque 	= [0,0,0]

	#This class represents the actual C++ interface
	class listener(threading.Thread):
		stopOrder = 0
		def __init__(self, forceTorque):
			threading.Thread.__init__(self)
			self.ft = forceTorque
			self.daemon = True  # Allow main to exit even if still running.
			self.paused = True  # Start out paused.
		def run(self):
			#This function does not return
			self.ft.start()
		def stop(self):
			self.ft.stop()
			self.stopped = True

	def __init__(self):
		self.force = [0,0,0]
		self.torque= [0,0,0]
		#This interface is instantiated but not yet started
		self.ft = ftsi.ForceTorque()

		#Starts the recording process in another thread
		self.listenerThread = self.listener(self.ft)
		self.listenerThread.start()

	#The destructor stops the thread
	def __del__(self):
		self.listenerThread.stop()

	def getForce(self):
		self.force[0] = self.ft.getForceX()
		self.force[1] = self.ft.getForceY()
		self.force[2] = self.ft.getForceZ()
		return self.force

	def getTorque(self):
		self.torque[0] = self.ft.getTorqueX()
		self.torque[1] = self.ft.getTorqueY()
		self.torque[2] = self.ft.getTorqueZ()
		return self.torque

	def printFM(self):
		print("Force : X={: .6f} Y={: .6} Z={: .6}".format(self.force[0],self.force[1],self.force[2]))
		print("Torque: X={: .6}  Y={: .6} Z={: .6}".format(self.torque[0],self.torque[1],self.torque[2]))
