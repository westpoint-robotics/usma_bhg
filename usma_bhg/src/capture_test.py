import pyspin as PySpin
 
# Set camera serial numbers
serial_1 = '19304679'
serial_2 = '19304678'

# Get system
system = PySpin.System.GetInstance()
 
# Get camera list
cam_list = system.GetCameras()

print("Getting Camera objs")
# Get cameras by serial
cam_1 = cam_list.GetBySerial(serial_1)
cam_2 = cam_list.GetBySerial(serial_2)
 
print("Initializing Cameras")
# Initialize cameras
cam_1.Init()
cam_2.Init()

cam_1.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)
cam_2.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)

cam_1.TriggerMode.SetValue(PySpin.TriggerMode_Off)
cam_2.TriggerMode.SetValue(PySpin.TriggerMode_Off)

cam_1.AcquisitionMode.SetValue(PySpin.AcquisitionMode_SingleFrame)
cam_2.AcquisitionMode.SetValue(PySpin.AcquisitionMode_SingleFrame)

for x in range(10):
	print("")
	print("Loop being, iteration: " + str(x))
	print("Beginning Aq")
	cam_1.BeginAcquisition()
	cam_2.BeginAcquisition()
	
	print("Grabbing Images")
	# Acquire images
	image_1 = cam_1.GetNextImage()
	image_2 = cam_2.GetNextImage()
	 
	print("Writing Images")
	# Save images
	image_1.Save(serial_1 + '_' + str(x) +'.png')
	image_2.Save(serial_2 + '_' + str(x) + '.png')
	 
	# Release images
	image_1.Release()
	image_2.Release()
	 
	print("Ending Aq")
	# end acquisition
	cam_1.EndAcquisition()
	cam_2.EndAcquisition()