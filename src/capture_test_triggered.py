import pyspin as PySpin
import time

# Set camera serial numbers
serial_1 = '19304679'
serial_2 = '19304678'

# Get system
system = PySpin.System.GetInstance()
 
# Get camera list
cam_list = system.GetCameras()
 
# Get cameras by serial
cam_1 = cam_list.GetBySerial(serial_1)
cam_2 = cam_list.GetBySerial(serial_2)
 
# Initialize cameras
cam_1.Init()
cam_2.Init()

cam_1.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)
cam_2.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)

cam_1.TriggerMode.SetValue(PySpin.TriggerMode_Off)
cam_1.TriggerSource.SetValue(PySpin.TriggerSource_Software)
cam_1.TriggerMode.SetValue(PySpin.TriggerMode_On)

cam_2.TriggerMode.SetValue(PySpin.TriggerMode_Off)
cam_2.TriggerSource.SetValue(PySpin.TriggerSource_Software)
cam_2.TriggerMode.SetValue(PySpin.TriggerMode_On)

cam_1.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
cam_2.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)


cam_1.BeginAcquisition()
cam_2.BeginAcquisition()

for x in range(10):
	nodemap1 = cam_1.GetNodeMap()
	node_softwaretrigger_cmd1 = PySpin.CCommandPtr(nodemap1.GetNode("TriggerSoftware"))
	node_softwaretrigger_cmd1.Execute()
	image_1 = cam_1.GetNextImage()

	nodemap2= cam_2.GetNodeMap()
	node_softwaretrigger_cmd2 = PySpin.CCommandPtr(nodemap2.GetNode("TriggerSoftware"))
	node_softwaretrigger_cmd2.Execute()
	image_2 = cam_2.GetNextImage()

	# Acquire images
	#image_1 = cam_1.GetNextImage()
	#image_2 = cam_2.GetNextImage()
	
	# Save images
	image_1.Save(serial_1 + '_' + str(x) +'.png')
	image_2.Save(serial_2 + '_' + str(x) + '.png')
	 
	# Release images
	image_1.Release()
	image_2.Release()
	 
# end acquisition
cam_1.EndAcquisition()
cam_2.EndAcquisition()