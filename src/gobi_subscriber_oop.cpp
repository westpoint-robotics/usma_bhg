#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include "XFilters.h" // Xeneth SDK main header.
#include<string>
using namespace std;


class GobiBHG {
    private:
        XCHANDLE handle = 0; // Handle to the camera        
        dword *frameBuffer; // 16-bit buffer to store the capture frame.
        dword frameSize; // The size in bytes of the raw image.
        bool is_initialized;
        int serial_num;
        
    public:
        GobiBHG(){
            frameBuffer = 0; 
            frameSize = 0;  
            is_initialized = false;  
            serial_num = 0;
        }
        
        ~GobiBHG(){
            this->clean_up();
        }
        
        int retrieve_info(){
            ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
            unsigned int deviceCount = 0;
            if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) != I_OK) {
                printf("An error occurred while enumerating the devices. errorCode: %i\n", int(errorCode));
                return -1;
            }
            if (deviceCount == 0) {
                printf("Enumeration was a success but no devices were found!\n");
                return 0;
            }         
            XDeviceInformation *devices = new XDeviceInformation[deviceCount];
            if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached)) != I_OK) {
                printf("Error while retrieving the cached device information structures. errorCode: %i\n", int(errorCode));
                delete [] devices;
                return -1;
            }

            /*  All discovered devices are now available in our local array and we are now able 
             *  to iterate the list and output each item in the array */

            for(unsigned int i = 0; i < deviceCount; i++) {
                XDeviceInformation * dev = &devices[i];
                printf("device[%i] %s @ %s (%s) \n", i, dev->name, dev->address, dev->transport);
                printf("PID: %4X\n", dev->pid); 
                printf("Serial: %i\n", dev->serial);
                printf("URL: %s\n", dev->url);
                printf("State: %s\n\n", dev->state == XDS_Available ? "Available" : dev->state == XDS_Busy ? "Busy" : "Unreachable");
                if (std::string(dev->name).rfind("Gobi") == 0){
                    this->serial_num = dev->serial;                    
                }
            }
            printf("Gobi serial number is: %i\n", this->serial_num);
            delete [] devices;   
            return 0;    
        }
        
        void initialize_cam(){
            // Open a connection to the first detected camera by using connection string cam://0
            printf("Opening connection to cam://0\n");
            this->handle = XC_OpenCamera("cam://0"); 
            if(XC_IsInitialised(handle)){             
                printf("GOBI is initialized.\n");
                this->is_initialized = true;            
            }
            else{
                printf("GOBI initialization failed\n");
                this->is_initialized = false;    
            }
        }
        
        void start_capture(){
            ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
            if ((errorCode = XC_StartCapture(this->handle)) != I_OK)
            {
                printf("Could not start capturing, errorCode: %lu\n", errorCode);
            }
            else if (XC_IsCapturing(this->handle)) // When the camera is capturing ...
            {    
            
                // Load the color profile delivered with this sample.
                if (errorCode = XC_LoadColourProfile(handle, "/home/user1/catkin_ws/src/usma_bhg/resources/ThermalBlue.png") != I_OK)
                {
                    printf("*** Gobi ***: Problem while loading the desired colorprofile, errorCode: %lu\n", errorCode);
                }
                else
                {
                    printf("*** Gobi ***: Successfully loaded the desired colorprofile.\n");
                }        
                // Set the colourmode so that the last loaded colorprofile is used.
                XC_SetColourMode(handle, ColourMode_Profile);
                
                // TODO use the camera calibration file.
                XC_LoadCalibration(this->handle, "pack.xca", XLC_StartSoftwareCorrection);

                // Load thermography filter
                //int thermalFilterId = XC_FLT_Queue(this->handle, "Thermography", 0);

                // Determine native framesize.
                //this->frameSize = XC_GetFrameSize(this->handle);
                this->frameSize = XC_GetWidth(handle) * XC_GetHeight(handle); // currently = 307200 which is 640x480

                // Initialize the 16-bit buffer.
                //this->frameBuffer = new word[this->frameSize / 2];  
                this->frameBuffer = new dword[this->frameSize];  
            }
        }
        
        void retrieve_frame(){
            ErrCode errorCode = 0; 
            for(int cnt = 0; cnt < 100; cnt++) {
                //if ((errorCode = XC_GetFrame(this->handle, FT_NATIVE, XGF_Blocking, this->frameBuffer, this->frameSize)) != I_OK)
                if ((errorCode = XC_GetFrame(this->handle, FT_32_BPP_RGBA, XGF_Blocking, this->frameBuffer, this->frameSize * 4)) != I_OK)
                {
                    printf("Problem while fetching frame, errorCode %lu\n", errorCode);
                }
            }

            errorCode = XC_SaveData(this->handle, "output.xpng", XSD_SaveThermalInfo | XSD_Force16);    
        }
        
        void clean_up(){
            ErrCode errorCode = 0; 
            // When the camera is still capturing, ...
            if(XC_IsCapturing(this->handle))
            {
                // ... stop capturing.
                printf("Stop capturing.\n");
                if ((errorCode = XC_StopCapture(this->handle)) != I_OK)
                {
                    printf("Could not stop capturing, errorCode: %lu\n", errorCode);
                }
            }

            // When the handle to the camera is still initialised ...
            if (XC_IsInitialised(this->handle))
            {
                printf("Closing connection to camera.\n");
                XC_CloseCamera(this->handle);
            }

            printf("Clearing buffers.\n");
            if (this->frameBuffer != 0)
            {
                delete [] this->frameBuffer;
                this->frameBuffer = 0;
            }       
        }
};


int main()
{
    GobiBHG gobi_cam; // instantiate gobi class
    gobi_cam.retrieve_info();
    gobi_cam.initialize_cam();
    gobi_cam.start_capture();
    gobi_cam.retrieve_frame();
    
    return 0;
}
