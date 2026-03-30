import os
import cv2
import sys
import time
import numpy as np
from ctypes import *
from MvCameraControl_class import *


class HIKCamera():
    def __init__(self):
        self.cam = MvCamera()
        self.stDeviceList = None
        self.nPayloadSize = None
        self.data_buf = None

    def check_env(self):
        """
        检查是否有可用设备
        """
        SDKVersion = MvCamera.MV_CC_GetSDKVersion()
        print("SDKVersion[0x%x]" % SDKVersion)
        
        deviceList = MV_CC_DEVICE_INFO_LIST()
        tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
        
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            return False
            
        if deviceList.nDeviceNum == 0:
            print("find no device!")
            return False
            
        print("find %d devices!" % deviceList.nDeviceNum)
        
        for i in range(0, deviceList.nDeviceNum):
            mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
                print("\ngige device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)
                
                nip1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
                nip2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
                nip3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
                nip4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
                print ("current ip: %d.%d.%d.%d\n" % (nip1, nip2, nip3, nip4))
            
            elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
                print("\nu3v device: [%d]" % i)
                strModeName = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                    if per == 0:
                        break
                    strModeName = strModeName + chr(per)
                print("device model name: %s" % strModeName)

                strSerialNumber = ""
                for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                    if per == 0:
                        break
                    strSerialNumber = strSerialNumber + chr(per)
                print("user serial number: %s" % strSerialNumber)
                
            if deviceList.nDeviceNum == 1:
                nConnectionNum = 0
                print("select device 0")
            
            else:
                print("device number error!")
                return False

            self.stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
            return True


    def init_camera(self):
        """
        初始化相机
        """
        ret = self.cam.MV_CC_CreateHandle(self.stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            return False

        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            return False

        # 从文件导入相机属性
        ret = self.cam.MV_CC_FeatureLoad("FeatureFile.ini")
        if ret != 0:
            print("load camera properties fail! ret [0x%x]" % ret)
        print("finish import the camera properties from the file")
        
        if self.stDeviceList.nTLayerType == MV_GIGE_DEVICE:
            nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize",nPacketSize)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)
                
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            return False

        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        
        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            print("get payload size fail! ret[0x%x]" % ret)
            return False
        self.nPayloadSize = stParam.nCurValue

        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            print("start grabbing fail! ret[0x%x]" % ret)
            return False

        self.stDeviceList = MV_FRAME_OUT_INFO_EX()
        memset(byref(self.stDeviceList), 0, sizeof(self.stDeviceList))
        self.data_buf = (c_ubyte * self.nPayloadSize)()
        return True
    
    def start_camera(self):
        ret1 = self.check_env()
        ret2 = self.init_camera()
        if (ret1 and ret2):
            print("----初始化相机成功----")
            return True
        elif ret1==False and ret2==True:
            print("check_env出错")
        elif ret1==True and ret2==False:
            print("init_camera出错")
        else:
            print("check_env 和 init_camera出错")
        return False
    
    def get_image_BGR(self):
        ret = self.cam.MV_CC_GetOneFrameTimeout(byref(self.data_buf), self.nPayloadSize, self.stDeviceList, 1000)
        if ret == 0:
            
            nRGBSize = self.stDeviceList.nWidth * self.stDeviceList.nHeight * 3
            
            stConvertParam = MV_CC_PIXEL_CONVERT_PARAM()
            memset(byref(stConvertParam), 0, sizeof(stConvertParam))
            stConvertParam.nWidth = self.stDeviceList.nWidth
            stConvertParam.nHeight = self.stDeviceList.nHeight
            stConvertParam.pSrcData = self.data_buf
            stConvertParam.nSrcDataLen = self.stDeviceList.nFrameLen
            stConvertParam.enSrcPixelType = self.stDeviceList.enPixelType  
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed 
            stConvertParam.pDstBuffer = (c_ubyte * nRGBSize)()
            stConvertParam.nDstBufferSize = nRGBSize
            
            ret = self.cam.MV_CC_ConvertPixelType(stConvertParam)
            if ret != 0:
                print("convert pixel fail! ret[0x%x]" % ret)
                del self.data_buf
                return False
            
            img_buff = (c_ubyte * stConvertParam.nDstLen)()
            memmove(byref(img_buff), stConvertParam.pDstBuffer, stConvertParam.nDstLen)
            image_BGR_array = np.array(img_buff)
            image_BGR = image_BGR_array.reshape((self.stDeviceList.nHeight, self.stDeviceList.nWidth, 3))

            return image_BGR
        else:
            print("get one frame fail, ret[0x%x]" % ret)
            
            return False


    def get_image_RGB(self):
        image_BGR = self.get_image_BGR()
        if not image_BGR is None:
            imageRGB = cv2.cvtColor(image_BGR, cv2.COLOR_BGR2RGB)
            return imageRGB
        else:
            return None


    def close_camera(self):
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            del self.data_buf
            sys.exit()
        
        ret = self.cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close device fail! ret[0x%x]" % ret)
            del self.data_buf
            sys.exit()
            
        ret = self.cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            del self.data_buf
            sys.exit()
            
        del self.data_buf

        
if __name__ == "__main__":
    camera = HIKCamera()
    ret1 = camera.check_env()
    ret2 = camera.init_camera()
    
    while (ret1 and ret2):
        start_time = time.time()

        image_BGR = camera.get_image_BGR()
        if not image_BGR is None:
            # output_image = cv2.resize(image_BGR, (800, 600))
            
            cv2.imshow("image", image_BGR)
            cv2.waitKey(1)

        end_time = time.time()
        fps = 1 / (end_time - start_time)
        print("shape", image_BGR.shape)
        print("Collect ok! Current FPS: ", fps)