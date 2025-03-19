import rclpy

from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32MultiArray

class ThermalCameraPublisher(Node):
    def __init__(self):
        super().__init__('thermal_camera_publisher')
        self.publisher_img = self.create_publisher(Image, 'thermal_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.device_id = 1 # Default camera device ID 後ほど動的変数に変更
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L)  # Open the thermal camera
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
        self.bridge = CvBridge()

        if not self.cap.isOpened():
            self.get_logger().error('Could not open thermal camera')
            raise RuntimeError('Could not open thermal camera')

    def timer_callback(self):
        #256x192 General settings
        width = 256 #Sensor width
        height = 192 #sensor height
        scale = 3 #scale multiplier
        newWidth = width*scale 
        newHeight = height*scale
        alpha = 1.0 # Contrast control (1.0-3.0)
        colormap = 0
        font=cv2.FONT_HERSHEY_SIMPLEX
        dispFullscreen = False
        cv2.namedWindow('Thermal',cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow('Thermal', newWidth,newHeight)
        rad = 0 #blur radius
        threshold = 2
        hud = True
        recording = False
        elapsed = "00:00:00"
        snaptime = "None"

        # Capture frame-by-frame
        ret, frame = self.cap.read()
        if ret == True:
            imdata,thdata = np.array_split(frame, 2)
            #now parse the data from the bottom frame and convert to temp!
            #https://www.eevblog.com/forum/thermal-imaging/infiray-and-their-p2-pro-discussion/200/
            #Huge props to LeoDJ for figuring out how the data is stored and how to compute temp from it.
            #grab data from the center pixel...
            hi = thdata[96][128][0]
            lo = thdata[96][128][1]
            #print(hi,lo)
            lo = lo*256
            rawtemp = hi+lo
            #print(rawtemp)
            temp = (rawtemp/64)-273.15
            temp = round(temp,2)
            #print(temp)
            #break

            #find the max temperature in the frame
            lomax = thdata[...,1].max()
            posmax = thdata[...,1].argmax()
            #since argmax returns a linear index, convert back to row and col
            mcol,mrow = divmod(posmax,width)
            himax = thdata[mcol][mrow][0]
            lomax=lomax*256
            maxtemp = himax+lomax
            maxtemp = (maxtemp/64)-273.15
            maxtemp = round(maxtemp,2)

            
            #find the lowest temperature in the frame
            lomin = thdata[...,1].min()
            posmin = thdata[...,1].argmin()
            #since argmax returns a linear index, convert back to row and col
            lcol,lrow = divmod(posmin,width)
            himin = thdata[lcol][lrow][0]
            lomin=lomin*256
            mintemp = himin+lomin
            mintemp = (mintemp/64)-273.15
            mintemp = round(mintemp,2)

            #find the average temperature in the frame
            loavg = thdata[...,1].mean()
            hiavg = thdata[...,0].mean()
            loavg=loavg*256
            avgtemp = loavg+hiavg
            avgtemp = (avgtemp/64)-273.15
            avgtemp = round(avgtemp,2)

            

            # Convert the real image to RGB
            bgr = cv2.cvtColor(imdata,  cv2.COLOR_YUV2BGR_YUYV)
            #Contrast
            bgr = cv2.convertScaleAbs(bgr, alpha=alpha)#Contrast
            #bicubic interpolate, upscale and blur
            bgr = cv2.resize(bgr,(newWidth,newHeight),interpolation=cv2.INTER_CUBIC)#Scale up!
            if rad>0:
                bgr = cv2.blur(bgr,(rad,rad))

            #apply colormap
            if colormap == 0:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_JET)
                cmapText = 'Jet'
            if colormap == 1:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_HOT)
                cmapText = 'Hot'
            if colormap == 2:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_MAGMA)
                cmapText = 'Magma'
            if colormap == 3:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_INFERNO)
                cmapText = 'Inferno'
            if colormap == 4:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_PLASMA)
                cmapText = 'Plasma'
            if colormap == 5:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_BONE)
                cmapText = 'Bone'
            if colormap == 6:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_SPRING)
                cmapText = 'Spring'
            if colormap == 7:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_AUTUMN)
                cmapText = 'Autumn'
            if colormap == 8:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_VIRIDIS)
                cmapText = 'Viridis'
            if colormap == 9:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_PARULA)
                cmapText = 'Parula'
            if colormap == 10:
                heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_RAINBOW)
                heatmap = cv2.cvtColor(heatmap, cv2.COLOR_BGR2RGB)
                cmapText = 'Inv Rainbow'


            # 		# draw crosshairs
            # cv2.line(heatmap,(int(newWidth/2),int(newHeight/2)+20),\
            # (int(newWidth/2),int(newHeight/2)-20),(255,255,255),2) #vline
            # cv2.line(heatmap,(int(newWidth/2)+20,int(newHeight/2)),\
            # (int(newWidth/2)-20,int(newHeight/2)),(255,255,255),2) #hline

            # cv2.line(heatmap,(int(newWidth/2),int(newHeight/2)+20),\
            # (int(newWidth/2),int(newHeight/2)-20),(0,0,0),1) #vline
            # cv2.line(heatmap,(int(newWidth/2)+20,int(newHeight/2)),\
            # (int(newWidth/2)-20,int(newHeight/2)),(0,0,0),1) #hline
            # #show temp
            # cv2.putText(heatmap,str(temp)+' C', (int(newWidth/2)+10, int(newHeight/2)-10),\
            # cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(heatmap,str(temp)+' C', (int(newWidth/2)+10, int(newHeight/2)-10),\
            # cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0, 255, 255), 1, cv2.LINE_AA)

            # if hud==True:
            #     # display black box for our data
            #     cv2.rectangle(heatmap, (0, 0),(160, 120), (0,0,0), -1)
            #     # put text in the box
            #     cv2.putText(heatmap,'Avg Temp: '+str(avgtemp)+' C', (10, 14),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     cv2.putText(heatmap,'Label Threshold: '+str(threshold)+' C', (10, 28),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     cv2.putText(heatmap,'Colormap: '+cmapText, (10, 42),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     cv2.putText(heatmap,'Blur: '+str(rad)+' ', (10, 56),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     cv2.putText(heatmap,'Scaling: '+str(scale)+' ', (10, 70),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     cv2.putText(heatmap,'Contrast: '+str(alpha)+' ', (10, 84),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)


            #     cv2.putText(heatmap,'Snapshot: '+snaptime+' ', (10, 98),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.4,(0, 255, 255), 1, cv2.LINE_AA)

            #     if recording == False:
            #         cv2.putText(heatmap,'Recording: '+elapsed, (10, 112),\
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.4,(200, 200, 200), 1, cv2.LINE_AA)
            #     if recording == True:
            #         cv2.putText(heatmap,'Recording: '+elapsed, (10, 112),\
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.4,(40, 40, 255), 1, cv2.LINE_AA)
            
            # #Yeah, this looks like we can probably do this next bit more efficiently!
            # #display floating max temp
            # if maxtemp > avgtemp+threshold:
            #     cv2.circle(heatmap, (mrow*scale, mcol*scale), 5, (0,0,0), 2)
            #     cv2.circle(heatmap, (mrow*scale, mcol*scale), 5, (0,0,255), -1)
            #     cv2.putText(heatmap,str(maxtemp)+' C', ((mrow*scale)+10, (mcol*scale)+5),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0,0,0), 2, cv2.LINE_AA)
            #     cv2.putText(heatmap,str(maxtemp)+' C', ((mrow*scale)+10, (mcol*scale)+5),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0, 255, 255), 1, cv2.LINE_AA)

            # #display floating min temp
            # if mintemp < avgtemp-threshold:
            #     cv2.circle(heatmap, (lrow*scale, lcol*scale), 5, (0,0,0), 2)
            #     cv2.circle(heatmap, (lrow*scale, lcol*scale), 5, (255,0,0), -1)
            #     cv2.putText(heatmap,str(mintemp)+' C', ((lrow*scale)+10, (lcol*scale)+5),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0,0,0), 2, cv2.LINE_AA)
            #     cv2.putText(heatmap,str(mintemp)+' C', ((lrow*scale)+10, (lcol*scale)+5),\
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.45,(0, 255, 255), 1, cv2.LINE_AA)


            # 画像のパブリッシュ
            img_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding="bgr8")
            self.publisher_img.publish(img_msg)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()