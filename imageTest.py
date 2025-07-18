import depthai as dai
import cv2

pipeline = dai.Pipeline()

#カラーカメラノード作成
cam = pipeline.createColorCamera()
cam.setPreviewSize(640,480)
cam.setInterleaved(False)
cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

xout = pipeline.createXLinkOut()
xout.setStreamName("video")
cam.preview.link(xout.input)

#デバイスと接続
with dai.Device(pipeline) as device:
    video = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    while True:
        frame = video.get().getCvFrame()
        cv2.imshow("OAK Camera", frame)
        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWiondows()
