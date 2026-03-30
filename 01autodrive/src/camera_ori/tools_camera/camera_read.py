
import cv2
import os
import shutil

def del_file(path_data):
    for i in os.listdir(path_data) :
        file_data = path_data + "/" + i
        if os.path.isfile(file_data) == True:
            os.remove(file_data)
        else:
            del_file(file_data)

path_data = r"camera_ori/tools_camera/picture"
del_file(path_data)
camera = cv2.VideoCapture(-1)
camera.set(cv2.CAP_PROP_FPS, 30)

i = 0
ret, img = camera.read()
print('输入j,下载当前图片')
print('输入q,终止程序')
while ret:

    cv2.imshow('img', img)
    ret, img = camera.read()
    print(img.shape)
    if cv2.waitKey(1) & 0xFF == ord('j'):  # 按j保存一张图片
        i += 1
        firename = str('./picture/img' + str(i) + '.jpg')
        cv2.imwrite(firename, img)
        print('写入：', firename)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()

