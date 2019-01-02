import numpy as np
import cv2
import datetime
import nnabla as nn
import nnabla.functions as F
import nnabla.parametric_functions as PF
import numpy as np
import sys
import os

CAPTURE = False
RECOGNITION = False
CAP_NUM = 0
print(sys.argv)
if len(sys.argv) >= 2:
    if sys.argv[1] == "capture":
        print("Emily: capture mode")
        CAPTURE = True
        CAP_NUM = int(sys.argv[2])
        FOLDER = sys.argv[3]
        PATH_TO_SAVE = "./training_data/" + FOLDER
    elif sys.argv[1] == "recognition":
        RECOGNITION = True
else:
    RECOGNITION = True
        

        
SIZE = 64

def draw_bounding_box():
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        rect = cv2.boundingRect(c)
        if rect[2] < 100 or rect[3] < 100: continue
        print (cv2.contourArea(c))
        x,y,w,h = rect
        cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)


def network(x, test=False):
    # Input:x -> 3,64,64

    # ImageAugmentation
    h = F.image_augmentation(x, (3,64,64), (0,0), 1, 1, 0, 1, 0, True, True, 0, False, 1, 0.5, False, 0)
    # Convolution -> 8,58,58
    h = PF.convolution(h, 8, (7,7), (0,0), name='Convolution')
    # ReLU
    h = F.relu(h, True)
    # MaxPooling -> 8,29,29
    h = F.max_pooling(h, (2,2), (2,2))
    # Convolution_2 -> 10,27,27
    h = PF.convolution(h, 10, (3,3), (0,0), name='Convolution_2')
    # MaxPooling_2 -> 10,13,13
    h = F.max_pooling(h, (2,2), (2,2))
    # Tanh
    h = F.tanh(h)
    # Affine -> 3
    h = PF.affine(h, (3,), name='Affine')
    # ReLU_2
    h = F.relu(h, True)
    # Affine_2
    h = PF.affine(h, (3,), name='Affine_2')
    # Softmax
    h = F.softmax(h)
    return h

x = nn.Variable((1,3,SIZE,SIZE))
y = network(x, test=True)
nn.load_parameters("./parameters.h5")
result_class = "can't identify"
cascPath = "../opencv/data/haarcascades/haarcascade_frontalface_alt.xml"
faceCascade = cv2.CascadeClassifier(cascPath)
print (faceCascade)

def pre_processing_frame(frame_in, width, height):
    #resize
    frame_resize = cv2.resize(frame_in, (width, height))
    #channel swap
    frame_resize = frame_resize[:,:,(2,1,0)]
    frame_resize = frame_resize.transpose(2,0,1)
    frame_resize = frame_resize*1.0/255
    return frame_resize

def classification(img_in):
    frame_processed = pre_processing_frame(img_in, SIZE, SIZE)
    processed_size = frame_processed.shape[:2]
    print(processed_size)
    height, width, channel = img_in.shape
    x.d = frame_processed
    forward = y.forward()
    result_array = y.d[0]
    index = np.unravel_index(result_array.argmax(), result_array.shape)
    print("result array ", result_array)
    max_index = index[0]
    prob = result_array[max_index]
    print("result_array.argmax = ", result_array.argmax())
    if max_index == 0:
        result_class = "Hello Carlos"
        print(result_class)
    elif max_index == 1: 
        result_class = "Hello Emily"
        print(result_class)
    else: 
        result_class = "can't identify"
    return result_class, str(prob)

def detection(img_in):
    # ToDo: convert the color channel
    height, width, channel = img_in.shape
    print("img_shape" + "height = ", height, "width = ", width, "channel = ", channel)

    frame_processed = pre_processing_frame(img_in, width, height)
    processed_size = frame_processed.shape
    print(processed_size)
    
    rows = height/64
    cols = width/64
    print("rows = ", rows, "cols = ", cols)
    # init an empty arrat for for the prediction
    detections = np.zeros((rows, cols))

    for i in range(0, rows):
        for j in range(0, cols):
            grid_square = frame_processed[:,i*64:(i+1)*64, j*64:(j+1)*64]
            x.d = grid_square
            forward = y.forward()
            detections[i,j] = y.d[0].argmax()
            print("i, j, result = ", i, j,detections[i,j] )
    print(detections)
    change_pixel_colour(img_in, detections)

def change_pixel_colour(img, detections):
    for i in range(detections.shape[0]):
        for j in range(detections.shape[1]):
            if int(detections[i, j]) ==0 | int(detections[i, j])==1:
                for pixel_i in range(i*64, (i+1)*64):
                    for pixel_j in range(j*64, (j+1)*64):
                        img[pixel_i, pixel_j,:] = [100,0,0]
        


def cv2_face_tracking(frame):
        faces = faceCascade.detectMultiScale(frame)
        # Draw a rectangle around the faces
        for (rec_x, rec_y, rec_w, rec_h) in faces:
            cv2.rectangle(frame, (rec_x, rec_y), (rec_x+rec_w, rec_y+rec_h), (0, 255, 0), 2)


cap = cv2.VideoCapture(0)

while(True):
    #capture frame by frame
    ret, frame = cap.read()

    #Our operations on the frame come here
    #gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    font = cv2.FONT_ITALIC
    x_axis = 10 #position of text
    y_axis = 20 #position of text
   
    frame_to_process = frame.copy()

    if CAPTURE:
        for i in range(CAP_NUM):
            frame_to_process = cv2.resize(frame_to_process, (SIZE, SIZE))
            if os.path.exists(PATH_TO_SAVE) == False:
                os.mkdir(PATH_TO_SAVE)
            file_name = PATH_TO_SAVE + '_opencv'+str(i)+'.png'
            cv2.imwrite(file_name, frame_to_process)
            print("saved img to ", file_name)
        cap.release()
        cv2.destoryAllWindows()

    if RECOGNITION:
        original_size = frame_to_process.shape[:2]
        print(original_size)

        #result = classification(frame)
        #cv2_face_tracking(frame)
        detection(frame)
        #date_time =  datetime.datetime.now()
        #datetime_str = date_time.strftime('%d %H:%M:%S')
        #text_to_display = datetime_str + "\n " + result[0] + "\n probability: " + result[1]
        #cv2.putText(frame, text_to_display, (x_axis,y_axis), font, 0.8, 255) 
    #Draw the text

    #display the reulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destoryAllWindows()

