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
        

        
SIZE =32

def draw_bounding_box():
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        rect = cv2.boundingRect(c)
        if rect[2] < 100 or rect[3] < 100: continue
        print (cv2.contourArea(c))
        x,y,w,h = rect
        cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)

def network(x, test=False):
    # Input:x -> 1,32,32
    # ImageAugmentation
    h = F.image_augmentation(x, (1,32,32), (0,0), 1, 1, 0, 1, 0, False, False, 0, False, 1, 0.5, False, 0)
    # Convolution -> 16,28,28
    h = PF.convolution(h, 16, (5,5), (0,0), name='Convolution')
    # ReLU
    h = F.relu(h, True)
    # MaxPooling -> 16,14,14
    h = F.max_pooling(h, (2,2), (2,2))
    # Convolution_2 -> 30,12,12
    h = PF.convolution(h, 30, (3,3), (0,0), name='Convolution_2')
    # MaxPooling_2 -> 30,6,6
    h = F.max_pooling(h, (2,2), (2,2))
    # Tanh_2
    h = F.tanh(h)
    # Affine_2 -> 2
    h = PF.affine(h, (2,), name='Affine_2')
    # Softmax
    h = F.softmax(h)
    return h

x = nn.Variable((1,1,SIZE,SIZE))
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
    #frame_resize = frame_resize[:,:,(2,1,0)]
    #frame_resize = frame_resize.transpose(2,0,1)
    frame_resize = frame_resize*1.0/255
    return frame_resize

def classification(img_in):
    frame_processed = pre_processing_frame(img_in, SIZE, SIZE)
    processed_size = frame_processed.shape[:2]
    print(processed_size)
    height, width = img_in.shape
    x.d = frame_processed
    forward = y.forward()
    result_array = y.d[0]
    index = np.unravel_index(result_array.argmax(), result_array.shape)
    print("result array ", result_array)
    max_index = index[0]
    prob = result_array[max_index]
    print("result_array.argmax = ", result_array.argmax())
    if max_index == 0:
        result_class = "Hello a face"
        print(result_class)
    elif max_index == 1: 
        result_class = "Hello something else"
        print(result_class)
    return result_class, str(prob)

def detection(img_in):
    # ToDo: convert the color channel
    # input gray, no colour channel
    #height, width, channel = img_in.shape
    #print("img_shape" + "height = ", height, "width = ", width, "channel = ", channel)

    height, width = img_in.shape
    print("img_shape" + "height = ", height, "width = ", width, "channel = ")
    frame_processed = pre_processing_frame(img_in, width, height)
    processed_size = frame_processed.shape
    print(processed_size)
    
    rows = height/32
    cols = width/32
    print("rows = ", rows, "cols = ", cols)
    # init an empty arrat for for the prediction
    detections = np.zeros((rows, cols))

    for i in range(0, rows):
        for j in range(0, cols):
            grid_square = frame_processed[i*32:(i+1)*32, j*32:(j+1)*32]
            x.d = grid_square
            forward = y.forward()
            detections[i,j] = y.d[0].argmax()
            print("i, j, result = ", i, j,detections[i,j] )
    print(detections)
    bounding_box = countering(detections)
    print("bounding box", bounding_box)
    #drawing_mask(img_in, bounding_box)
    mask_without_countering(img_in, detections)

def drawing_mask(img, bounding_box):
    for i in range(bounding_box[0],bounding_box[2]):
        for j in range(bounding_box[1], bounding_box[3]):
            img[i,j] = 0

def countering(detections):
    count = 0
    bounding_box = [0,0,0,0]
    for i in range(detections.shape[0]):
        for j in range(detections.shape[1]):
            if detections[i,j] == 0:
                if count == 0:
                    bounding_box[0] = i*32
                    bounding_box[1]= j*32
                    count += 1
                else:
                    bounding_box[2] = i*32
                    bounding_box[3] = j*32
    return bounding_box

def mask_without_countering(img,detections):
    for i in range(detections.shape[0]):
        for j in range(detections.shape[1]):
            if detections[i, j] == 0:
                for pixel_i in range(i*32, (i+1)*32):
                    for pixel_j in range(j*32, (j+1)*32):
                        img[pixel_i, pixel_j] = 0
               

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
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    font = cv2.FONT_ITALIC
    x_axis = 10 #position of text
    y_axis = 20 #position of text
   
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
        original_size = gray.shape[:2]
        print(original_size)

        #result = classification(gray)
        #cv2_face_tracking(gray)
        detection(gray)
        date_time =  datetime.datetime.now()
        datetime_str = date_time.strftime('%d %H:%M:%S')
        text_to_display = datetime_str  
        cv2.putText(gray, text_to_display, (x_axis,y_axis), font, 0.8, 255) 
    #Draw the text

    #display the reulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destoryAllWindows()

