from keras3yolo import predict_for_robot
import math
import numpy as np


def setup_prediction_model():
    print("\nSetting up Tensorflow")
    predict_for_robot.setup_prediction_model()
    print("Tensorflow loaded succesfully\n")


def predict_on_image(img):
    return predict_for_robot.predict_on_image(img)


def get_relative_OTM_position(prediction,
                              img,
                              OTM_size=[0.088, 0.045],
                              camera_angle=[111.5, 122],
                              mode="calc",
                              correction=True):
    # mode='calc' --> Berechne aus prediction (ohne Machine Learning)
    if mode == 'calc':
        # TODO get actuall values of x,y,theta
        # rob_x=robot_position[0]
        # rob_y=robot_position[1]
        # rob_theta=robot_position[2]

        # cam_pos_x= rob_x+math.sin(rob_theta*math.pi)/180
        # cam_pos_y=rob_y+math.sin(rob_theta*math.pi)/180
        img_size = [len(img), len(img[0])]
        #print("\n relative OTM pos")
        #print(img_size)
        prediction.print_box()
        OTM_retina_size = [prediction.ymax - prediction.ymin, prediction.xmax - prediction.xmin]
        print(OTM_retina_size)
        if correction:
            OTM_retina_size = reduce_labeling_error(OTM_size, OTM_retina_size)
        print(OTM_retina_size)
        relative_dist = calc_simple_dist(img_size, OTM_size, OTM_retina_size, camera_angle)
        print("dist",relative_dist)

        relative_angle=angle_from_image(img_size[1],(prediction.xmax+prediction.xmin)/2,camera_angle[1])
        print("angle",relative_angle)
        result={'angle':relative_angle,
                'dist':(relative_dist[0]+relative_dist[1])/2}

        return result#relative_dist,relative_angle


def calc_simple_dist(img_size, OTM_size, OTM_retina_size, camera_angle):
    # OTM_size=(OTM_h,OTM_w)= Hohe und breite des Objekes im meter
    # OTM_retine_size=(OTM_r_h,OTM_r_w) = hohe und breite im Bild in Pixel
    # camera_angle=(angle_h,angle_w)=angle size a camera can film
    ###calculate Distance from_height###
    distance_h = OTM_size[0] / (math.tan(camera_angle[0] * math.pi / 180 / 2) * OTM_retina_size[0] / img_size[0])

    ###calculate Distance from_width###
    distance_w = OTM_size[1] / (math.tan(camera_angle[1] * math.pi / 180 / 2) * OTM_retina_size[1] / img_size[1])
    return (distance_h, distance_w)


def reduce_labeling_error(OTM_size, OTM_retina_size):
    # corrects width_height ration of label with known width/height
    # adjusts width to fit to the height
    desired_ratio = float(OTM_size[0]) / float(OTM_size[1])
    current_ratio = float(OTM_retina_size[0]) / float(OTM_retina_size[1])
    helper = current_ratio/desired_ratio
    new_OTM_size= [OTM_retina_size[0]*math.sqrt(helper),OTM_retina_size[1]/math.sqrt(helper)]
    return new_OTM_size


def calc_dist_from_images(delta_d, bbox0, bbox1):
    d1_h = -delta_d / ((bbox0[0] / bbox1[0]) - 1)
    d1_w = -delta_d / ((bbox0[1] / bbox1[1]) - 1)
    return d1_h, d1_w


def tan_deg(deg):
    return math.tan(deg * math.pi / 180)


def atan_deg(deg):
    return math.atan(deg) * 180 / math.pi


def angle_from_image(img_width,OTM_position_x, cammera_angle):
    # OTM_Position is distance centre of box to the middle  in pixel along the horizontal line
    print("values for angle")
    print("img_width:",img_width)
    print("OTM_x:",OTM_position_x)
    print("cam_angle:",cammera_angle)
    delta_x = OTM_position_x - img_width / 2
    print ("delta_x:",delta_x)
    sign=1
    if delta_x<0:
        sign=-1
    delta_x=float(abs(delta_x))
    img_x = img_width / 2
    alpha = cammera_angle / 2
    angle = atan_deg(delta_x * tan_deg(alpha) / img_x)
    #angle=math.atan(delta_x*math.tan(math.degrees(alpha))/img_x)
    print("angle vor anpassung:",angle*sign)
    helper = delta_x / img_x
    angle = angle - angle * ((math.acos(helper)) - 0.55) / 2

    #angle=math.degrees(math.atan((float(delta_x)/img_x)))
    print (delta_x,angle)
    return angle*sign
