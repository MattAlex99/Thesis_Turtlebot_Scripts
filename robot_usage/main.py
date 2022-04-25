import rospy
rospy.init_node('robot_manager_2', anonymous=False)
import math

import cv2
import camera_management
import movement_management
movement_management.movement_setup()

import manipulation_management
#manipulation_management.setup_manipulator()

import image_analysis
image_analysis.setup_prediction_model()
import time
import matplotlib.pyplot as plt
from helpers.euler_to_quaternians import get_quaternion_from_euler as eul_to_quat

def sort_predictions_by_certainty(predictions):
    result= sorted(predictions, key=lambda x: x.score, reverse=True)
    for box in result:
        box.print_box()
    return result



def _main_():
    print("in main")
    # includes the basic workflow
    ###Initiate camera###
    camera = camera_management.TakePhoto()
    dump_position = [0, 0]  # [x,y]
    #movement_management.resume_patroll()
    ###Repeating Workflow###
    while not rospy.is_shutdown():
        print ("in loop")
        print(movement_management.get_global_pose())
        raw_input("Press a Buton to search for an object")
        #movement_management.patroll()
        image_received_bool, image = camera.take_picture()
        if image_received_bool:
            print ("received image")
            old_time=time.time()
            predictions = image_analysis.predict_on_image(image)
            new_time=time.time()
            print("time needed for analysis", new_time-old_time)

            if len(predictions) > 0:
                print("found one")
                """Deliver Object workflow"""
                ###Get OTM###
                #movement_management.interupt_patroll()
                # extract most likely prediction. recognised OTM will be picked up
                relevant_prediction = sort_predictions_by_certainty(predictions)[0]

                img_coppy=image
                for prediction in predictions:
                    img_coppy = cv2.rectangle(img_coppy, (prediction.xmin, prediction.ymin),
                                              (prediction.xmax, prediction.ymax), (255, 0, 0), 2)

                img_coppy=cv2.rectangle(img_coppy,(relevant_prediction.xmin,relevant_prediction.ymin),(relevant_prediction.xmax,relevant_prediction.ymax),(0,255,0),2)

                OTM_Position_relative = image_analysis.get_relative_OTM_position(
                    relevant_prediction,
                    image,
                    correction=True
                )
                plt.imshow(image)
                plt.show()
                robot_pose = movement_management.get_global_pose()

                #movement_management.navigate_to_OTM(robot_pose, OTM_Position_relative,offset=0.13)
                #time.sleep(5)


                #manipulation_management.grab_object_at(0)

                ###Deliver OTM###
                # move to dump
               # movement_management.move_to_dump()

                #time.sleep(5)
                #manipulation_management.put_down_object_at(0)
            # resume patroll
            #movement_management.resume_patroll()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    #movement_management.move_to_dump()
    _main_()

    """
    OTM_Position_relative={'angle':-45.0,'dist':1}
    quat_x,quat_y,quat_z,quat_w=eul_to_quat(0,0,math.radians(180))
    movement_management.move_to_dump()
   robot_pose={
        'x':0,
        'y':0,
        'z':0,
        'quant_x': quat_x,
        'quant_y': quat_y,
        'quant_z': quat_z,
        'quant_w': quat_w,
    }

    movement_management.goToPose.goto(pos=robot_pose,quat=quaternion_desired)
    #movement_management.navigate_to_OTM(robot_pose, OTM_Position_relative, offset=0.20)
    #time.sleep(5)
    """
# See PyCharm help at https://www.jetbrains.com/help/pycharm/
