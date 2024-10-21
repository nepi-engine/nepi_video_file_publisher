#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
import numpy as np
import cv2
import open3d as o3d
import random



from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_img


from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_save
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_img 

from nepi_app_file_pub_vid.msg import FilePubVidStatus

from std_msgs.msg import UInt8, Int32, Float32, Empty, String, Bool, Header

from sensor_msgs.msg import Image

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF




#########################################
# Node Class
#########################################

class NepiFilePubVidApp(object):

  HOME_FOLDER = "/mnt/nepi_storage"

  SUPPORTED_FILE_TYPES = ['avi','AVI']

  #Set Initial Values
  MIN_SIZE = 240
  MAX_SIZE = 3700
  STANDARD_IMAGE_SIZES = ['240 x 320', '480 x 640', '630 x 900','720 x 1080','955 x 600','1080 x 1440','1024 x 768 ','1980 x 2520','2048 x 1536','2580 x 2048','3648 x 2736']
  FACTORY_IMG_SIZE = '630 x 900'
  IMG_PUB_ENCODING_OPTIONS = ["bgr8","rgb8","mono8"]
  FACTORY_IMG_ENCODING_OPTION = "bgr8" 

  UPDATER_DELAY_SEC = 1.0
  
  
  paused = False
  last_folder = ""
  current_folders = []
  current_file = 'None'
  last_folder = ""
  current_fps = '0'

  running = False
  file_count = 0
  pub_pub = None

  oneshot = False

  default_size = FACTORY_IMG_SIZE.split('x')
  width = int(default_size[1])
  height = int(default_size[0])

  vidcap = None


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "app_file_pub_vid" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################
    

    ## App Setup ########################################################
    self.initParamServerValues(do_updates=False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)

    # Create class publishers
    self.status_pub = rospy.Publisher("~status", FilePubVidStatus, queue_size=1, latch=True)

    # Start updater process
    rospy.Timer(rospy.Duration(self.UPDATER_DELAY_SEC), self.updaterCb)

    # General Class Subscribers
    rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    rospy.Subscriber('~select_folder', String, self.selectFolderCb)
    rospy.Subscriber('~home_folder', Empty, self.homeFolderCb)
    rospy.Subscriber('~back_folder', Empty, self.backFolderCb)

    # Image Pub Scubscirbers and publishers
    rospy.Subscriber('~set_size', String, self.setSizeCb)
    rospy.Subscriber('~set_encoding', String, self.setEncodingCb)
    rospy.Subscriber('~set_random', Bool, self.setRandomCb)
    rospy.Subscriber('~set_overlay', Bool, self.setOverlayCb) 
    rospy.Subscriber('~start_pub', Empty, self.startPubCb)
    rospy.Subscriber('~stop_pub', Empty, self.stopPubCb)

    rospy.Subscriber('~pause_pub', Bool, self.pausePubCb)
    rospy.Subscriber('~step_forward', Empty, self.stepForwardPubCb)


    time.sleep(1)
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self," Initialization Complete")
    self.publish_status()
    # Spin forever (until object is detected)
    rospy.spin()




  #######################
  ### App Config Functions

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~current_folder', self.HOME_FOLDER)

    rospy.set_param('~size',self.FACTORY_IMG_SIZE)
    rospy.set_param('~encoding',self.FACTORY_IMG_ENCODING_OPTION)

    rospy.set_param('~random',False)
    rospy.set_param('~overaly',False)

    rospy.set_param('~running', False)

    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
    self.init_current_folder = rospy.get_param('~current_folder', self.HOME_FOLDER)

    self.init_size = rospy.get_param('~size',self.FACTORY_IMG_SIZE)
    self.init_encoding = rospy.get_param('~encoding',self.FACTORY_IMG_ENCODING_OPTION)

    self.init_random = rospy.get_param('~random',False)
    self.init_overlay = rospy.get_param('~overlay',False)
    self.init_running = rospy.get_param('~running', False)

    self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
    rospy.set_param('~current_folder', self.init_current_folder)

    rospy.set_param('~size',self.init_size)
    rospy.set_param('~encoding',self.init_encoding)

    rospy.set_param('~random',self.init_random)
    rospy.set_param('~overlay',  self.init_overlay)
    rospy.set_param('~running',self.init_running)

    if do_updates:
      self.updateFromParamServer()
      self.publish_status()

  ###################
  ## Status Publisher
  def publish_status(self):
    status_msg = FilePubVidStatus()

    status_msg.home_folder = self.HOME_FOLDER
    current_folder = rospy.get_param('~current_folder', self.init_current_folder)
    status_msg.current_folder = current_folder
    if current_folder == self.HOME_FOLDER:
      selected_folder = 'Home'
    else:
      selected_folder = os.path.basename(current_folder)
    status_msg.selected_folder = selected_folder
    status_msg.current_folders = self.current_folders
    status_msg.supported_file_types = self.SUPPORTED_FILE_TYPES
    status_msg.file_count = self.file_count
    status_msg.current_file =  self.current_file
    status_msg.current_fps = self.current_fps

    status_msg.paused = self.paused

    status_msg.size_options_list = self.STANDARD_IMAGE_SIZES
    status_msg.set_size = rospy.get_param('~size',self.init_size)
    status_msg.encoding_options_list = self.IMG_PUB_ENCODING_OPTIONS
    status_msg.set_encoding = rospy.get_param('~encoding',self.init_encoding)
    status_msg.set_random = rospy.get_param('~random',self.init_random)
    status_msg.set_overlay = rospy.get_param('~overlay',  self.init_overlay)

    status_msg.running = rospy.get_param('~running',self.init_running)

    self.status_pub.publish(status_msg)


  #############################
  ## APP callbacks

  def updaterCb(self,timer):
    update_status = False
    # Get settings from param server
    current_folder = rospy.get_param('~current_folder', self.init_current_folder)
    #nepi_msg.publishMsgWarn(self,"Current Folder: " + str(current_folder))
    #nepi_msg.publishMsgWarn(self,"Last Folder: " + str(self.last_folder))
    # Update folder info
    if current_folder != self.last_folder:
      update_status = True
      if os.path.exists(current_folder):
        #nepi_msg.publishMsgWarn(self,"Current Folder Exists")
        current_paths = nepi_ros.get_folder_list(current_folder)
        current_folders = []
        for path in current_paths:
          folder = os.path.basename(path)
          if folder[0] != ".":
            current_folders.append(folder)
        self.current_folders = sorted(current_folders)
        #nepi_msg.publishMsgWarn(self,"Folders: " + str(self.current_folders))
        num_files = 0
        for f_type in self.SUPPORTED_FILE_TYPES:
          num_files = num_files + nepi_ros.get_file_count(current_folder,f_type)
        self.file_count =  num_files
      self.last_folder = current_folder
    # Start publishing if needed
    running = rospy.get_param('~running',self.init_running)
    if running and self.pub_pub == None:
      self.startPub()
      update_status = True
    # Publish status if needed
    if update_status == True:
      self.publish_status()

  def selectFolderCb(self,msg):
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    new_folder = msg.data
    new_path = os.path.join(current_folder,new_folder)
    if os.path.exists(new_path):
      self.last_folder = current_folder
      rospy.set_param('~current_folder',new_path)
    self.publish_status()


  def homeFolderCb(self,msg):
    rospy.set_param('~current_folder',self.HOME_FOLDER)
    self.publish_status()

  def backFolderCb(self,msg):
    current_folder = rospy.get_param('~current_folder',self.init_current_folder)
    if current_folder != self.HOME_FOLDER:
      new_folder = os.path.dirname(current_folder )
      if os.path.exists(new_folder):
        self.last_folder = current_folder
        rospy.set_param('~current_folder',new_folder)
    self.publish_status()


  def pausePubCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    self.paused = msg.data
    self.publish_status()

  def stepForwardPubCb(self,msg):
    if self.paused:
      self.oneshot = True




  #############################
  ## Image callbacks

  def setSizeCb(self,msg):
    new_size = msg.data
    success = False
    try:
      size_list = new_size.split("x")
      h = int(size_list[0])
      w = int(size_list[1])
      success = True
    except Exception as e:
      nepi_msg.publishMsgWarn(self, "Unable to parse size message: " + new_size + " " + str(e) )

    if success:
      if h >= self.MIN_SIZE and h <= self.MAX_SIZE and w >= self.MIN_SIZE and w <= self.MAX_SIZE:
        rospy.set_param('~size',new_size)
        self.width = w
        self.height = h
      else:
        nepi_msg.publishMsgWarn(self, "Received size out of range: " + new_size )
    self.publish_status()

  def setEncodingCb(self,msg):
    new_encoding = msg.data
    if new_encoding in self.IMG_PUB_ENCODING_OPTIONS:
      rospy.set_param('~encoding',new_encoding)
    self.publish_status()

  def setRandomCb(self,msg):
    ##nepi_msg.publishMsgInfo(self,msg)
    rospy.set_param('~random',msg.data)
    self.publish_status()

  def setOverlayCb(self,msg):
      ##nepi_msg.publishMsgInfo(self,msg)
      overlay = msg.data
      rospy.set_param('~overlay',overlay)
      self.publish_status()

  def startPubCb(self,msg):
    self.startPub()

  def startPub(self):
    if self.pub_pub == None:
      self.pub_pub = rospy.Publisher("~images", Image, queue_size=1, latch=True)
      time.sleep(1)
      current_folder = rospy.get_param('~current_folder', self.init_current_folder)
      # Now start publishing images
      self.file_list = []
      self.num_files = 0
      if os.path.exists(current_folder):
        for f_type in self.SUPPORTED_FILE_TYPES:
          [file_list, num_files] = nepi_ros.get_file_list(current_folder,f_type)
          self.file_list.extend(file_list)
          self.num_files += num_files
          #nepi_msg.publishMsgWarn(self,"File Pub List: " + str(self.file_list))
          #nepi_msg.publishMsgWarn(self,"File Pub Count: " + str(self.num_files))
        if self.num_files > 0:
          self.current_ind = 0
          rospy.Timer(rospy.Duration(1), self.publishCb, oneshot = True)
          running = True
          rospy.set_param('~running',True)
        else:
          nepi_msg.publishMsgInfo(self,"No image files found in folder " + current_folder)
      else:
        nepi_msg.publishMsgInfo(self,"Folder " + current_folder + " not found")
    self.publish_status()


  def stopPubCb(self,msg):
    running = rospy.get_param('~running',self.init_running)
    running = False
    rospy.set_param('~running',False)
    time.sleep(1)
    if self.pub_pub != None:
      self.pub_pub.unregister()
      time.sleep(1)
      self.pub_pub = None
    self.current_file = "None"
    self.current_fps = "0"
    self.publish_status()


  def publishCb(self,timer):
    running = rospy.get_param('~running',self.init_running)

    if running:
      if self.pub_pub != None:
        # Set current index
        if set_random == True and self.paused == False:
          self.current_ind = int(random.random() * self.num_files)
        else:
          self.current_ind = self.current_ind + 1
        # Check ind bounds
        if self.current_ind > (self.num_files-1):
          self.current_ind = 0 # Start over
        elif self.current_ind < 0:
          self.current_ind = self.num_files-1
        file2open = self.file_list[self.current_ind]
        self.current_file = file2open.split('/')[-1]
        #nepi_msg.publishMsgInfo(self,"Opening File: " + file2open)
        if os.path.isfile(file2open):
          nepi_msg.publishMsgInfo(self,"Opening File: " + file2open)
          self.vidcap = cv2.VideoCapture(file2open)
          if self.vidcap.isOpened() == True:
            success,image = self.vidcap.read()
            shape_str = str(image.shape)
            nepi_msg.publishMsgInfo(self,'Image size: ' + shape_str)
            fps = self.vidcap.get(5)
            self.current_fps = str(round(fps, 2))
            nepi_msg.publishMsgInfo(self,'Frames per second : ' + self.current_fps)

            frame_count = self.vidcap.get(7)
            nepi_msg.publishMsgInfo(self,'Frame count : ' + str(frame_count))

            cv2_img = None
            while success == True and running == True and not rospy.is_shutdown():
                running = rospy.get_param('~running',self.init_running)
                size = rospy.get_param('~size',self.init_size)
                encoding = rospy.get_param('~encoding',self.init_encoding)
                set_random = rospy.get_param('~random',self.init_random)
                overlay = rospy.get_param('~overlay',  self.init_overlay)
                if cv2_img is None or self.paused == False or self.oneshot == True:
                  self.oneshot = False
                  # Publish video at native fps
                  success,cv2_img = self.vidcap.read()
                  if success == False:
                    self.vidcap.release()
                    time.sleep(1)
                    self.vidcap = None
                  else: 
                    cv2_img = cv2.resize(cv2_img,(self.width,self.height))
                    # Overlay Label
                    if overlay == True:
                      # Overlay text data on OpenCV image
                      font                   = cv2.FONT_HERSHEY_DUPLEX
                      fontScale, thickness  = nepi_img.optimal_font_dims(cv2_img,font_scale = 1.5e-3, thickness_scale = 1.5e-3)
                      fontColor = (0, 255, 0)
                      lineType = 1
                      text2overlay=self.current_file
                      bottomLeftCornerOfText = (int(self.width*.05),int(self.height*.1))
                      cv2.putText(cv2_img,text2overlay, 
                          bottomLeftCornerOfText, 
                          font, 
                          fontScale,
                          fontColor,
                          thickness,
                          lineType)
                  if success:
                    # Publish new image to ros
                    img_shape = cv2_img.shape     
                    if encoding == 'mono8' and img_shape[2] == 3:
                      cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
                    if encoding != 'mono8' and img_shape[2] == 1:
                      cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_GRAY2BGR)
                    out_img_msg = nepi_img.cv2img_to_rosimg(cv2_img,encoding=encoding)
                    if not rospy.is_shutdown():
                      out_img_msg.header.stamp = rospy.Time.now()
                      self.pub_pub.publish(out_img_msg) 

    running = rospy.get_param('~running',self.init_running)
    if running == True:
      rospy.Timer(rospy.Duration(1), self.publishCb, oneshot = True)
    else:
      if self.vidcap != None:
        self.vidcap.release()
        time.sleep(1)
        self.vidcap = None
      self.current_ind = 0
      if self.pub_pub != None:
        self.pub_pub.unregister()
        time.sleep(1)
        self.pub_pub = None









               
    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self," Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiFilePubVidApp()







