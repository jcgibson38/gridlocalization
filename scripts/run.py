#!/usr/bin/env python
import rospy
import rosbag
import rospkg
import numpy as np
from tf.transformations import euler_from_quaternion
from scipy.stats import norm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

rospack = rospkg.RosPack()
path = rospack.get_path('lab4')
bag = rosbag.Bag(path+'/res/lab4/grid.bag')
topics=['Movements','Observations']

rospy.init_node('main',anonymous=True)
pub = rospy.Publisher("visualization_msgs", Marker, queue_size=5)
curid = 0
prevp = (11,27,2)
# tags #
tagdict = {0:(1.25,5.25),1:(1.25,3.25),2:(1.25,1.25),3:(4.25,1.25),4:(4.25,3.25),5:(4.25,5.25)}

# Initialize belief #
rows = 7./0.2
cols = 7./0.2
lays = 360./90.
mybelief = np.zeros((rows,cols,lays)) # 35x35x4
# Initial p distribution #
initx = 11
inity = 27
inittheta = 2 # 200.52 deg
mybelief[initx,inity,inittheta] = 1.
myupdatedbelief = np.copy(mybelief)
# Noise #
sigmax = 0.1
sigmay = 0.1
sigmatheta = 45

def lowlim(x):
    r = 2
    if x-r < 0:
        return 0
    else:
        return x-r

def uplim(x):
    r = 2
    if x+r > 34:
        return 34
    else:
        return x+r

def getinds(x):
    index = np.unravel_index(np.argmax(x),x.shape)
    return index[0],index[1],index[2]

def publishnode(x,prev):
    global pub,curid
    curid += 1
    linelist = Marker()
    linelist.header.frame_id = "/my_frame"
    linelist.header.stamp = rospy.Time.now()
    linelist.ns = "basic_line"
    linelist.action = Marker.ADD
    linelist.pose.orientation.w = 1.0
    linelist.id = curid
    linelist.type = Marker.LINE_LIST
    linelist.scale.x = 0.1
    linelist.color.r = 1.0
    linelist.color.a = 1.0

    rospy.loginfo(prev)
    startpoint = Point()
    startpoint.x = prev[0]*0.2+0.1
    startpoint.y = prev[1]*0.2+0.1
    startpoint.z = 0

    rospy.loginfo(x)
    endpoint = Point()
    endpoint.x = x[0]*0.2+0.1
    endpoint.y = x[1]*0.2+0.1
    endpoint.z = 0

    linelist.points.append(startpoint)
    linelist.points.append(endpoint)
    pub.publish(linelist)


for topic,msg,t in bag.read_messages(topics=topics):
    mybelief[:,:,:] = myupdatedbelief[:,:,:]
    ### We need to update our belief to reflect this motion ###
    if topic == 'Movements':
        # Get rotation 1 #
        X = msg.rotation1.x
        Y = msg.rotation1.y
        Z = msg.rotation1.z
        W = msg.rotation1.w
        quaternion = (X,Y,Z,W)
        rot1 = np.degrees(euler_from_quaternion(quaternion)[2]) # degrees

        # Get rotation 2 #
        X = msg.rotation2.x
        Y = msg.rotation2.y
        Z = msg.rotation2.z
        W = msg.rotation2.w
        quaternion = (X,Y,Z,W)
        rot2 = np.degrees(euler_from_quaternion(quaternion)[2]) # degrees

        # Get translation #
        trans = msg.translation

        ind1,ind2,ind3 = getinds(mybelief)

        # We need to loop through all cells to update belief #
        for xdex1 in range(lowlim(ind1),uplim(ind1),1):
            for ydex1 in range(lowlim(ind2),uplim(ind2),1):
                for tdex1 in range(4):
                    # loop through all potential starting cells #
                    newprob = 0.
                    for xdex2 in range(lowlim(xdex1),uplim(xdex1),1):
                        for ydex2 in range(lowlim(ydex1),uplim(ydex1),1):
                            for tdex2 in range(4):
                                # calculate [x,y,theta] #
                                x = xdex2*0.2 + 0.1
                                y = ydex2*0.2 + 0.1
                                theta = tdex2*90. + 45.
                                # calculate [xp,yp,thetap] #
                                xp = x+trans*np.cos(np.radians(theta+rot1))
                                yp = y+trans*np.sin(np.radians(theta+rot1))
                                thetap = theta+rot1+rot2
                                # calculate prob density #
                                mydistx = norm(xp,sigmax)
                                mydisty = norm(yp,sigmay)
                                mydistt = norm(thetap,sigmatheta)
                                # CDF limits #
                                xstart = xdex1*0.2
                                xend = (xdex1+1)*0.2
                                ystart = ydex1*0.2
                                yend = (ydex1+1)*0.2
                                thetastart = tdex1*90
                                thetaend = (tdex1+1)*90

                                newprob += ((mydistx.cdf(xend)-mydistx.cdf(xstart))*(mydisty.cdf(yend)-mydisty.cdf(ystart))*(mydistt.cdf(thetaend)-mydistt.cdf(thetastart)))*mybelief[xdex2,ydex2,tdex2]
                    # Update belief #
                    myupdatedbelief[xdex1,ydex1,tdex1] = newprob
        s = 1./np.sum(myupdatedbelief)
        myupdatedbelief *= s

    ### We need to update belief based on obersvation ###
    if topic == 'Observations':
        ind1,ind2,ind3 = getinds(mybelief)
        # get bearing #
        X = msg.bearing.x
        Y = msg.bearing.y
        Z = msg.bearing.z
        W = msg.bearing.w
        quaternion = (X,Y,Z,W)
        bear = np.degrees(euler_from_quaternion(quaternion)[2]) # degrees

        # range #
        r = msg.range

        # tag id #
        tagid = msg.tagNum

        # We need to loop through all possible states #
        for xdex1 in range(35):
            for ydex1 in range(35):
                for tdex1 in range(4):
                    # calc [x,y,theta] #
                    x = xdex1*0.2 + 0.1
                    y = ydex1*0.2 + 0.1
                    theta = (tdex1*90. + 45.)
                    # calc where landmark should be if I am here at [x,y,theta] #
                    xp = x+r*np.cos(np.radians(theta+bear))
                    yp = y+r*np.sin(np.radians(theta+bear))

                    # Get the prob distribution centered at this landmark #
                    tagpdfx = norm(tagdict[tagid][0],sigmax)
                    tagpdfy = norm(tagdict[tagid][1],sigmay)

                    # Get the prob of measuring xp,yp from x,y,theta #
                    prob = tagpdfx.pdf(xp)*tagpdfy.pdf(yp)
                    # update belief #
                    myupdatedbelief[xdex1,ydex1,tdex1] = prob*mybelief[xdex1,ydex1,tdex1]
        s = 1./np.sum(myupdatedbelief)
        myupdatedbelief *= s
        # Publish to RVIZ #
        inds = getinds(myupdatedbelief)
        publishnode(inds,prevp)
        prevp = inds
