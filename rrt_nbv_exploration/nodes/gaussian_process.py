#!/usr/bin/env python3
# BSD 3-Clause License
#
# Copyright (c) 2019, Magnus Selin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import numpy as np
from collections import namedtuple
import rospy
from rrt_nbv_exploration_msgs.msg import Tree
from rrt_nbv_exploration_msgs.msg import Node
from rrt_nbv_exploration_msgs.srv import RequestGp,RequestGpResponse
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class GaussianProcess:
    def __init__(self):
        self.s = rospy.Service('rne/requestGp', RequestGp, self.query_server)        
        self.mean_pub = rospy.Publisher('mean_markers', MarkerArray, queue_size=10)
        
        self.tree_sub = rospy.Subscriber('rne/rrt_tree', Tree, self.tree_callback)
        
        self.min = [-10, -10, 0]
        self.max = [ 10,  10, 3]
        self.resolution      = rospy.get_param('~visualize/resolution',   1)
        
        HyperParam = namedtuple("HyperParam", "l sigma_f sigma_n")
        self.hyperparam = HyperParam(l = 1, sigma_f = 1, sigma_n = 0.1)
    
    def sqexpkernel(self, x1, x2, hyperparam):
        n1 = x1.shape[0]
        n2 = x2.shape[0]
        K = np.empty([n1, n2])
        #print(n1, n2)
    
        for i in range(0,n2):
            #print(x1)
            #print(x2[i,:])
            #print(x1 - x2[i,:])
            l = np.linalg.norm(x1 - x2[i,:], axis = 1)
            #print(l)
            K[:,i] = hyperparam.sigma_f**2 * np.exp(-0.5 * (l / hyperparam.l)**2)
    
        return K
    
    def gp(self, y, x, xstar, hyperparam, kernel):
        if(y.shape[0] is 0 or x.shape[0] is 0):
            return (np.empty((0)), np.empty((0)))
    
        K     = kernel(x, x, hyperparam)            # K(x,x)
        kstar = kernel(x, xstar, hyperparam)        # K(x,x*)
        kss   = kernel(xstar, xstar, hyperparam)    # K(x*,x*)
    
        # Algorithm 2.1 from Rasmussen & Williams
        L = np.linalg.cholesky(K + hyperparam.sigma_n**2*np.identity(K.shape[0]))
        alpha = np.linalg.solve( np.transpose(L), np.linalg.solve(L, y))
        posterior_mean = np.matmul(np.transpose(kstar), alpha)
    
        v = np.linalg.solve(L, kstar)
        posterior_variance = np.diag(kss - np.matmul(np.transpose(v), v))
    
        return posterior_mean, posterior_variance
    
    """ Receive Tree message and give frontiers to GP visualization """
    def tree_callback(self, msg):
        self.evaluate(msg.frontiers)
        
    """ Handle query to Gaussian Process """
    def query_server(self, req):
        y = np.empty((0))
        x = np.empty((0,3))

        for frontier in req.frontiers:
            y = np.append(y, [frontier.gain], axis=0)
            x = np.append(x, [[frontier.position.x, frontier.position.y, frontier.position.z]], axis = 0)
            
        if y.shape[0] == 0:
            response = QueryResponse()
            response.mu = 0
            response.sigma = 1
            return response

        xstar = np.array([[req.position.x, req.position.y, req.position.z]])

        mean, sigma = self.gp(y, x, xstar, self.hyperparam, self.sqexpkernel)

        response = RequestGpResponse()
        response.mu = mean
        response.sigma = sigma

        return response

    """ Evaluate potential information gain function over grid and publish it in rviz """
    def evaluate(self, frontiers):
        if self.mean_pub.get_num_connections() > 0:
            y = np.empty((0))
            x = np.empty((0,3))
            xstar = np.empty((0,3))
            for frontier in frontiers:
                y = np.append(y, [frontier.gain], axis=0)
                x = np.append(x, [[frontier.position.x, frontier.position.y, frontier.position.z]], axis = 0)
    
            xt = np.arange(self.min[0], self.max[0], self.resolution)
            yt = np.arange(self.min[1], self.max[1], self.resolution)
            zt = [1]
    
            for xx in xt:
                for yy in yt:
                    for zz in zt:
                        xstar = np.append(xstar, [[xx, yy, zz]], axis = 0)
    
            mean, sigma = self.gp(y, x, xstar, self.hyperparam, self.sqexpkernel)
            mean_markers = MarkerArray()
            for id, pts in enumerate(zip(xstar, mean, sigma)):
                mean_markers.markers.append(self.np_array_to_marker(id, pts[0], pts[1], max(1-pts[2], 0)))
    
            self.mean_pub.publish(mean_markers)
        
    def np_array_to_marker(self, id, p, v=0, a=0):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = 0.1
        marker.color.r = v / 72.0
        marker.color.g = 0 
        marker.color.b = 0.5
        marker.color.a = a
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = p[0]
        marker.pose.position.y = p[1]
        marker.pose.position.z = p[2]
        marker.lifetime = rospy.Time(10)

        return marker
        
if __name__ == '__main__':
    rospy.init_node('gaussian_process', anonymous=True)
    gaussianProcess = GaussianProcess()
    rospy.spin()