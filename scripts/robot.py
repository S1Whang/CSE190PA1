#!/usr/bin/env python

import rospy
from read_config import read_config
from cse_190_assi_1.srv import requestMapData
from cse_190_assi_1.srv import requestTexture
from cse_190_assi_1.srv import moveService
from cse_190_assi_1.msg import temperatureMessage
from cse_190_assi_1.msg import RobotProbabilities
from std_msgs.msg import Bool, String, Float32
class Robot():
    def __init__(self):
        self.current_move = 0
        self.config = read_config()
        rospy.init_node("Robot")
        self.move_requester = rospy.ServiceProxy(
            "moveService", moveService)
        self.texture_requester = rospy.ServiceProxy(
            "requestTexture", requestTexture)
        self.temperature_service = rospy.Subscriber(
            "/temp_sensor/data", temperatureMessage,
            self.handle_temperature_message)
        self.activation_publisher = rospy.Publisher(
            "/temp_sensor/activation", Bool, queue_size = 10)
        self.temperature_results = rospy.Publisher(
            "/results/temperature_data", Float32, queue_size = 10)
        self.texture_results = rospy.Publisher(
            "/results/texture_data", String, queue_size = 10)
        self.final_results = rospy.Publisher(
            "/map_node/sim_complete", Bool, queue_size = 10)
        self.prob_results = rospy.Publisher(
            "/results/probabilities", RobotProbabilities,
            queue_size = 10)
        self.rows = len(self.config['pipe_map'])
        self.cols = len(self.config['pipe_map'][0])
        self.prob = 1.0/(self.rows*self.cols)
        print self.rows, self.cols, self.prob ,self.cols*self.rows
        self.baymap = [[self.prob for x in range(len(self.config
            ['pipe_map'] [0]))]for y in range(len(self.config
            ['pipe_map']))]
        self.calculate_textures()
        self.calculate_temperatures()
        self.temp_activate_sensor = False
        rospy.sleep(3)
        self.rate = rospy.Rate(5)
        while not self.temp_activate_sensor:
            self.activation_publisher.publish(True)
            self.rate.sleep()
        rospy.spin()
    def handle_temperature_message(self, message):
        """Callback Function for Temperature Readings"""
        if not self.temp_activate_sensor: 
            self.temp_activate_sensor = True
        self.text_response = self.texture_requester() 
        print "Temp: ", message.temperature
        self.calculate_textures_new()
        self.temperature_results.publish(message.temperature)
        self.texture_results.publish(self.text_response.data)
        if self.current_move >= len(self.config['move_list']):
            self.shut_down = self.final_results.publish(True)
            rospy.sleep(3)
            rospy.signal_shutdown("Robot Going To Sleep")
        self.move_response = self.move_requester(
            self.config['move_list'][self.current_move])
        print "Move: ", self.config['move_list'][self.current_move]
        self.current_move += 1
    def calculate_textures(self):
        self.rough_old = self.rough_new = 0.0
        self.smooth_old = self.smooth_new = 0.0
        self.total = 0.0
        for i in self.config['texture_map']:
            for j in i:
                self.total += 1.0
                if j == "R": self.rough_old += 1.0
                if j == "S": self.smooth_old += 1.0
        print self.rough_old, self.smooth_old
        self.rough_new = self.rough_old/self.total
        self.smooth_new = self.smooth_old/self.total
    def calculate_textures_new(self):
        p_tex = self.config['prob_tex_correct']
        t_map = self.config['texture_map']
        for i in range(self.rows):
            for j in range(self.cols):
                if t_map[i][j] == self.text_response.data:
                    self.baymap[i][j] = p_tex * self.baymap[i][j] 
                else:
                    self.baymap[i][j] = (1-p_tex) * self.baymap[i][j]
        print "Text: ", self.text_response.data
    def calculate_temperatures(self):
        self.hot = 0
        self.mid = 0
        self.cold = 0
        for i in self.config['pipe_map']:
           for j in i:
               if j == 'H': self.hot += 1
               if j == 'C': self.cold += 1
               if j == '-': self.mid += 1
        self.tot = self.hot + self.mid + self.cold
        print self.hot, self.cold, self.mid
        print (self.hot*40.0+self.cold*20.0 + self.mid * 25.0)/self.tot
        mu = self.config['temp_noise_std_dev']
    def calculate_move(self):
        p_move = self.config['prob_move_correct']
    def sum_list(self, l):
        total = 0.0
        for i in range(len(l)):
            for j in range(len(l[i])):
                total += l[i][j]
        return total
 
if __name__ == '__main__':
    robo = Robot()
