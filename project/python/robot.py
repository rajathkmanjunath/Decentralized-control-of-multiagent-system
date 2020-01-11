import numpy as np
import pybullet as p
import itertools
# import math
# from controller import controller

class Robot():
    """
    The class is the interface to a single robot
    """
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()
        self.square_done = False
        self.move_out = False
        self.circle_done = False
        self.v_done = False
        self.v_side = False
        self.v_up = False
        self.circle_form = False
        self.purple_place = False
        self.v_back = False
        self.v_down = False
        self.v_right = False
        self.v_inverse = False
        self.v_inverse_down = False
        self.form_circle = False
        self.circle_up = False
        self.circle_left = False
        self.red_place = False
        self.v_transform = False
        self.v_final_down = False
        self.v_final_right = False
        self.back_in = False
        self.form_rhombus = False

        # No friction between bbody and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)
        self.prev_position = [0,0,0]
        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)
        self.static = False
        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
        self.obstacles = []
        self.desired_positions_square= 1*np.array([1.,1.0,1.,-1.0,0.,1.,0.,-1.,-1.,1.0,-1.,-1.0])
        self.desired_positions_circle = 0.5*np.array([np.cos(np.pi/3),np.sin(np.pi/3),1.,0.,-np.cos(np.pi/3),np.sin(np.pi/3),np.cos(np.pi/3),-np.sin(np.pi/3),-1.,0,-np.cos(np.pi/3),-np.sin(np.pi/3)])
        # self.desired_positions_line = 1*np.array([4.,0.,4.,1.,4.,2.,4.,3.,4.,4.,4.,5.])
        self.desired_positions_v = 0.4*np.array([0.,0.,1/np.sqrt(2),-1/np.sqrt(2),-1.0,0.,np.sqrt(2),-np.sqrt(2),-1-1/np.sqrt(2),-1/np.sqrt(2),-1-np.sqrt(2),-np.sqrt(2)])
        self.desired_positions_v_inv = 0.4*np.array([1+np.sqrt(2),np.sqrt(2),1+1/np.sqrt(2),1/np.sqrt(2),-np.sqrt(2),np.sqrt(2),1,0,-1/np.sqrt(2),1/np.sqrt(2),0.,0.])
        self.desired_positions_rhombus = 1*np.array([1.,1.0,1.5,-np.cos(np.pi/6),0.,1.,0.5,-np.cos(np.pi/6),-1.,1.0,-.5,-np.cos(np.pi/6)])
        self.count = 0

        for i in np.arange(-2,2,0.1):
            self.obstacles.append([0,i])
            self.obstacles.append([3,i])

        for i in np.arange(0,2,0.1):
            self.obstacles.append([i,2])

        for i in np.arange(1,3,0.1):
            self.obstacles.append([i,-2])

        # for i in np.arange(1.8,2.2):
        #     for j in np.arange(3.8,4.2):
        #         self.obstacles.append([i,j])

        self.obstacles = np.asarray(self.obstacles)


    def reset(self):
        """
        Moves the robot back to its initial position
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))

    def set_wheel_velocity(self, vel):
        """
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel)
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
            targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]

    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received

    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])

    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors


    def is_static(self, position):
        if(np.linalg.norm(self.prev_position - position)<10e-5):
            self.static = True

        else:
            self.static = False

    def compute_controller(self):
        """
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED

        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """
        # desired_graph = np.array([1, .5**2, .5**2, 1, .5**2, .5**2, 2, 2, 1])
        # edges = np.array([[0,1],[1,3],[3,5],[4,5],[0,2],[2,4],[0,3],[1,4],[2,5]])

        # desired_positions = np.array([0,0,0,0,0,0,0,0,0,0])
        # desired_positions_line = 0.5*np.array([0.1,1,0.2,2,0.3,3,0.4,4,0.5,5,0.6,6])
        # desired_graph = np.zeros(9)
        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()
        gain = 5
        potential_feild = [4,5.5,0.3]
        potential_feild1 = [4,3,0.3]
        potential_feild2 = [2,3,0.3]
        potential_feild3 = [2,4,0.3]
        potential_feild4 = [2.5,5.5,0.3]
        potential_feild5 = [4,3.5,0.3]
        potential_feild6 = [4,2,0.3]
        potential_feild7 = [0.5,3,0.3]
        potential_feild8 = [0.5,5.5,0.3]
        potential_feild9 = [2.5,4.5,0.3]
        potential_feild10 = [2.5,0,0.3]
        potential_feild11 = [1.5,0,0.3]
        feild_distance1 = 6
        feild_distance2 = 2
        min_distance = 0.4
        kr = 30
        ko = 10
        ks = 3
        kc = 5
        kf = 5
        kv = 7
        #send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)

        # check if we received the position of our neighbors and compute desired change in position
        # as a function of the neighbors (message is composed of [neighbors id, position])
        if(not self.square_done):
            dx = 0
            dy = 0


        elif(not self.move_out):
            dx = gain*(potential_feild[0] - pos[0])
            dy = gain*(potential_feild[1] - pos[1])

        elif(not self.circle_done):
            dx = 0
            dy = 0

        elif(not self.v_done):
            dx = kv*(potential_feild1[0] - pos[0])
            dy = kv*(potential_feild1[1] - pos[1])

        elif(not self.v_side):
            dx = kv*(potential_feild2[0] - pos[0])
            dy = kv*(potential_feild2[1] - pos[1])

        elif(not self.v_up):
            dx = kv*(potential_feild3[0] - pos[0])
            dy = kv*(potential_feild3[1] - pos[1])

        elif(not self.circle_form):
            dx = kv*(potential_feild3[0] - pos[0])
            dy = kv*(potential_feild3[1] - pos[1])

        elif(not self.purple_place):
            dx = kv*(potential_feild4[0] - pos[0])
            dy = kv*(potential_feild4[1] - pos[1])

        elif(not self.v_back):
            dx = kv*(potential_feild4[0] - pos[0])
            dy = kv*(potential_feild4[1] - pos[1])

        elif(not self.v_down):
            dx = kv*(potential_feild3[0] - pos[0])
            dy = kv*(potential_feild3[1] - pos[1])

        elif(not self.v_right):
            dx = kv*(potential_feild5[0] - pos[0])
            dy = kv*(potential_feild5[1] - pos[1])

        elif(not self.v_inverse):
            dx = kv*(potential_feild5[0] - pos[0])
            dy = kv*(potential_feild5[1] - pos[1])

        elif(not self.v_inverse_down):
            dx = kv*(potential_feild6[0] - pos[0])
            dy = kv*(potential_feild6[1] - pos[1])

        elif(not self.form_circle):
            dx = kv*(potential_feild6[0] - pos[0])
            dy = kv*(potential_feild6[1] - pos[1])

        elif(not self.circle_up):
            dx = kv*(potential_feild5[0] - pos[0])
            dy = kv*(potential_feild5[1] - pos[1])

        elif(not self.circle_left):
            dx = kv*(potential_feild7[0] - pos[0])
            dy = kv*(potential_feild7[1] - pos[1])

        elif(not self.red_place):
            dx = kv*(potential_feild8[0] - pos[0])
            dy = kv*(potential_feild8[1] - pos[1])

        elif(not self.v_transform):
            dx = kv*(potential_feild8[0] - pos[0])
            dy = kv*(potential_feild8[1] - pos[1])

        elif(not self.v_final_down):
            dx = kv*(potential_feild7[0] - pos[0])
            dy = kv*(potential_feild7[1] - pos[1])

        elif(not self.v_final_right):
            dx = kv*(potential_feild9[0] - pos[0])
            dy = kv*(potential_feild9[1] - pos[1])

        elif(not self.back_in):
            dx = kv*(potential_feild10[0] - pos[0])
            dy = kv*(potential_feild10[1] - pos[1])

        else:
            dx = kv*(potential_feild11[0] - pos[0])
            dy = kv*(potential_feild11[1] - pos[1])

        # positions = np.zeros(12)
        # print("robot:",self.id)
        # positions[self.id*2] = pos[0]
        # positions[self.id*2+1] = pos[1]
        if messages:
            for m in messages:
                # positions[2*m[0]] = m[1][0]
                # positions[m[0]*2+1] = m[1][1]
                # print(m[0])
                """ formation control square"""
                # print(self.square_done, self.move_out, self.circle_done)
                if(not self.square_done):
                    dx += ks*((m[1][0] - pos[0]) - (self.desired_positions_square[2*self.id] - self.desired_positions_square[2*m[0]]))
                    dy += ks*((m[1][1] - pos[1]) - (self.desired_positions_square[2*self.id+1] - self.desired_positions_square[2*m[0]+1]))

                # """formation control circle"""
                elif(not self.move_out):
                    if(np.linalg.norm(m[1]-pos)<0.5):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))

                # """potential field control"""
                # if(np.linalg.norm((pos-potential_feild))<feild_distance2):

                elif(not self.circle_done):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))
                    """potential field control"""

                elif(not self.v_done):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_side):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_up):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.circle_form):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))

                elif(not self.purple_place):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))

                elif(not self.v_back):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_down):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_right):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_inverse):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v_inv[2*self.id] - self.desired_positions_v_inv[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v_inv[2*self.id+1] - self.desired_positions_v_inv[2*m[0]+1]))

                elif(not self.v_inverse_down):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v_inv[2*self.id] - self.desired_positions_v_inv[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v_inv[2*self.id+1] - self.desired_positions_v_inv[2*m[0]+1]))

                elif(not self.form_circle):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))

                elif(not self.circle_up):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))

                elif(not self.circle_left):
                    if(np.linalg.norm(m[1]-pos)<0.5):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))


                elif(not self.red_place):
                    if(np.linalg.norm(m[1]-pos)<0.5):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kc*((m[1][0] - pos[0]) - (self.desired_positions_circle[2*self.id] - self.desired_positions_circle[2*m[0]]))
                    dy += kc*((m[1][1] - pos[1]) - (self.desired_positions_circle[2*self.id+1] - self.desired_positions_circle[2*m[0]+1]))

                elif(not self.v_transform):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_final_down):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.v_final_right):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    # dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_v[2*self.id] - self.desired_positions_v[2*m[0]]))
                    # dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_v[2*self.id+1] - self.desired_positions_v[2*m[0]+1]))

                elif(not self.back_in):
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))


                else:
                    if(np.linalg.norm(m[1]-pos)<0.4):
                        dx += np.sign(pos[0]-m[1][0])*kr*np.exp(-abs(pos[0]-m[1][0]))
                        dy += np.sign(pos[1]-m[1][1])*kr*np.exp(-abs(pos[1]-m[1][1]))
                    dx += kv*((m[1][0] - pos[0]) - (self.desired_positions_rhombus[2*self.id] - self.desired_positions_rhombus[2*m[0]]))
                    dy += kv*((m[1][1] - pos[1]) - (self.desired_positions_rhombus[2*self.id+1] - self.desired_positions_rhombus[2*m[0]+1]))


            for obs in self.obstacles:
                if(np.linalg.norm(pos[:-1]-obs) < min_distance):
                    dx += np.sign(pos[0]-obs[0])*ko*np.exp(-abs(pos[0]-obs[0]))
                    dy += np.sign(pos[1]-obs[1])*ko*np.exp(-abs(pos[1]-obs[1]))

            if(self.count == 2500):
                if(not self.square_done):
                    self.square_done = True

                elif(not self.move_out):
                    self.move_out = True

                elif(not self.circle_done):
                    self.circle_done = True

                elif(not self.v_done):
                    self.v_done = True

                elif(not self.v_side):
                    self.v_side = True

                elif(not self.v_up):
                    self.v_up = True

                elif(not self.circle_form):
                    self.circle_form = True

                elif(not self.purple_place):
                    self.purple_place = True

                elif(not self.v_back):
                    self.v_back = True

                elif(not self.v_down):
                    self.v_down = True

                elif(not self.v_right):
                    self.v_right = True

                elif(not self.v_inverse):
                    self.v_inverse = True

                elif(not self.v_inverse_down):
                    self.v_inverse_down = True

                elif(not self.form_circle):
                    self.form_circle = True

                elif(not self.circle_up):
                    self.circle_up = True

                elif(not self.circle_left):
                    self.circle_left = True

                elif(not self.red_place):
                    self.red_place = True

                elif(not self.v_transform):
                    self.v_transform = True

                elif(not self.v_final_down):
                    self.v_final_down = True

                elif(not self.v_final_right):
                    self.v_final_right = True

                elif(not self.back_in):
                    self.back_in = True

                else:
                    self.form_rhombus = True

                self.count = 0

            # if(np.linalg.norm(pos == self.prev_position) < 0.006):
            #     self.static = True
            #
            # else:
            #     self.static = False
            # self.is_static(pos)
            # print(self.static, self.square_done, self.move_out, self.circle_done)
            # self.prev_position = pos
            self.count +=1
            # if(self.circle_done):
            #     potential_feild[0]+=.1
            #     potential_feild[1]+=.1
            # if(self.static):
            #     if()
            #
            #     else:
            #         dx+=0
            #         dy+=0


                # else:
                #     dx = gain*(np.log(feild_distance2) + feild_distance1/feild_distance2)
                #     dy = gain*(np.log(feild_distance2) + feild_distance1/feild_distance2)

            # integrate
            # des_pos_x = pos[0] + self.dt * dx
            # des_pos_y = pos[1] + self.dt * dy
            # desired_graph = self.get_graph(desired_pos)

            # des_pos = self.formation_consensus(edges, desired_positions, positions, gain, self.dt)
            # print(des_pos.shape)
            # des_pos_x = np.asarray([des_pos[i] for i in range(0,12,2)])
            # des_pos_y = np.asarray([des_pos[i] for i in range(1,12,2)])
            # dx = des_pos_x[self.id]
            # dy = des_pos_y[self.id]
            # print(dy)
            #compute velocity change for the wheels
            vel_norm = np.linalg.norm([dx, dy]) #norm of desired velocity
            if vel_norm < 0.01:
                vel_norm = 0.01
            des_theta = np.arctan2(dy/vel_norm, dx/vel_norm)
            right_wheel = np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            left_wheel = -np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            self.set_wheel_velocity([left_wheel, right_wheel])


    # def get_laplacian(self, incidence_matrix):
    #     laplacian = incidence_matrix.dot(incidence_matrix.T)
    #     return laplacian
    #
    # def get_incidence_matrix(self, edges, num_vertices):
    #
    #     incidence_matrix = np.zeros((2*num_vertices, 2*len(edges)))
    #     for i in range(len(edges)):
    #
    #         incidence_matrix[2*edges[i][0],2*i] =-1
    #         incidence_matrix[2*edges[i][1],2*i] = 1
    #         incidence_matrix[2*edges[i][0]+1,2*i+1] =-1
    #         incidence_matrix[2*edges[i][1]+1,2*i+1] = 1
    #
    #     return incidence_matrix
    #
    # def get_rigidity_matrix(self, positions):
    #     return np.asarray([[2*positions[0] - 2*positions[2], 2*positions[1] - 2*positions[3], -2*positions[0] + 2*positions[2], -2*positions[1] + 2*positions[3], 0, 0, 0, 0, 0, 0, 0, 0],
    #     [0, 0, 2*positions[2] - 2*positions[6], 2*positions[3] - 2*positions[7], 0, 0, -2*positions[2] + 2*positions[6], -2*positions[3] + 2*positions[7], 0, 0, 0, 0],
    #     [0, 0, 0, 0, 0, 0, 2*positions[6] - 2*positions[10], 2*positions[7] - 2*positions[11], 0, 0, -2*positions[6] + 2*positions[10], -2*positions[7] + 2*positions[11]],
    #     [0, 0, 0, 0, 0, 0, 0, 0, 2*positions[8] - 2*positions[10], 2*positions[9] - 2*positions[11], -2*positions[8] + 2*positions[10], -2*positions[9] + 2*positions[11]],
    #     [0, 0, 0, 0, 2*positions[4] - 2*positions[8], 2*positions[5] - 2*positions[9], 0, 0, -2*positions[4] + 2*positions[8], -2*positions[5] + 2*positions[9], 0, 0],
    #     [2*positions[0] - 2*positions[4], 2*positions[1] - 2*positions[5], 0, 0, -2*positions[0] + 2*positions[4], -2*positions[1] + 2*positions[5], 0, 0, 0, 0, 0, 0],
    #     [2*positions[0] - 2*positions[10], 2*positions[1] - 2*positions[11], 0, 0, 0, 0, 0, 0, 0, 0, -2*positions[0] + 2*positions[10], -2*positions[1] + 2*positions[11]],
    #     [0, 0, 2*positions[2] - 2*positions[8], 2*positions[3] - 2*positions[9], 0, 0, 0, 0, -2*positions[2] + 2*positions[8], -2*positions[3] + 2*positions[9], 0, 0],
    #     [0, 0, 0, 0, 2*positions[4] - 2*positions[6], 2*positions[5] - 2*positions[7], -2*positions[4] + 2*positions[6], -2*positions[5] + 2*positions[7], 0, 0, 0, 0]])
    #
    # def get_graph(self, positions):
    #     return np.asarray([[(positions[0]-positions[2])**2+(positions[1]-positions[3])**2],
    #     [(positions[2]-positions[6])**2+(positions[3]-positions[7])**2],
    #     [(positions[6]-positions[10])**2+(positions[7]-positions[11])**2],
    #     [(positions[8]-positions[10])**2+(positions[9]-positions[11])**2],
    #     [(positions[4]-positions[8])**2+(positions[5]-positions[9])**2],
    #     [(positions[0]-positions[4])**2+(positions[1]-positions[5])**2],
    #     [(positions[0]-positions[10])**2+(positions[1]-positions[11])**2],
    #     [(positions[2]-positions[8])**2+(positions[3]-positions[9])**2],
    #     [(positions[4]-positions[6])**2+(positions[5]-positions[7])**2],])
    #
    #
    # def formation_consensus(self, edges, desired_positions, positions, k, dt):
    #     incidence_matrix = self.get_incidence_matrix(edges, 6)
    #     laplacian = self.get_laplacian(incidence_matrix)
    #     return (-k*laplacian.dot(positions)+k*incidence_matrix.dot(incidence_matrix.T.dot(desired_positions)))*dt
