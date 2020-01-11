import numpy as np

class controller():
    """
    This class controls the robots using different types of control law
    """

    def __init__(self):
        self.dt = 0.001
        self.static = False


    def get_laplacian(incidence_matrix):
        laplacian = incidence_matrix.dot(incidence_matrix.T)
        return laplacian

    def get_incidence_matrix(edges, num_vertices):

        incidence_matrix = np.zeros((2*num_vertices, 2*len(edges)))
        for i in range(len(edges)):

            incidence_matrix[2*edges[i][0],2*i] =-1
            incidence_matrix[2*edges[i][1],2*i] = 1
            incidence_matrix[2*edges[i][0]+1,2*i+1] =-1
            incidence_matrix[2*edges[i][1]+1,2*i+1] = 1

        return incidence_matrix

    def get_rigidity_matrix(positions):
        return np.asarray([[2*positions[0] - 2*positions[2], 2*positions[1] - 2*positions[3], -2*positions[0] + 2*positions[2], -2*positions[1] + 2*positions[3], 0, 0, 0, 0, 0, 0, 0, 0],
                            [0, 0, 2*positions[2] - 2*positions[6], 2*positions[3] - 2*positions[7], 0, 0, -2*positions[2] + 2*positions[6], -2*positions[3] + 2*positions[7], 0, 0, 0, 0],
                            [0, 0, 0, 0, 0, 0, 2*positions[6] - 2*positions[10], 2*positions[7] - 2*positions[11], 0, 0, -2*positions[6] + 2*positions[10], -2*positions[7] + 2*positions[11]],
                            [0, 0, 0, 0, 0, 0, 0, 0, 2*positions[8] - 2*positions[10], 2*positions[9] - 2*positions[11], -2*positions[8] + 2*positions[10], -2*positions[9] + 2*positions[11]],
                            [0, 0, 0, 0, 2*positions[4] - 2*positions[8], 2*positions[5] - 2*positions[9], 0, 0, -2*positions[4] + 2*positions[8], -2*positions[5] + 2*positions[9], 0, 0],
                            [2*positions[0] - 2*positions[4], 2*positions[1] - 2*positions[5], 0, 0, -2*positions[0] + 2*positions[4], -2*positions[1] + 2*positions[5], 0, 0, 0, 0, 0, 0],
                            [2*positions[0] - 2*positions[10], 2*positions[1] - 2*positions[11], 0, 0, 0, 0, 0, 0, 0, 0, -2*positions[0] + 2*positions[10], -2*positions[1] + 2*positions[11]],
                            [0, 0, 2*positions[2] - 2*positions[8], 2*positions[3] - 2*positions[9], 0, 0, 0, 0, -2*positions[2] + 2*positions[8], -2*positions[3] + 2*positions[9], 0, 0],
                            [0, 0, 0, 0, 2*positions[4] - 2*positions[6], 2*positions[5] - 2*positions[7], -2*positions[4] + 2*positions[6], -2*positions[5] + 2*positions[7], 0, 0, 0, 0]])


    def get_graph(positions):
        return np.asarray([[(positions[0]-positions[2])**2+(positions[1]-position[3])**2],
        [(positions[2]-positions[6])**2+(positions[3]-position[7])**2],
        [(positions[6]-positions[10])**2+(positions[7]-position[11])**2],
        [(positions[8]-positions[10])**2+(positions[9]-position[11])**2],
        [(positions[4]-positions[8])**2+(positions[5]-position[9])**2],
        [(positions[0]-positions[4])**2+(positions[1]-position[5])**2],
        [(positions[0]-positions[10])**2+(positions[1]-position[11])**2],
        [(positions[2]-positions[8])**2+(positions[3]-position[9])**2],
        [(positions[4]-positions[6])**2+(positions[5]-position[7])**2],])

    def formation_consensus(positions, desired_graph):
        rigidity_matrix = get_rigidity_matrix(positions)
        current_graph = get_graph(positions)
        return (rigidity_matrix.T.dot(desired_graph - current_graph))*self.dt + positions

    def is_static(positions, self.positions):
        if(np.all((positions-self.positions)**2<0.6)):
            self.static = True

        else:
            self.static = False

        self.positions = positions
        
