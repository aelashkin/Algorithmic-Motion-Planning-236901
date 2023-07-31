import operator
import numpy as np

class RRTTree(object):
    
    def __init__(self, planning_env, task="mp"):
        
        self.planning_env = planning_env
        self.task = task
        self.vertices = {}
        self.edges = {}

        # inspecion planning properties
        if self.task == "ip":
            self.max_coverage = 0
            self.max_coverage_id = 0

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def add_vertex(self, config, inspected_points=None, inspected_now=None):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(config=config, inspected_points=inspected_points, inspected_now=inspected_now)

        # check if vertex has the highest coverage so far, and replace if so
        if self.task == "ip":
            v_coverage = self.planning_env.compute_coverage(inspected_points=inspected_points)
            if v_coverage > self.max_coverage:
                self.max_coverage = v_coverage
                self.max_coverage_id = vid

        return vid

    def add_edge(self, sid, eid, edge_cost=0):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def is_goal_exists(self, config):
        '''
        Check if goal exists.
        @param config Configuration to check if exists.
        '''
        goal_idx = self.get_idx_for_config(config=config)
        if goal_idx is not None:
            return True
        return False

    def get_vertex_for_config(self, config):
        '''
        Search for the vertex with the given config and return it if exists
        @param config Configuration to check if exists.
        '''
        v_idx = self.get_idx_for_config(config=config)
        if v_idx is not None:
            return self.vertices[v_idx]
        return None

    def get_idx_for_config(self, config):
        '''
        Search for the vertex with the given config and return the index if exists
        @param config Configuration to check if exists.
        '''
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.config == config).all()]
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def get_nearest_config(self, config):
        '''
        Find the nearest vertex for the given config and returns its state index and configuration
        @param config Sampled configuration.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.planning_env.robot.compute_distance(config, vertex.config))

        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].config

    def get_k_nearest_neighbors(self, config, k, k_log=False):
        '''
        Return k-nearest neighbors in descending order by inspected union with config
        @param state Sampled state.
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        # coverages = []
        # config_inspected = self.planning_env.get_inspected_points(config)

        for _, vertex in self.vertices.items():
            dists.append(self.planning_env.robot.compute_distance(config, vertex.config))
            # new_inspected = self.planning_env.compute_union_of_points(config_inspected, vertex.inspected_now)
            # coverages.append(len(new_inspected))

        #print("dists is:", dists, type(dists))
        dists = np.array(dists)

        n = len(dists)
        # Limit the value of k to the number of elements in the array
        if k_log == True:
            k = int (len(self.vertices) / 7) + 1
            # k = int(np.floor(np.log(n)))
            #print ("k is:", k)
        # print("k is:", k)
        k = min(k, n)
        knn_ids = np.argpartition(dists, k - 1)[:k]
        #knn_dists = [dists[i] for i in knn_ids]

        coverages = []
        config_inspected = self.planning_env.get_inspected_points(config)

        for id in knn_ids:
            new_inspected = self.planning_env.compute_union_of_points(config_inspected, self.vertices[id].inspected_points)
            coverages.append(len(new_inspected))
        coverages = np.array(coverages)
        new_ids = np.argsort(coverages)[::-1]

        # print("knn_ids is:", knn_ids, type(knn_ids))
        knn_ids = knn_ids[new_ids]
        # print("knn_ids is:", knn_ids, type(knn_ids))
        # vid, _ = self.get_nearest_config(config)
        # print("nearest vid is:", vid, type(vid))

        # print("knn_ids is:", knn_ids, type(knn_ids))
        # print("inspected_ids are:", inspected_ids, type(inspected_ids))

        return knn_ids


    def get_k_highest_coverage(self, config, k, k_log=False):
        '''
        Return k-nearest neighbors in descending order by inspected union with config
        @param state Sampled state.
        @param k Number of nearest neighbors to retrieve.
        '''
        # dists = []
        coverages = []
        config_inspected = self.planning_env.get_inspected_points(config)

        for _, vertex in self.vertices.items():
            new_inspected = self.planning_env.compute_union_of_points(config_inspected, vertex.inspected_points)
            coverages.append(len(new_inspected))

        #print("dists is:", dists, type(dists))
        coverages = np.array(coverages)

        n = len(coverages)
        # Limit the value of k to the number of elements in the array
        if k_log == True:
            k = int (len(self.vertices) / 7) + 1
            # k = int(np.floor(np.log(n)))
            #print ("k is:", k)
        # print("k is:", k)
        k = min(k, n)
        knn_ids = np.argpartition(coverages, k - 1)[:k]
        #knn_dists = [dists[i] for i in knn_ids]

        dists = []

        for id in knn_ids:
            dists.append(self.planning_env.robot.compute_distance(config, self.vertices[id].config))
        dists = np.array(dists)
        new_ids = np.argsort(dists)[::-1]

        # print("knn_ids is:", knn_ids, type(knn_ids))
        knn_ids = knn_ids[new_ids]
        # print("knn_ids is:", knn_ids, type(knn_ids))
        # vid, _ = self.get_nearest_config(config)
        # print("nearest vid is:", vid, type(vid))

        # print("knn_ids is:", knn_ids, type(knn_ids))
        # print("inspected_ids are:", inspected_ids, type(inspected_ids))

        return knn_ids


class RRTVertex(object):

    def __init__(self, config, cost=0, inspected_points=None, inspected_now=None):

        self.config = config
        self.cost = cost
        self.inspected_points = inspected_points
        self.inspected_now = inspected_now

    def set_cost(self, cost):
        '''
        Set the cost of the vertex.
        '''
        self.cost = cost