from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory.fot_wrapper \
    import run_fot
import time
from pylot.planning.planner import Planner


class FOTPlanner(Planner):
    """Frenet Optimal Trajectory (FOT) planner.

    This planner uses a global route and predictions to produce a frenet
    optimal trajectory plan. Details can be found at
    `Frenet Optimal Trajectory Planner`_.

    .. _Frenet Optimal Trajectory Planner:
       https://github.com/erdos-project/frenet_optimal_trajectory_planner
    """
    def __init__(self, world, hyperparameters):
        super().__init__(world)
        self.s0 = 0.0
        self._hyperparameters = {
            "num_threads": 1,
            "max_speed": hyperparameters['max_speed'],
            "max_accel": hyperparameters['max_accel'],
            "max_curvature": hyperparameters['max_curvature'],
            "max_road_width_l": hyperparameters['max_road_width_l'],
            "max_road_width_r": hyperparameters['max_road_width_r'],
            "d_road_w": hyperparameters['d_road_w'],
            "dt": hyperparameters['dt'],
            "maxt": hyperparameters['maxt'],
            "mint": hyperparameters['mint'],
            "d_t_s": hyperparameters['d_t_s'],
            "n_s_sample": hyperparameters['n_s_sample'],
            "obstacle_clearance": hyperparameters['obstacle_clearance_fot'],
            "kd": hyperparameters['kd'],
            "kv": hyperparameters['kv'],
            "ka": hyperparameters['ka'],
            "kj": hyperparameters['kj'],
            "kt": hyperparameters['kt'],
            "ko": hyperparameters['ko'],
            "klat": hyperparameters['klat'],
            "klon": hyperparameters['klon']
        }
        print("planner initialized")

    def fot_parameters_using_99_percentile(self, ttd):
        maxt = self._flags.maxt
        runtimes = [309, 208, 148, 67, 40]
        dts = [0.09, 0.11, 0.13, 0.19, 0.31]
        d_road_ws = [0.3, 0.3, 0.3, 0.5, 0.7]

        for index, runtime in enumerate(runtimes):
            if ttd >= runtime:
                return maxt, dts[index], d_road_ws[index]
        # Not enough time to run the planner.
        print(
            'Not enough time to run the planner. Using the fastest version')
        return maxt, dts[-1], d_road_ws[-1]

    def update_hyper_parameters(self, timestamp, ttd):
        """Changes planning hyper parameters depending on time to decision."""
        # Change hyper paramters if static or dynamic deadlines are enabled.
        if self._flags.deadline_enforcement == 'dynamic':
            maxt, dt, d_road_w = self.fot_parameters_using_99_percentile(ttd)
        elif self._flags.deadline_enforcement == 'static':
            maxt, dt, d_road_w = self.fot_parameters_using_99_percentile(
                self._flags.planning_deadline)
        else:
            return
        print(
            '@{}: planner using maxt {}, dt {}, d_road_w {}'.format(
                timestamp, maxt, dt, d_road_w))
        self._hyperparameters['maxt'] = maxt
        self._hyperparameters['dt'] = dt
        self._hyperparameters['d_road_w'] = d_road_w

    def run(self, timestamp, ttd=None):
        """Runs the planner.

        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """
        self.update_hyper_parameters(timestamp, ttd)
        print("@{}: Hyperparameters: {}".format(
            timestamp, self._hyperparameters))
        initial_conditions = self._compute_initial_conditions()
        print("@{}: Initial conditions: {}".format(
            timestamp, initial_conditions))
        start = time.time()
        (path_x, path_y, speeds, ix, iy, iyaw, d, s, speeds_x, speeds_y, misc,
         costs, success) = run_fot(initial_conditions, self._hyperparameters)
        fot_runtime = (time.time() - start) * 1000
        print('@{}: Frenet runtime {}'.format(
            timestamp, fot_runtime))
        if success:
            print("@{}: Frenet succeeded.".format(timestamp))
            self._log_output(timestamp, path_x, path_y, speeds, ix, iy, iyaw,
                             d, s, speeds_x, speeds_y, costs)
            output_wps = self.build_output_waypoints(path_x, path_y, speeds)
        else:
            print(
                "@{}: Frenet failed. Sending emergency stop.".format(
                    timestamp))
            output_wps = self._world.follow_waypoints(0)

        # update current pose
        self.s0 = misc['s']
        return output_wps

    def _compute_initial_conditions(self):
        ego_transform = self._world.ego_transform
        obstacle_list = self._world.get_obstacle_list()
        current_index = self._world.waypoints.closest_waypoint(
            ego_transform.location)
        # compute waypoints offset by current location
        wps = self._world.waypoints.slice_waypoints(
            max(current_index - self._flags.num_waypoints_behind, 0),
            min(current_index + self._flags.num_waypoints_ahead,
                len(self._world.waypoints.waypoints)))
        initial_conditions = {
            'ps': self.s0,
            'target_speed': self._flags.target_speed,
            'pos': ego_transform.location.as_numpy_array_2D(),
            'vel': self._world.ego_velocity_vector.as_numpy_array_2D(),
            'wp': wps.as_numpy_array_2D().T,
            'obs': obstacle_list,
        }
        return initial_conditions

    def _log_output(self, timestamp, path_x, path_y, speeds, ix, iy, iyaw, d,
                    s, speeds_x, speeds_y, costs):
        print("@{}: Frenet Path X: {}".format(
            timestamp, path_x.tolist()))
        print("@{}: Frenet Path Y: {}".format(
            timestamp, path_y.tolist()))
        print("@{}: Frenet Speeds: {}".format(
            timestamp, speeds.tolist()))
        print("@{}: Frenet IX: {}".format(timestamp, ix.tolist()))
        print("@{}: Frenet IY: {}".format(timestamp, iy.tolist()))
        print("@{}: Frenet IYAW: {}".format(
            timestamp, iyaw.tolist()))
        print("@{}: Frenet D: {}".format(timestamp, d.tolist()))
        print("@{}: Frenet S: {}".format(timestamp, s.tolist()))
        print("@{}: Frenet Speeds X: {}".format(
            timestamp, speeds_x.tolist()))
        print("@{}: Frenet Speeds Y: {}".format(
            timestamp, speeds_y.tolist()))
        print("@{}: Frenet Costs: {}".format(timestamp, costs))
