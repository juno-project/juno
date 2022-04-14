from pylot.planning.planner import Planner

from rrt_star_planner.RRTStar.rrt_star_wrapper import apply_rrt_star


class RRTStarPlanner(Planner):
    """Wrapper around the RRT* planner.

    Note:
        Details can be found at `RRT* Planner`_.

    Args:
        world: (:py:class:`~pylot.planning.world.World`): A reference to the
            planning world.
        flags (absl.flags): Object to be used to access absl flags.

    .. _RRT* Planner:
       https://github.com/erdos-project/rrt_star_planner
    """
    def __init__(self, world, rrt_star_parameters):
        print("start initializing rrt_start_planner")
        super().__init__(world)
        self._hyperparameters = {
            "step_size": rrt_star_parameters['step_size'],
            "max_iterations": rrt_star_parameters['max_iterations'],
            "end_dist_threshold": rrt_star_parameters['end_dist_threshold'],
            "obstacle_clearance": rrt_star_parameters['obstacle_clearance'],
            "lane_width": rrt_star_parameters['lane_width']
        }
        print("rrt_star_planner initilized")

    def run(self, timestamp, ttd=None):
        """Runs the planner.
   
        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """
        obstacle_list = self._world.get_obstacle_list()
        if len(obstacle_list) == 0:
            # Do not use RRT* if there are no obstacles.
            # Do not use Hybrid A* if there are no obstacles.
            output_wps = self._world.follow_waypoints(self._flags.target_speed)
        else:
            # RRT* does not take into account the driveable region.
            # It constructs search space as a top down, minimum bounding
            # rectangle with padding in each dimension.
            print("@{}: Hyperparameters: {}".format(
                timestamp, self._hyperparameters))
            initial_conditions = self._compute_initial_conditions(
                obstacle_list)
            print("@{}: Initial conditions: {}".format(
                timestamp, initial_conditions))
            path_x, path_y, success = apply_rrt_star(initial_conditions,
                                                     self._hyperparameters)
            if success:
                print("@{}: RRT* succeeded".format(timestamp))
                speeds = [self._flags.target_speed] * len(path_x)
                print("@{}: RRT* Path X: {}".format(
                    timestamp, path_x.tolist()))
                print("@{}: RRT* Path Y: {}".format(
                    timestamp, path_y.tolist()))
                print("@{}: RRT* Speeds: {}".format(
                    timestamp, speeds))
                output_wps = self.build_output_waypoints(
                    path_x, path_y, speeds)
            else:
                print("@{}: RRT* failed. "
                                   "Sending emergency stop.".format(timestamp))
                output_wps = self._world.follow_waypoints(0)
        return output_wps

    def _compute_initial_conditions(self, obstacles):
        ego_transform = self._world.ego_transform
        self._world.waypoints.remove_completed(ego_transform.location)
        end_index = min(self._flags.num_waypoints_ahead,
                        len(self._world.waypoints.waypoints) - 1)
        if end_index < 0:
            # If no more waypoints left. Then our location is our end wp.
            print("@{}: No more waypoints left")
            end_wp = ego_transform
        else:
            end_wp = self._world.waypoints.waypoints[end_index]
        initial_conditions = {
            "start": ego_transform.location.as_numpy_array_2D(),
            "end": end_wp.location.as_numpy_array_2D(),
            "obs": obstacles,
        }
        return initial_conditions
