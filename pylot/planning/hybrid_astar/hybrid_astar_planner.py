from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper \
    import apply_hybrid_astar

import numpy as np

from pylot.planning.planner import Planner


class HybridAStarPlanner(Planner):
    """Wrapper around the Hybrid A* planner.

    Note:
        Details can be found at `Hybrid A* Planner`_.

    Args:
        world: (:py:class:`~pylot.planning.world.World`): A reference to the
            planning world.
        flags (absl.flags): Object to be used to access absl flags.

    .. _Hybrid A* Planner:
       https://github.com/erdos-project/hybrid_astar_planner
    """
    def __init__(self, world, hybrid_parameters):
        super().__init__(world)
        self._hyperparameters = {
            "step_size": hybrid_parameters['max_iterations_hybrid_astar'],
            "max_iterations": hybrid_parameters['max_iterations_hybrid_astar'],
            "completion_threshold": hybrid_parameters['completion_threshold'],
            "angle_completion_threshold": hybrid_parameters['angle_completion_threshold'],
            "rad_step": hybrid_parameters['rad_step'],
            "rad_upper_range": hybrid_parameters['rad_upper_range'],
            "rad_lower_range": hybrid_parameters['rad_lower_range'],
            "obstacle_clearance": hybrid_parameters['obstacle_clearance_hybrid_astar'],
            "lane_width": hybrid_parameters['lane_width_hybrid_astar'],
            "radius": hybrid_parameters['radius'],
            "car_length": hybrid_parameters['car_length'],
            "car_width": hybrid_parameters['car_width']
        }

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
            # Do not use Hybrid A* if there are no obstacles.
            output_wps = self._world.follow_waypoints(self._flags.target_speed)
        else:
            # Hybrid a* does not take into account the driveable region.
            # It constructs search space as a top down, minimum bounding
            # rectangle with padding in each dimension.
            print("@{}: Hyperparameters: {}".format(
                timestamp, self._hyperparameters))
            initial_conditions = self._compute_initial_conditions(
                obstacle_list)
            print("@{}: Initial conditions: {}".format(
                timestamp, initial_conditions))
            path_x, path_y, _, success = apply_hybrid_astar(
                initial_conditions, self._hyperparameters)
            if success:
                print(
                    "@{}: Hybrid A* succeeded".format(timestamp))
                speeds = [self._flags.target_speed] * len(path_x)
                print("@{}: Hybrid A* Path X: {}".format(
                    timestamp, path_x.tolist()))
                print("@{}: Hybrid A* Path Y: {}".format(
                    timestamp, path_y.tolist()))
                print("@{}: Hybrid A* Speeds: {}".format(
                    timestamp, speeds))
                output_wps = self.build_output_waypoints(
                    path_x, path_y, speeds)
            else:
                print("@{}: Hybrid A* failed. "
                                   "Sending emergency stop.".format(timestamp))
                output_wps = self._world.follow_waypoints(0)
        return output_wps

    def _compute_initial_conditions(self, obstacles):
        ego_transform = self._world.ego_transform
        start = np.array([
            ego_transform.location.x,
            ego_transform.location.y,
            np.deg2rad(ego_transform.rotation.yaw),
        ])
        self._world.waypoints.remove_completed(ego_transform.location)
        end_index = min(self._flags.num_waypoints_ahead,
                        len(self._world.waypoints.waypoints) - 1)
        if end_index < 0:
            # If no more waypoints left. Then our location is our end wp.
            print("@{}: No more waypoints left")
            end_wp = ego_transform
        else:
            end_wp = self._world.waypoints.waypoints[end_index]
        end = np.array([
            end_wp.location.x, end_wp.location.y,
            np.deg2rad(ego_transform.rotation.yaw)
        ])
        initial_conditions = {
            "start": start,
            "end": end,
            "obs": obstacles,
        }
        return initial_conditions
