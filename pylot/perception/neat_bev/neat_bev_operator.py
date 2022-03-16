
import os
import cv2
import logging
import time

import erdos
import pylot
from pylot.simulation.utils import get_world

import numpy as np

from PIL import Image, ImageDraw
from collections import deque
from neat.planner import RoutePlanner
from neat.config import GlobalConfig
from neat import AttentionField
import torch
from pylot.control.messages import ControlMessage
from neat.route_manipulation import downsample_route,interpolate_trajectory
from neat.route_indexer import RouteParser
from neat.utils import iou, flow_to_color

class NeatBEVOperator(erdos.Operator):
    """Detects obstacles using a  model.

    The operator receives frames on a camera stream, and runs a model for each
    frame.

    Args:
        camera_stream (:py:class:`erdos.ReadStream`): The stream on which
            camera frames are received.
        obstacles_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends
            :py:class:`~pylot.perception.messages.ObstaclesMessage` messages.
        model_path(:obj:`str`): Path to the model pb file.
        flags (absl.flags): Object to be used to access absl flags.
    """
    def __init__(self, camera_stream: erdos.ReadStream,
                 camera_left_stream: erdos.ReadStream,
                 camera_right_stream: erdos.ReadStream,
                 pose_stream: erdos.ReadStream,
                 gnss_stream: erdos.ReadStream,
                 imu_stream: erdos.ReadStream,
                 time_to_decision_stream: erdos.ReadStream,
                 control_stream:erdos.WriteStream,
                 goal_location: pylot.utils.Location,
                 flags):

        camera_stream.add_callback(self.on_camera_msg)
        camera_left_stream.add_callback(self.on_left_camera_msg)
        camera_right_stream.add_callback(self.on_right_camera_msg)
        pose_stream.add_callback(self.on_pose_stream_msg)
        gnss_stream.add_callback(self.on_gnss_stream_msg)
        imu_stream.add_callback(self.on_imu_stream_msg)
        time_to_decision_stream.add_callback(self.on_time_to_decision_update)
        erdos.add_watermark_callback([
            camera_stream, camera_left_stream, camera_right_stream,
            time_to_decision_stream], [control_stream], self.on_watermark)

        self._goal = goal_location
        self._left_imgs = {}
        self._right_imgs = {}
        self._imgs = {}
        self._gps = {}
        self._imu = {}
        self._flags = flags
        self._pose = {}
        self._count = 0

        self._logger = erdos.utils.setup_logging(self.config.name,self.config.log_file_name)
        # self._obstacles_stream = obstacles_stream
        # self._waypoints_stream = waypoints_stream
        self._data_path = os.path.join(self._flags.data_path, 'input_images')
        os.makedirs(self._data_path, exist_ok=True)

        self._client, self._world = get_world(self._flags.simulator_host, self._flags.simulator_port,
                                  self._flags.simulator_timeout)
        self.world = self._client.load_world('Town{:02d}'.format(
                self._flags.simulator_town))

        self._initalize_net()

    @staticmethod
    def connect(camera_stream: erdos.ReadStream,
                 camera_left_stream: erdos.ReadStream,
                 camera_right_stream: erdos.ReadStream,
                 pose_stream: erdos.ReadStream,
                 gnss_stream: erdos.ReadStream,
                 imu_stream: erdos.ReadStream,
                 time_to_decision_stream: erdos.ReadStream):

        control_stream = erdos.WriteStream()
        return [control_stream]

    def destroy(self):
        self._logger.warn('destroying {}'.format(self.config.name))

    def on_left_camera_msg(self, msg):
        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))
        msg.frame.save(msg.timestamp.coordinates[0], self._data_path,
                       'left')
        # The models expect BGR images.
        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        frame = cv2.cvtColor(msg.frame.frame, cv2.COLOR_BGR2RGB)
        img = self.pre_process(Image.fromarray(frame))
        self._left_imgs[msg.timestamp] = torch.from_numpy(img).unsqueeze(0).float()

    def on_right_camera_msg(self, msg):
        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))
        msg.frame.save(msg.timestamp.coordinates[0], self._data_path,
                       'right')
        # The models expect BGR images.
        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        frame = cv2.cvtColor(msg.frame.frame, cv2.COLOR_BGR2RGB)
        img = self.pre_process(Image.fromarray(frame))
        self._right_imgs[msg.timestamp] = torch.from_numpy(img).unsqueeze(0).float()

    def on_camera_msg(self, msg):
        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))
        msg.frame.save(msg.timestamp.coordinates[0], self._data_path,
                       'center')
        # The models expect BGR images.
        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        frame = cv2.cvtColor(msg.frame.frame, cv2.COLOR_BGR2RGB)
        img = self.pre_process(Image.fromarray(frame))
        self._imgs[msg.timestamp] = torch.from_numpy(img).unsqueeze(0).float()


    def on_pose_stream_msg(self, msg):
        """
        参考  neat\leaderboard\leaderboard\envs\sensor_interface.py 里的 SpeedometerReader
        """
        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))

        self._pose[msg.timestamp] = np.array([msg.data.forward_speed])
        # print('self._count', self._count)
        if self._count < 1:
            ego_transform = msg.data.transform
            print('goal', self._goal.as_simulator_location())
            print('pose', ego_transform.location.as_simulator_location())
            trajectory = [ego_transform.location.as_simulator_location(), self._goal.as_simulator_location()]
            self._initalize_global_plan(trajectory)
        self._count += 1


    def on_gnss_stream_msg(self, msg):
        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))
        gps =  np.array([msg.latitude,msg.longitude], dtype=np.float64)
        gps = (gps - np.array([0.0, 0.0])) * np.array([111324.60662786, 111319.490945])

        self._gps[msg.timestamp] = gps

    def on_imu_stream_msg(self, msg):

        self._logger.debug('@{}: {} received left camera message'.format(
            msg.timestamp, self.config.name))

        self._imu[msg.timestamp] = msg.compass

    def on_time_to_decision_update(self, msg: erdos.Message):
        self._logger.debug('@{}: {} received ttd update {}'.format(
            msg.timestamp, self.config.name, msg))

    def get_target(self,pos,compass):

        next_wp, next_cmd = self._route_planner.run_step(pos)
        # print('next_wp', next_wp)
        # print('pos', pos)
        theta = compass + np.pi / 2
        R = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        local_command_point = np.array([next_wp[0] - pos[0], next_wp[1] - pos[1]])
        local_command_point = R.T.dot(local_command_point)
        # print('local_command_point', local_command_point)
        return tuple(torch.from_numpy(local_command_point).float())


    def pre_process(self,image, scale=1, crop=256):
        """
        Scale and crop a PIL image, returning a channels-first numpy array.
        """
        # image = Image.open(filename)
        (width, height) = (image.width // scale, image.height // scale)
        im_resized = image.resize((width, height))
        image = np.asarray(im_resized)
        start_x = height // 2 - crop // 2
        start_y = width // 2 - crop // 2
        cropped_image = image[start_x:start_x + crop, start_y:start_y + crop]
        cropped_image = np.transpose(cropped_image, (2, 0, 1))
        return cropped_image

    def _initalize_net(self):
        print('initalize_net')

        self.input_buffer = {'rgb': deque(), 'rgb_left': deque(), 'rgb_right': deque()}
        self.step = -1
        self.neat_config = GlobalConfig()
        self.net = AttentionField(self.neat_config)
        encoder_path = '/home/erdos/workspace/pylot/model_ckpt/neat/best_encoder.pth'
        decoder_path = '/home/erdos/workspace/pylot/model_ckpt/neat/best_decoder.pth'
        # print(torch.load(encoder_path))

        self.net.encoder.load_state_dict(torch.load(encoder_path, torch.device('cpu')))
        self.net.decoder.load_state_dict(torch.load(decoder_path, torch.device('cpu')))

        self.plan_grid = self.net.create_plan_grid(self.neat_config.plan_scale, self.neat_config.plan_points, 1)
        self.light_grid = self.net.create_light_grid(self.neat_config.light_x_steps, self.neat_config.light_y_steps, 1)

        self.net.to(self.net.device)
        self.net.eval()

        # print(self.net.state_dict()['encoder.img_emb.features.conv1.weight'])
        # for name, param in self.net.named_parameters():
        #     print(name, param.shape)self.net.eval()

        print('initalize_net success')

        return

    def _initalize_global_plan(self, trajectory):
        gps_route, route = interpolate_trajectory(self.world, trajectory)
        ds_ids = downsample_route(route, 50)
        self._global_plan = [gps_route[x] for x in ds_ids]

        self._route_planner = RoutePlanner(4.0, 50.0)
        self._route_planner.set_route(self._global_plan, True)


    def on_watermark(self, timestamp: erdos.Timestamp,
                     # obstacles_stream: erdos.WriteStream,
                     # waypoints_stream: erdos.WriteStream,
                     control_stream:erdos.WriteStream):
        self._logger.debug('@{}: received watermark'.format(timestamp))
        if timestamp.is_top:
            return

        # self.net.eval()
        gt_velocity = self._pose.pop(timestamp)
        # print('gt_velocity', gt_velocity)
        pos = self._gps.pop(timestamp)
        compass = self._imu.pop(timestamp)
        target_point = self.get_target(pos,compass)
        rgb = self._imgs.pop(timestamp)
        # print('self._imgs.pop(timestamp)', rgb)
        rgb_left = self._left_imgs.pop(timestamp)
        rgb_right = self._right_imgs.pop(timestamp)
        if  len(self.input_buffer['rgb']) == 0:
                while len(self.input_buffer['rgb']) < self.neat_config.seq_len:
                    self.input_buffer['rgb'].append(rgb)
                    self.input_buffer['rgb_left'].append(rgb_left)
                    self.input_buffer['rgb_right'].append(rgb_right)
        else:
            self.input_buffer['rgb'].append(rgb)
            self.input_buffer['rgb_left'].append(rgb_left)
            self.input_buffer['rgb_right'].append(rgb_right)
            self.input_buffer['rgb'].popleft()
            self.input_buffer['rgb_left'].popleft()
            self.input_buffer['rgb_right'].popleft()

        images = []
        for i in range(self.neat_config.seq_len):
            images.append(self.input_buffer['rgb'][i])
            if self.neat_config.num_camera == 3:
                images.append(self.input_buffer['rgb_left'][i])
                images.append(self.input_buffer['rgb_right'][i])

        # print(self.net.encoder.img_emb.features.conv1.state_dict())
        # torch.save({'images':images}, '/home/erdos/workspace/pylot/neat/sample/images.pth')
        encoding = self.net.encoder(images, torch.from_numpy(gt_velocity).float())
        # print('encoding', encoding)
        # print('gt_velocity', gt_velocity)
        target_point = torch.stack(target_point)
        print('target_point', target_point.unsqueeze(0))

        pred_waypoint_mean, red_light_occ = self.net.plan(target_point.unsqueeze(0), encoding, self.plan_grid, self.light_grid,
                                                          self.neat_config.plan_points, self.neat_config.plan_iters)

        self.visualize(target_point.unsqueeze(0), pred_waypoint_mean, encoding, timestamp.coordinates[0])

        steer, throttle, brake, metadata = self.net.control_pid(pred_waypoint_mean[:, self.neat_config.seq_len:],
                                                                gt_velocity, target_point, red_light_occ)
        steer = float(steer)
        throttle = float(throttle)
        brake = float(brake)
        # print(metadata)

        if brake < 0.05: brake = 0.0
        if throttle > brake: brake = 0.0

        control_stream.send(
            ControlMessage(steer, throttle, brake, False, False, timestamp))

    def visualize(self, target_point, pred_waypoint_mean, encoding, timestamp):

        out_res = 128
        linspace_x = torch.linspace(-self.neat_config.axis / 2, self.neat_config.axis / 2, steps=out_res)
        linspace_y = torch.linspace(-self.neat_config.axis / 2, self.neat_config.axis / 2, steps=out_res)
        linspace_t = torch.linspace(0, self.neat_config.tot_len - 1, steps=self.neat_config.tot_len)

        # target point in pixel coordinates
        target_point_pixel = target_point.squeeze().cpu().numpy()
        target_point_pixel[1] += self.neat_config.offset * self.neat_config.resolution

        # hack for when actual target is outside image (axis/2 * resolution)
        target_point_pixel = np.clip(target_point_pixel, -(self.neat_config.axis / 2 * self.neat_config.resolution - 1),
                                     (self.neat_config.axis / 2 * self.neat_config.resolution - 1))
        target_point_pixel = (target_point_pixel * out_res // 50 + out_res // 2).astype(np.uint8)

        batch_size = 1
        print('visualize')
        for i in range(self.neat_config.seq_len):
            # predicted waypoint in pixel coordinates
            pred_waypoint = pred_waypoint_mean[0, i].data.cpu().numpy()
            pred_waypoint[1] += self.neat_config.offset * self.neat_config.resolution
            pred_waypoint = np.clip(pred_waypoint, -(self.neat_config.axis / 2 * self.neat_config.resolution - 1),
                                    (self.neat_config.axis / 2 * self.neat_config.resolution - 1))
            pred_waypoint = (pred_waypoint * out_res // (self.neat_config.axis * self.neat_config.resolution) + out_res // 2).astype(np.uint8)
            img_rows = []
            flow_rows = []
            out_path = os.path.join(self._flags.data_path, str(timestamp), str(i))
            os.makedirs(out_path, exist_ok=True)
            for row in range(out_res):
                grid_x, grid_y, grid_t = torch.meshgrid(linspace_x, linspace_y[row], linspace_t[i].unsqueeze(0))
                grid_points = torch.stack((grid_x, grid_y, grid_t), dim=3).unsqueeze(0).repeat(batch_size, 1, 1, 1, 1)
                grid_points = grid_points.reshape(batch_size, -1, 3).to(torch.device('cpu'), dtype=torch.float32)
                pred_img_pts, pred_img_offsets, _ = self.net.decode(grid_points, target_point, encoding)
                pred_img_pts = torch.argmax(pred_img_pts[-1], dim=1)
                pred_img = pred_img_pts.reshape(batch_size, out_res)
                pred_flow = pred_img_offsets[-1].reshape(batch_size, 2, out_res)
                img_rows.append(pred_img)
                flow_rows.append(pred_flow)

            pred_img = torch.stack(img_rows, dim=-1)
            pred_flow = torch.stack(flow_rows, dim=-1)

            semantics = pred_img[0, :, :].transpose(1, 0).data.cpu().numpy().astype(np.uint8)
            semantic_display = np.zeros((semantics.shape[0], semantics.shape[1], 3))
            for key, value in self.neat_config.classes.items():
                semantic_display[np.where(semantics == key)] = value
            semantic_display = semantic_display.astype(np.uint8)
            semantic_display = Image.fromarray(semantic_display)
            print('save', os.path.join(out_path, 'out_'+str(timestamp)+'.jpg'))

            semantic_display.save(os.path.join(out_path, 'out_'+str(timestamp)+'.jpg'))

            # # flow image of predicted offsets
            flow_uv = pred_flow[0, :, :, :].transpose(2, 0).data.cpu().numpy() * out_res / self.neat_config.axis
            flow_rgb = flow_to_color(flow_uv)

            flow_display = Image.fromarray(flow_rgb)
            draw = ImageDraw.Draw(flow_display)
            draw.ellipse([tuple(target_point_pixel - 2), tuple(target_point_pixel + 2)], fill='Blue', outline='Blue')
            draw.ellipse([tuple(pred_waypoint - 2), tuple(pred_waypoint + 2)], fill='Red', outline='Red')
            print('save', os.path.join(out_path, 'flow_'+str(timestamp)+'.jpg'))
            flow_display.save(os.path.join(out_path, 'flow_' + str(timestamp) + '.jpg'))

