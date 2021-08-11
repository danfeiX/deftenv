import os

import pybullet as p
import pybullet_data
from deftenv.envs.base_objects import Object

from deftenv.envs.camera import Camera
import deftenv.pybullet_tools.utils as PBU
from deftenv.envs.base_env import BaseEnv


class TableTop(BaseEnv):
    def __init__(self, **kwargs):
        kwargs["robot_base_pose"] = ([0.5, 0.3, 1.2], [0, 0, 1, 0])
        super(TableTop, self).__init__(**kwargs)

    def _create_sensors(self):
        PBU.set_camera(0, -45, 0.8, (0.5, -0.3, 1.0))
        # PBU.set_camera(0, -45, 0.8, (0.0, -0.3, 1.0))
        self.camera = Camera(
            height=self._camera_width,
            width=self._camera_height,
            fov=60,
            near=0.01,
            far=10.,
            renderer=p.ER_TINY_RENDERER
        )
        self.camera.set_pose_ypr((0.0, -0.3, 1.0), distance=0.8, yaw=0, pitch=-45)

    def _create_fixtures(self):
        table_id = p.loadURDF(
            os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
            useFixedBase=True,
            basePosition=(0, 0, 0.0)
        )
        table = Object()
        table.loaded = True
        table.body_id = table_id
        self.fixtures.add_object("table", table)

