import numpy as np
import os
from collections import OrderedDict

import pybullet as p
import deftenv


from deftenv.envs.base_objects import InteractiveObj, YCBObject
from deftenv.envs.camera import Camera
import deftenv.utils.env_utils as EU
import deftenv.pybullet_tools.utils as PBU
import deftenv.utils.plan_utils as PU
import deftenv.envs.skills as skills
from deftenv.envs.objects import Faucet, Box, CoffeeMachine, CoffeeGrinder, TeaDispenser
from deftenv.envs.base_env import BaseEnv
from deftenv.envs.envs import TableTop


class Kitchen(BaseEnv):
    def __init__(self, **kwargs):
        kwargs["robot_base_pose"] = ([0.8, 0.3, 1.2], [0, 0, 1, 0])
        super(Kitchen, self).__init__(**kwargs)

    def _create_sensors(self):
        PBU.set_camera(45, -60, 1.5, (0, 0, 0.7))
        self.camera = Camera(
            height=self._camera_width,
            width=self._camera_height,
            fov=60,
            near=0.01,
            far=10.,
            renderer=p.ER_TINY_RENDERER
        )
        self.camera.set_pose_ypr((0, 0, 0.7), distance=1.5, yaw=45, pitch=-60)

    def _get_feature_observation(self):
        num_beads = np.zeros((len(self.objects), 2))
        for i, o in enumerate(self.objects.object_list):
            num_beads[i, 0] = len(
                EU.objects_center_in_container(self.objects["faucet_coffee"].beads, container_id=o.body_id)
            )
            num_beads[i, 1] = len(
                EU.objects_center_in_container(self.objects["coffee_machine"].beads, container_id=o.body_id)
            )
        obs = dict(
            num_beads=num_beads
        )
        return obs

    def _create_fixtures(self):
        pass
        # p.loadMJCF(os.path.join(pybullet_data.getDataPath(), "mjcf/ground_plane.xml"))

    def _create_objects(self):
        drawer = InteractiveObj(filename=os.path.join(deftenv.assets_path, 'cabinet2/cabinet_0007.urdf'))
        drawer.load()
        drawer.set_position([0, 0, 0.5])
        p.setJointMotorControl2(drawer.body_id, 2, p.POSITION_CONTROL, force=0)
        EU.set_friction(drawer.body_id, friction=10.)
        self.objects.add_object("drawer", drawer)

        o = Box(color=(0.9, 0.9, 0.9, 1))
        o.load()
        self.objects.add_object("platform1", o, category="platform")

        o = YCBObject('025_mug')
        o.load()
        p.changeDynamics(o.body_id, -1, mass=1.5)
        EU.set_friction(o.body_id)
        self.objects.add_object("mug1", o, category="mug")

        o = CoffeeGrinder(mass=1000.0)
        o.load()
        self.objects.add_object("grinder", o)

        o = Faucet(num_beads=10, dispense_freq=1,
                   beads_color=(70 / 255, 59 / 255, 42 / 255, 1),
                   base_color=(0.5, 0.5, 0.5, 0.0),
                   base_size=(0.13, 0.13, 0.01), dispense_height=0.13, exclude_ids=(self.objects["grinder"].body_id,))
        o.load()
        self.objects.add_object("faucet_coffee", o)
        self.stateful_objects.add_object("faucet_coffee", o)

        o = CoffeeMachine(
            filename=os.path.join(deftenv.assets_path, "coffee_machine_slim/102901.urdf"),
            beans_set=self.objects["faucet_coffee"].beads,
            num_beans_trigger=5,
            beads_color=(162/255, 80/255, 55/255, 1),
            beads_size=0.015,
            dispense_position=np.array([0.05, 0, 0.02]),
            platform_position=np.array([0.08, 0, -0.11]),
            button_pose=((0.0, -0.13, 0.16), skills.ALL_ORIENTATIONS["backward"]),
            scale=0.25
        )
        o.load()
        p.changeDynamics(o.body_id, -1, mass=1000.0)
        self.objects.add_object("coffee_machine", o)
        self.stateful_objects.add_object("coffee_machine", o)
        self.objects.add_object("coffee_machine_platform", o.platform)
        self.objects.add_object("coffee_machine_button", o.button)

    def _reset_objects(self):
        z = PBU.stable_z(self.objects["coffee_machine"].body_id, self.objects["drawer"].body_id)
        self.objects["coffee_machine"].set_position_orientation(
            PU.sample_positions_in_box([-0.2, -0.2], [-0.25, -0.25], [z, z]), skills.ALL_ORIENTATIONS["left"])

        z = PBU.stable_z(self.objects["platform1"].body_id, self.objects["drawer"].body_id)
        self.objects["platform1"].set_position_orientation(
            PU.sample_positions_in_box([0.2, 0.2], [-0.1, -0.1], [z, z]), PBU.unit_quat())

        z = PBU.stable_z(self.objects["mug1"].body_id, self.objects["drawer"].body_id, surface_link=2)
        z -= 0.15
        self.objects["mug1"].set_position_orientation(
            PU.sample_positions_in_box([0.2, 0.2], [-0.05, 0.05], [z, z]), PBU.unit_quat())

        z = PBU.stable_z(self.objects["grinder"].body_id, self.objects["drawer"].body_id)
        pos = PU.sample_positions_in_box([-0.25, -0.15], [0.2, 0.2], [z, z])
        self.objects["grinder"].set_position_orientation(pos, skills.ALL_ORIENTATIONS["back"])

        z = PBU.stable_z(self.objects["faucet_coffee"].body_id, self.objects["grinder"].body_id, surface_link=-1)
        pos = PBU.get_pose(self.objects["grinder"].body_id)[0]
        self.objects["faucet_coffee"].set_position_orientation((pos[0], pos[1], z), PBU.unit_quat())

        EU.set_collision_between(self.robot.gripper.body_id, self.objects["coffee_machine"].body_id, 0)


class KitchenDualAP(Kitchen):
    def _get_pour_pos(self, ns):
        pour_delta = np.zeros((ns, 3))
        pour_delta[:, 2] = 0.5
        pour_delta[:, :2] += (np.random.rand(ns, 2) * 0.07 + 0.03) * np.random.choice([-1, 1], size=(ns, 2))
        return pour_delta

    def _create_skill_lib(self):
        lib_skills = (
            skills.GraspDistDiscreteOrn(
                name="grasp", lift_height=0.02, lift_speed=0.01, reach_distance=0.01, pos_offset=(0, 0, 0.01),
                params=OrderedDict(
                    grasp_distance=skills.SkillParamsContinuous(low=[0.03], high=[0.05]),
                    grasp_orn=skills.SkillParamsDiscrete(size=len(skills.SKILL_ORIENTATIONS))
                ),
                # joint_resolutions=(0.05, 0.05, 0.05, np.pi / 32, np.pi / 32, np.pi / 32)
            ),
            skills.PlacePosYawOrn(
                name="place", retract_distance=0.1, num_pause_steps=30,
                params=OrderedDict(
                    place_pos=skills.SkillParamsContinuous(low=[-0.05, -0.05, 0.01], high=[0.05, 0.05, 0.01]),
                    place_orn=skills.SkillParamsContinuous(low=[0], high=[0])
                ),
                joint_resolutions=(0.05, 0.05, 0.05, np.pi / 32, np.pi / 32, np.pi / 32)
            ),
            skills.PourPosAngle(
                name="pour", pour_angle_speed=np.pi / 32, num_pause_steps=30,
                params=OrderedDict(
                    pour_pos=skills.SkillParamsContinuous(low=(-0.1, -0.1, 0.5), high=(0.1, 0.1, 0.5)),
                    pour_angle=skills.SkillParamsContinuous(low=(np.pi * 0.25,), high=(np.pi,))
                ),
                joint_resolutions=(0.05, 0.05, 0.05, np.pi / 32, np.pi / 32, np.pi / 32)
            ),
            skills.OperatePrismaticPosDistance(
                name="open_prismatic",
                params=OrderedDict(
                    grasp_pos=skills.SkillParamsContinuous(low=[0.35, -0.05, 0.15], high=[0.45, 0.05, 0.25]),
                    prismatic_move_distance=skills.SkillParamsContinuous(low=[-0.4], high=[-0.])
                ),
                # joint_resolutions=(0.05, 0.05, 0.05, np.pi / 32, np.pi / 32, np.pi / 32)
            ),
            skills.ConditionSkill(
                name="filled_coffee", precondition_fn=lambda oid: len(EU.objects_center_in_container(
                    self.objects["coffee_machine"].beads, oid)) >= 3
            ),
            skills.ConditionSkill(
                name="filled_tea", precondition_fn=lambda oid: len(EU.objects_center_in_container(
                    self.objects["faucet_tea"].beads, oid)) >= 3
            )
        )
        self.skill_lib = skills.SkillLibrary(self, self.planner, obstacles=self.obstacles, skills=lib_skills)

    def _get_feature_observation(self):
        num_beads = np.zeros((len(self.objects), 3))
        for i, o in enumerate(self.objects.object_list):
            num_beads[i, 0] = len(
                EU.objects_center_in_container(self.objects["faucet_coffee"].beads, container_id=o.body_id)
            )
            num_beads[i, 1] = len(
                EU.objects_center_in_container(self.objects["faucet_tea"].beads, container_id=o.body_id)
            )
            coffee_beans = EU.objects_center_in_container(self.objects["faucet_coffee"].beads, container_id=o.body_id)
            if o.body_id == self.objects["coffee_machine"].body_id:
                coffee_beans = [c for c in coffee_beans if PBU.get_pose(c)[0][2] > self.objects["coffee_machine"].get_position()[2] - 0.05]
            num_beads[i, 1] = len(coffee_beans)

        obs = dict(
            num_beads=num_beads
        )
        return obs

    def is_success_all_tasks(self):
        mug1_graspable = PBU.is_center_placed_on(self.objects["mug1"].body_id, self.objects["platform1"].body_id)
        num_coffee_in_mug1 = len(EU.objects_center_in_container(
            self.objects["coffee_machine"].beads, self.objects["mug1"].body_id))
        num_beans_in_mug1 = len(EU.objects_center_in_container(
            self.objects["faucet_coffee"].beads, self.objects["mug1"].body_id))
        num_tea_in_mug1 = len(EU.objects_center_in_container(
            self.objects["faucet_tea"].beads, self.objects["mug1"].body_id))

        coffee_beads = EU.objects_center_in_container(self.objects["faucet_coffee"].beads, container_id=self.objects["coffee_machine"].body_id)
        coffee_beads_in_machine = [c for c in coffee_beads if PBU.get_pose(c)[0][2] > self.objects["coffee_machine"].get_position()[2] - 0.05]
        num_beans_in_coffee_machine = len(coffee_beads_in_machine)

        all_successes = {
            "fill_mug1_coffee": num_coffee_in_mug1 >= 3,
            "fill_mug1_beans": num_beans_in_mug1 >= 3,
            "fill_mug1_tea": num_tea_in_mug1 >= 3,
            "fill_coffee_machine": num_beans_in_coffee_machine > 3,
            "mug1_graspable": mug1_graspable
        }
        if self.target_skill == "filled_coffee":
            successes = dict([(k, all_successes[k]) for k in all_successes if k in ["fill_mug1_coffee", "fill_mug1_beans", "fill_coffee_machine", "mug1_graspable"]])
            successes["task"] = successes["fill_mug1_coffee"]
        elif self.target_skill == "filled_tea":
            successes = dict([(k, all_successes[k]) for k in all_successes if k in ["fill_mug1_tea", "mug1_graspable"]])
            successes["task"] = successes["fill_mug1_tea"]
        else:
            raise NotImplementedError

        return successes

    def set_goal(self, task_specs):
        self._task_spec = task_specs
        self.target_skill = self.skill_lib.skill_names[task_specs[0]]

    def _sample_task(self):
        self.target_skill = np.random.choice(["filled_coffee", "filled_tea"])
        # self.target_skill = "filled_coffee"
        self._task_spec = np.array([self.skill_lib.name_to_skill_index(self.target_skill), self.objects.names.index("mug1")])

    def _create_objects(self):
        super(KitchenDualAP, self)._create_objects()
        o = TeaDispenser(mass=1000.0)
        o.load()
        self.objects.add_object("tea", o)

        o = Faucet(num_beads=10, dispense_freq=1,
                   beads_color=(208 / 255, 240 / 255, 192 / 255, 1),
                   base_color=(0, 0, 0, 0),
                   base_size=(0.1, 0.1, 0.005), dispense_height=0.13, exclude_ids=(self.objects["tea"].body_id,))
        o.load()
        self.objects.add_object("faucet_tea", o)
        self.stateful_objects.add_object("faucet_tea", o)

    def _reset_objects(self):
        super(KitchenDualAP, self)._reset_objects()
        z = PBU.stable_z(self.objects["grinder"].body_id, self.objects["drawer"].body_id)
        pos = PU.sample_positions_in_box([-0.25, -0.2], [0.05, 0.05], [z, z])
        self.objects["grinder"].set_position_orientation(pos, skills.ALL_ORIENTATIONS["back"])
        #
        z = PBU.stable_z(self.objects["faucet_coffee"].body_id, self.objects["grinder"].body_id, surface_link=-1)
        pos = PBU.get_pose(self.objects["grinder"].body_id)[0]
        self.objects["faucet_coffee"].set_position_orientation((pos[0], pos[1], z), PBU.unit_quat())

        z = PBU.stable_z(self.objects["tea"].body_id, self.objects["drawer"].body_id) + 0.005
        pos = PU.sample_positions_in_box([-0.15, -0.1], [0.3, 0.3], [z, z])
        self.objects["tea"].set_position_orientation(pos, skills.ALL_ORIENTATIONS["back"])

        z = PBU.stable_z(self.objects["faucet_tea"].body_id, self.objects["tea"].body_id, surface_link=-1)
        pos = PBU.get_pose(self.objects["tea"].body_id)[0]
        self.objects["faucet_tea"].set_position_orientation((pos[0], pos[1], z), PBU.unit_quat())

    def get_task_skeleton(self):
        skeleton = [(
            self.get_constrained_skill_param_sampler("open_prismatic", "drawer"),
            "drawer"
        ), (
            self.get_constrained_skill_param_sampler("grasp", "mug1"),
            "mug1"
        ), (
            self.get_constrained_skill_param_sampler("place", "platform1"),
            "platform1"
        ), (
            self.get_constrained_skill_param_sampler("grasp", "mug1"),
            "mug1"
        )]
        if self.target_skill == "filled_coffee":
            skeleton += [(
                self.get_constrained_skill_param_sampler("place", "faucet_coffee"),
                "faucet_coffee"
            ), (
                self.get_constrained_skill_param_sampler("grasp", "mug1"),
                "mug1"
            ), (
                self.get_constrained_skill_param_sampler("pour", "coffee_machine_platform"),
                "coffee_machine_platform"
            ), (
                self.get_constrained_skill_param_sampler("place", "coffee_machine_platform"),
                "coffee_machine_platform"
            ), (
                self.get_constrained_skill_param_sampler("filled_coffee", "mug1"),
                "mug1"
            )]
        else:
            skeleton += [(
                self.get_constrained_skill_param_sampler("place", "faucet_tea"),
                "faucet_tea"
            ), (
                self.get_constrained_skill_param_sampler("filled_tea", "mug1"),
                "mug1"
            )]
        return skeleton

    def get_constrained_skill_param_sampler(self, skill_name, object_name, num_samples=None):
        if skill_name == "grasp":
            return lambda: self.skill_lib.sample_serialized_skill_params(
                    "grasp", num_samples=num_samples, grasp_orn=dict(choices=[3, 4]))
        elif skill_name == "pour":
            return lambda: self.skill_lib.sample_serialized_skill_params(
                "pour", num_samples=num_samples, pour_pos=dict(sampler_fn=self._get_pour_pos))
        else:
            return lambda: self.skill_lib.sample_serialized_skill_params(skill_name, num_samples=num_samples)


class KitchenDualCoffeeAP(KitchenDualAP):
    def _sample_task(self):
        self.target_skill = "filled_coffee"
        self._task_spec = np.array([self.skill_lib.name_to_skill_index(self.target_skill), self.objects.names.index("mug1")])


class KitchenDualTeaAP(KitchenDualAP):
    def _sample_task(self):
        self.target_skill = "filled_tea"
        self._task_spec = np.array([self.skill_lib.name_to_skill_index(self.target_skill), self.objects.names.index("mug1")])
