from panda_gym.pybullet import PyBullet
import os
from contextlib import contextmanager
from typing import Any, Dict, Iterator, Optional

import numpy as np
import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

import panda_gym.assets

class PybulletCup(PyBullet):
    def create_cup(
        self,
        body_name: str,
        half_extents: np.ndarray,
        mass: float,
        position: np.ndarray,
        rgba_color: Optional[np.ndarray] = None,
        specular_color: Optional[np.ndarray] = None,
        ghost: bool = False,
        lateral_friction: Optional[float] = None,
        spinning_friction: Optional[float] = None,
        texture: Optional[str] = None,
    ) -> None:
        """Create a box.

        Args:
            body_name (str): The name of the body. Must be unique in the sim.
            half_extents (np.ndarray): Half size of the box in meters, as (x, y, z).
            mass (float): The mass in kg.
            position (np.ndarray): The position, as (x, y, z).
            rgba_color (np.ndarray, optional): Body color, as (r, g, b, a). Defaults as [0, 0, 0, 0]
            specular_color (np.ndarray, optional): Specular color, as (r, g, b). Defaults to [0, 0, 0].
            ghost (bool, optional): Whether the body can collide. Defaults to False.
            lateral_friction (float or None, optional): Lateral friction. If None, use the default pybullet
                value. Defaults to None.
            spinning_friction (float or None, optional): Spinning friction. If None, use the default pybullet
                value. Defaults to None.
            texture (str or None, optional): Texture file name. Defaults to None.
        """
        rgba_color = rgba_color if rgba_color is not None else np.zeros(4)
        specular_color = specular_color if specular_color is not None else np.zeros(3)
        visual_kwargs = {
            "halfExtents": half_extents,
            "specularColor": specular_color,
            "rgbaColor": rgba_color,
        }
        collision_kwargs = {"halfExtents": half_extents}
        self._create_geometry(
            body_name,
            geom_type=self.physics_client.GEOM_CYLINDER,
            mass=mass,
            position=position,
            ghost=ghost,
            lateral_friction=lateral_friction,
            spinning_friction=spinning_friction,
            visual_kwargs=visual_kwargs,
            collision_kwargs=collision_kwargs,
        )
        if texture is not None:
            texture_path = os.path.join(panda_gym.assets.get_data_path(), texture)
            texture_uid = self.physics_client.loadTexture(texture_path)
            self.physics_client.changeVisualShape(self._bodies_idx[body_name], -1, textureUniqueId=texture_uid)

