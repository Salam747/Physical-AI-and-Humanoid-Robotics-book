# Simplified Isaac Sim script for domain randomization
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Initialize world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Add a cuboid with random properties
cuboid = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cuboid",
        name="random_cuboid",
        position=np.array([0, 0, 0.5]),
        scale=np.array([np.random.rand() * 0.5 + 0.1 for _ in range(3)]),
        color=np.array([np.random.rand(), np.random.rand(), np.random.rand()]),
    )
)

# Start simulation (actual Isaac Sim environment setup would be more complex)
# world.reset()
# while world.is_running():
#     world.step(render=True)
