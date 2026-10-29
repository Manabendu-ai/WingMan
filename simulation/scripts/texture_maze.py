from pathlib import Path
import re

world = Path.home() / "PX4-Autopilot/Tools/simulation/gz/worlds/maze_exploration.sdf"
text = world.read_text()

# Give each wall a slightly different material color.
colors = [
    (0.72, 0.76, 0.80),
    (0.82, 0.78, 0.70),
    (0.70, 0.78, 0.74),
    (0.78, 0.72, 0.80),
]

wall_count = 0

def update_wall(match):
    global wall_count
    block = match.group(0)

    # Only modify wall models; leave the ground and markers untouched.
    if not re.search(r'<model name="(?:outer_|inner_)', block):
        return block

    color = colors[wall_count % len(colors)]
    rgb = f"{color[0]} {color[1]} {color[2]} 1"
    block = re.sub(
        r'<(ambient|diffuse)>[^<]+</\1>',
        lambda m: f'<{m.group(1)}>{rgb}</{m.group(1)}>',
        block
    )
    wall_count += 1
    return block

# Each wall is a one-line model in this world file.
text = re.sub(
    r'<model name="(?:outer_|inner_)[^"]+".*?</model>',
    update_wall,
    text
)

world.write_text(text)
print(f"Updated wall colors in {wall_count} wall models.")
print("Geometry and collision definitions were not changed.")
