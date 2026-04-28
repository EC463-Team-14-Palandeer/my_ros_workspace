# Test file to see if the RL loads correctly or if that is the issue in the program here.
# If it is the issue --> Need to retrain model by downgrading the venv I'm (Bogdan) using on my machine.
# Otherwise... It'll be bad.

from stable_baselines3 import SAC
print("Imported SB3 successfully")

try:
    print("Attempting to load model...")
    model = SAC.load("/workspaces/isaac_ros-dev/src/robo_cayote_control/models/sac_cayote_curriculum_490000_steps")
    print("SUCCESS! The model loaded despite the version gap.")
except Exception as e:
    print(f"FAILED: {e}")