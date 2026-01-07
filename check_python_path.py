import sys
import os

print("--- sys.path ---")
for p in sys.path:
    print(p)
print("----------------")

try:
    import openarm_recorder.openarm_recorder.recorder_node
    print("Successfully imported openarm_recorder.openarm_recorder.recorder_node")
except ImportError as e:
    print(f"Failed to import openarm_recorder.openarm_recorder.recorder_node: {e}")

try:
    import openarm_recorder.recorder_node
    print("Successfully imported openarm_recorder.recorder_node")
except ImportError as e:
    print(f"Failed to import openarm_recorder.recorder_node: {e}")
