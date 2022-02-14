# Task name.
string name

# Task state.
string state

# Absolute sim time when this task will be in "ready" state (ROS time).
Time ready_time

# Absolute sim time when this task will be in "running" state (ROS time).
Time running_time


# Time elapsed since task started (ROS time). This is the current sim time
# minus start time.
# When the elapsed time reaches the task timeout, `timed_out` is set to true.
Duration elapsed_time

# Remaining time until the task times out (ROS time).
Duration remaining_time

# True if task timed out.
bool timed_out

# Number of collisions.
uint32 num_collisions

# The score.
float64 score