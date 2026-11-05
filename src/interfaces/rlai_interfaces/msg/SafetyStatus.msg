# Forward-path safety assessment produced by the C++ safety monitor.

std_msgs/Header header

# True when the forward sector is closer than the configured safety distance.
bool blocked

# Closest valid range in the forward sector, metres.
float32 min_distance

# Human readable reason, e.g. "forward sector 0.22 m".
string reason
