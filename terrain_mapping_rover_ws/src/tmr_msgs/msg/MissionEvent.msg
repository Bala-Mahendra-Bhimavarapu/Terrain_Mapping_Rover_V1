# Human-readable mission event
std_msgs/Header header

string level              # "INFO", "WARNING", "ERROR", "CRITICAL"
string source             # Node that generated the event
string message            # Human-readable message
string event_type         # Event category (e.g., "SLAM", "NAV", "LANDMARK")
