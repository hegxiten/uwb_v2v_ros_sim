MINIMUM_DISTANCE = 1.05  # meters
MAXIMUM_DISTANCE = 500  # meters
STALE_THRESHOLD = 0.2  # seconds

TRACK_CENTERLINE_OFFSET = 1.2  # meters
LONGITUDINAL_OFFSET = 0.0  # meters
RAIL_TOP_OFFSET = 4.0  # meters
VEHICLE_LENGTH = 14.2  # meters

UWB_SLAVE_A_DEFAULT_INFO_POS = [TRACK_CENTERLINE_OFFSET, LONGITUDINAL_OFFSET, RAIL_TOP_OFFSET]
UWB_MASTER_A_DEFAULT_INFO_POS = [-TRACK_CENTERLINE_OFFSET, LONGITUDINAL_OFFSET, RAIL_TOP_OFFSET]
UWB_SLAVE_B_DEFAULT_INFO_POS = [TRACK_CENTERLINE_OFFSET, LONGITUDINAL_OFFSET, RAIL_TOP_OFFSET]
UWB_MASTER_B_DEFAULT_INFO_POS = [-TRACK_CENTERLINE_OFFSET, LONGITUDINAL_OFFSET, RAIL_TOP_OFFSET]

def get_topics(robot_num):
    TOPICS = ['/uwb_aloha_networking', '/clock', '/tf', '/tf_static', '/rosout', '/rosout_agg']
    TOPICS += [f'/robot{i}/odom' for i in range(robot_num)]
    TOPICS += [f'/robot{i}/cmd_vel' for i in range(robot_num)]
    TOPICS += [f'/robot{i}/robot_dist' for i in range(robot_num)]
    TOPICS += [f'/robot{i}/twr_ranges' for i in range(robot_num)]
    return TOPICS


