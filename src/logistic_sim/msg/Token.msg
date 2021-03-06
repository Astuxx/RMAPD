Header      HEADER
uint32      ID_SENDER
uint32      ID_RECEIVER
# status flags
bool        INIT
bool        ALLOCATE
bool        CHECK   # check path validity
bool        CALCULATE_PATHS
bool        NEW_MISSIONS_AVAILABLE  # to tell agents that new missions are available inside the token
bool        GOOD_PATHS  # the token paths are good
bool        SHUTDOWN    # to tell agents to stop simulation
bool        ALL_MISSIONS_INSERTED   # set by task planner to notify that all missions have been inserted
bool        SYNCED_ROBOTS # used to check agents synchronization on goal
bool        NEED_REPAIR # used by planner to tell agents that they need to repair their plans
bool        REPAIR  # start of the repair phase
bool        SINGLE_PLAN_REPAIR  # single robot repair phase
bool        SINGLE_PLAN_REPAIR_PROGRESS # tells if at least a robot managed to plan in this token cycle
bool        MULTI_PLAN_REPAIR   # multi-robot plan repair phase
bool        DCOP_UTIL_PHASE          # dcop util phase execution
bool        DCOP_VALUE_PHASE

Mission[]   MISSION
int32[]     TAKEN_MISSION   # used by CFree to track assigned missions

# robots' positions on the map
float64[]   X_POS
float64[]   Y_POS

# current and next vertices of agents
uint32[]    CURR_VERTEX
uint32[]    NEXT_VERTEX

# current destination of each agent
uint32[]    CURR_DST

# load capacity of each robot
uint32[]    CAPACITY

# initial vertex of each agent
uint32[]    INIT_POS

# statistics
time[]      MISSION_START_TIME
float32[]   MISSION_CURRENT_DISTANCE
int32[]     INTERFERENCE_STATUS
uint32[]    INTERFERENCE_COUNTER
uint32[]    MISSIONS_COMPLETED
uint32[]    TASKS_COMPLETED
float32[]   TOTAL_DISTANCE
# statistics for repair variants
uint32      OBSTACLE_EVENTS
uint32      SUCCESSFULL_SA_REPAIR
uint32[]    REPAIRS_PER_ROBOT
uint32      SUCCESSFULL_MA_REPAIR
uint32      FAILED_REPAIR

uint32[]    GOAL_STATUS
Path[]      TRAILS
Path[]      NEW_TRAILS
Path[]      HOME_TRAILS # last part of paths, dedicated to home return
uint32      INIT_POS_INDEX
bool[]      REACHED_HOME
int32       ACTIVE_ROBOTS
bool[]      FAILED_ALLOCATION # keep track of agents that have failed task allocation

# end simulation flag
bool        END_SIMULATION

# used for MAPD search
MAPDState   CURRENT_MAPD_STATE
Waypoints[] ROBOT_WAYPOINTS
# vector of edge removed from the last token cycle
Edge[]      REMOVED_EDGES
Edge[]      ADDED_EDGES
# vector that tells whether robots have repaired their paths
bool[]      HAS_REPAIRED_PATH
int32[]     NEW_HOMES # field set to -1 if no new home has been assigned