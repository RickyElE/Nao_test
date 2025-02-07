from controller import Supervisor, Robot, Motion, motion
import json

### Supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())
NodeInitialSign = False
nodeSheet = {}
nodePosition = {}
nodeOrientation = {}
emitter = supervisor.getDevice("emitter")
if emitter:
    emitter.setChannel(1)
    # print(emitter.getRange())
else:
    print("emitter not set")
receiver = supervisor.getDevice("receiver")
if receiver:
    receiver.setChannel(1)
    receiver.enable(time_step)
else:
    print("receiver not set")

def initialNeedingNode():
    print("Initial Needing Node")
    root = supervisor.getRoot()
    children = root.getField('children')
    temp_nodeSheet = {}
    # striker_red = football = None
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        # print(node.getDef())
        if node.getDef() == 'STRIKER_RED':
            striker_red = node
            temp_nodeSheet["striker_red"] = striker_red
            nodePosition["striker_red"] = None
            nodeOrientation["striker_red"] = None

        if node.getDef() == 'FOOTBALL':
            football = node
            temp_nodeSheet["football"] = football
            nodePosition["football"] = None
            nodeOrientation["football"] = None

        if node.getDef() == 'GOALKEEPER_RED':
            goalkeeper_red = node
            temp_nodeSheet["goalkeeper_red"] = goalkeeper_red
            nodePosition["goalkeeper_red"] = None
            nodeOrientation["goalkeeper_red"] = None

        if node.getDef() == 'STADIUMGOAL_RED':
            stadiumgoal_red = node
            temp_nodeSheet["stadiumgoal_red"] = stadiumgoal_red
            nodePosition["stadiumgoal_red"] = None
            nodeOrientation["stadiumgoal_red"] = None

        if node.getDef() == 'STADIUMGOAL_BLUE':
            stadiumgoal_blue = node
            temp_nodeSheet["stadiumgoal_blue"] = stadiumgoal_blue
            nodePosition["stadiumgoal_blue"] = None
            nodeOrientation["stadiumgoal_blue"] = None
    # print(temp_nodeSheet)
    return temp_nodeSheet

def Refresh_Position():
    global NodeInitialSign, nodeSheet
    # nodeSheet = {}
    if not NodeInitialSign:
        print("Is Initialling!")
        nodeSheet = initialNeedingNode()
        for node in nodeSheet.values():
            if node is None:
                print("node does not exist")
                return False
        NodeInitialSign = True
    # print(nodeSheet)
    for i in nodeSheet.keys():
        # print(i)
        nodePosition[i] = nodeSheet[i].getPosition()
        # print(f"{i} position: {nodePosition[i]}")
        nodeOrientation[i] = nodeSheet[i].getOrientation()
        # print(nodePosition[i])
        print(f"{i}: {nodeOrientation[i]}")
    shared_info = {
        "striker_red": {"position": nodePosition["striker_red"], "orientation": nodeOrientation["striker_red"]},
        # "defender_right": {"position": nodePosition["defender_right"], "orientation": nodeOrientation["defender_right"], "isholdingball": 0},
        "football": {"position": nodePosition["football"], "orientation": nodeOrientation["football"]},
        "goalkeeper_red": {"position": nodePosition["goalkeeper_red"], "orientation": nodeOrientation["goalkeeper_red"]},
        "stadiumgoal_red": {"position": nodePosition["stadiumgoal_red"], "orientation": nodeOrientation["stadiumgoal_red"]},
        "stadiumgoal_blue": {"position": nodePosition["stadiumgoal_blue"], "orientation": nodeOrientation["stadiumgoal_blue"]},
    }
    # print(shared_info)
    temp = json.dumps(shared_info).encode("utf-8")
    emitter.send(temp)

while supervisor.step(time_step) != -1:
    # print(receiver.getQueueLength())
    # if receiver.getQueueLength() > 0:
    #     data = receiver.getData().decode("utf-8")
    #     shared_info = json.loads(data)
    #     # print("shared_info:", shared_info)
    #     # print("type:", )
    #     receiver.nextPacket()
    pass
    if Refresh_Position() is False:
        break