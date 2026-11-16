"""Mode2 只允许主路左数第 4 道向第 3 道避让。"""


def fourth_lane(connection, name):
    road=connection.vehicle.getRoadID(name)
    index=connection.vehicle.getLaneIndex(name)
    count=connection.edge.getLaneNumber(road)
    # SUMO counts from the right. Town04's forbidden shoulder at index 0
    # does not affect counting from the leftmost highway lane.
    return count-index==4
