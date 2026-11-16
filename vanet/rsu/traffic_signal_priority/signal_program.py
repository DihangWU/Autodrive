"""保存、优先放行及恢复原有 SUMO 配时，不直接写 CARLA 灯态。"""


def save_program(connection, controller):
    lights=connection.trafficlight
    return dict(program=lights.getProgram(controller),phase=lights.getPhase(controller),
                remaining=max(.001,lights.getNextSwitch(controller)-connection.simulation.getTime()),
                state=lights.getRedYellowGreenState(controller))


def restore_program(connection, controller, saved):
    lights=connection.trafficlight
    if saved['program']=='online':
        lights.setRedYellowGreenState(controller,saved['state'])
    else:
        lights.setProgram(controller,saved['program'])
        lights.setPhase(controller,saved['phase'])
        lights.setPhaseDuration(controller,saved['remaining'])
