"""只在 TraCI 阻塞接收时释放 GUI 网络绘图锁，执行命令前恢复。"""
from pathlib import Path
from vanet.project_paths import SUMO_GUI_TRACI_RENDER_LOCK_HEADER_FILE

PAINT_TRACE_ENVIRONMENT_VARIABLE='AUTODRIVE_SUMO_GUI_PAINT_TRACE'
DISABLE_READ_UNLOCK_ENVIRONMENT_VARIABLE='AUTODRIVE_SUMO_GUI_DISABLE_READ_UNLOCK'
SUPPORT_FILES=(SUMO_GUI_TRACI_RENDER_LOCK_HEADER_FILE,)
INCLUDE='#include <config.h>'
WITH_HEADER=INCLUDE+'\n#include "'+SUMO_GUI_TRACI_RENDER_LOCK_HEADER_FILE.name+'"'


def source_patches():
    return {
        Path('guisim/GUINet.cpp'):[(INCLUDE,WITH_HEADER),
            ('''GUINet::simulationStep() {
    FXMutexLock locker(myLock);
    MSNet::simulationStep();''','''GUINet::simulationStep() {
    FXMutexLock locker(myLock);
    autodrive_gui::SimulationLockScope guiLock(this,
        [](void* net) { static_cast<GUINet*>(net)->unlock(); },
        [](void* net) { static_cast<GUINet*>(net)->lock(); });
    MSNet::simulationStep();''')],
        Path('traci-server/TraCIServer.cpp'):[(INCLUDE,WITH_HEADER),
            ('''// Read next request
                        myInputStorage.reset();
                        myCurrentSocket->second->socket->receiveExact(myInputStorage);''','''// Wait without holding the GUI drawing lock. The scope ends
                        // before dispatchCommand(), so state mutation stays locked.
                        myInputStorage.reset();
                        {
                            autodrive_gui::SocketReadScope idleRead;
                            myCurrentSocket->second->socket->receiveExact(myInputStorage);
                        }''')],
        Path('gui/GUIViewTraffic.cpp'):[(INCLUDE,WITH_HEADER),
            ('''    return hits2;
}''','''    autodrive_gui::traceCompletedPaint();
    return hits2;
}''')],
    }
