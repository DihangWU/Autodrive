"""编译两个翻译单元，验证阻塞读期间解锁、异常恢复和线程隔离。"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from vanet.project_paths import SUMO_GUI_TRACI_RENDER_LOCK_HEADER_FILE


class SumoGuiReadLockTests(unittest.TestCase):
    def test_native_lock_scopes_across_translation_units(self):
        compiler=shutil.which('g++')
        if compiler is None:self.skipTest('Native SUMO GUI compiler unavailable')
        with tempfile.TemporaryDirectory(prefix='sumo_gui_lock_') as folder:
            folder=Path(folder)
            helper=folder/'receive.cpp';main=folder/'main.cpp';binary=folder/'lock_check'
            helper.write_text('''#include "traci_render_lock.hpp"
#include <functional>
void receiveProbe(const std::function<void()>& body) {
    autodrive_gui::SocketReadScope waiting;
    body();
}
''')
            main.write_text('''#include "traci_render_lock.hpp"
#include <cassert>
#include <functional>
#include <future>
#include <mutex>
#include <stdexcept>
void receiveProbe(const std::function<void()>&);
bool available(std::mutex& mutex) {
    return std::async(std::launch::async, [&]() {
        if (!mutex.try_lock()) return false;
        mutex.unlock(); return true;
    }).get();
}
int main() {
    std::mutex mutex;
    auto release=[](void* p) { static_cast<std::mutex*>(p)->unlock(); };
    auto acquire=[](void* p) { static_cast<std::mutex*>(p)->lock(); };
    receiveProbe([&]() { assert(available(mutex)); });
    {
        std::unique_lock<std::mutex> lock(mutex);
        autodrive_gui::SimulationLockScope context(&mutex,release,acquire);
        assert(!available(mutex));
        receiveProbe([&]() { assert(available(mutex)); });
        assert(!available(mutex));
        try { receiveProbe([&]() { throw std::runtime_error("closed socket"); }); }
        catch (const std::runtime_error&) {}
        assert(!available(mutex));
        std::async(std::launch::async, [&]() {
            receiveProbe([&]() { assert(!available(mutex)); });
        }).get();
        {
            std::mutex inner;
            std::unique_lock<std::mutex> innerLock(inner);
            autodrive_gui::SimulationLockScope nested(&inner,release,acquire);
            receiveProbe([&]() { assert(available(inner)); assert(!available(mutex)); });
        }
        receiveProbe([&]() { assert(available(mutex)); });
    }
    assert(autodrive_gui::currentLock()==nullptr);
    assert(available(mutex));
}
''')
            env=dict(os.environ);env.pop('AUTODRIVE_SUMO_GUI_DISABLE_READ_UNLOCK',None)
            subprocess.run([compiler,'-std=c++17','-pthread','-I'+str(SUMO_GUI_TRACI_RENDER_LOCK_HEADER_FILE.parent),
                            str(main),str(helper),'-o',str(binary)],check=True,capture_output=True,text=True,timeout=30)
            subprocess.run([str(binary)],check=True,capture_output=True,text=True,env=env,timeout=10)
