"""归属识别、CARLA 隔离及 PID 安全；信号测试只操作本测试创建的进程。"""
from dataclasses import replace
from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import Mock
from vanet.project_paths import (PROJECT_DIRECTORY,EXPERIMENT_RESULTS_DIRECTORY,
    SHUTDOWN_CARLA_ENTRY_FILE,SHUTDOWN_SUMO_GUI_FILE,SHUTDOWN_LINCOLN_RUNTIME_DIRECTORY,
    SHUTDOWN_PROJECT_ENTRY_FILES)
from vanet.experiments.process_shutdown.process_inventory import Process,discover,classify,read_process,TERMINAL_MODULE
from vanet.experiments.process_shutdown.shutdown import send_signal,terminate


class ProcessInventoryTests(unittest.TestCase):
    def process(self,pid,parent,*args,tty=0,cwd=PROJECT_DIRECTORY):
        return Process(pid,parent,1000+pid,tty,tuple(args),cwd)

    def test_owned_trees_are_selected_without_carla_caller_or_unrelated_apps(self):
        p=self.process
        processes=[
            p(1,0,'gnome-terminal-server'),
            p(10,1,'bash',tty=1),p(11,10,'bash',str(SHUTDOWN_CARLA_ENTRY_FILE),tty=1),p(12,11,'docker','start'),
            p(20,1,'python','-m',TERMINAL_MODULE,'--folder',str(EXPERIMENT_RESULTS_DIRECTORY/'launch_test/logs'),'--name','SUMO',tty=2),
            p(21,20,'bash','-c','source setup; run_sumo'),
            p(22,21,'python','-u',str(SHUTDOWN_PROJECT_ENTRY_FILES['SUMO'][0])),p(23,22,str(SHUTDOWN_SUMO_GUI_FILE)),
            p(30,1,'bash',tty=3),p(31,30,'python',str(SHUTDOWN_PROJECT_ENTRY_FILES['Lincoln'][0]),tty=3),
            p(32,31,'rviz2','-d',str(SHUTDOWN_LINCOLN_RUNTIME_DIRECTORY/'test/lincoln.rviz')),
            p(40,1,'python','-c','print("'+str(PROJECT_DIRECTORY)+'")'),
            p(41,1,'rviz2','-d',str(PROJECT_DIRECTORY.parent/'other.rviz')),
            p(50,1,'bash',tty=4),p(51,50,'python','-m','vanet.experiments.process_shutdown.shutdown',tty=4)]
        owned,shells,protected=discover({p.pid:p for p in processes},caller_pid=51)
        self.assertEqual(set(owned),{20,21,22,23,31,32})
        self.assertEqual(set(shells),{30})
        self.assertTrue({1,10,11,12,50,51}<=protected)
        self.assertEqual(owned[22][1:],('SUMO','entry'))
        self.assertEqual(owned[20][1:],('SUMO','terminal'))

    def test_other_checkout_and_shell_command_text_are_not_owned(self):
        other=PROJECT_DIRECTORY.parent/'OtherProject'
        self.assertIsNone(classify(self.process(5,1,'python','-m','vanet.omnet.build_and_launch.run_network',cwd=other)))
        self.assertIsNone(classify(self.process(6,1,'bash','-lc','echo '+str(SHUTDOWN_PROJECT_ENTRY_FILES['Lincoln'][0]))))
        self.assertIsNone(classify(self.process(7,1,'python','-c','print(1)',str(SHUTDOWN_PROJECT_ENTRY_FILES['Lincoln'][0]))))

    def test_carla_terminal_and_its_children_are_never_targets(self):
        p=self.process
        processes=[p(1,0,'gnome-terminal-server'),p(2,1,'python','-m',TERMINAL_MODULE,'--name','CARLA',
            '--folder',str(EXPERIMENT_RESULTS_DIRECTORY/'launch_test/logs')),p(3,2,'bash',str(SHUTDOWN_CARLA_ENTRY_FILE)),
            p(4,3,'CarlaUE4-Linux-Shipping'),p(9,1,'python','-m','unrelated')]
        owned,shells,protected=discover({p.pid:p for p in processes},caller_pid=9)
        self.assertFalse(owned);self.assertFalse(shells)
        self.assertTrue({2,3,4}<=protected)


class IsolatedSignalTests(unittest.TestCase):
    def child(self,ignore=False):
        action='signal.SIG_IGN' if ignore else 'lambda *args:sys.exit(0)'
        code='import signal,sys,time; signal.signal(signal.SIGINT,'+action+'); signal.signal(signal.SIGTERM,'+action+'); print("READY",flush=True); time.sleep(60)'
        child=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True,start_new_session=True)
        self.addCleanup(self.cleanup_child,child)
        self.assertEqual(child.stdout.readline().strip(),'READY')
        return child

    @staticmethod
    def cleanup_child(child):
        if child.poll() is None:child.kill()
        child.wait(timeout=3)
        child.stdout.close()

    def test_stale_identity_and_protected_process_are_not_signalled(self):
        child=self.child();identity=read_process(child.pid)
        self.assertFalse(send_signal(replace(identity,started=identity.started+1),signal.SIGTERM))
        self.assertFalse(send_signal(identity,signal.SIGTERM,{child.pid}))
        self.assertIsNone(child.poll())

    def test_graceful_shutdown_uses_only_the_selected_isolated_process(self):
        child=self.child();sentinel=self.child();log=Mock()
        self.assertEqual(terminate([read_process(child.pid)],log,set(),grace=1.),[])
        self.assertEqual(child.wait(timeout=2),0)
        self.assertIsNone(sentinel.poll())
        self.assertFalse(any(call.kwargs.get('forced') for call in log.emit.call_args_list))

    def test_unresponsive_isolated_process_is_escalated_without_killing_sentinel(self):
        child=self.child(ignore=True);sentinel=self.child();log=Mock()
        self.assertEqual(terminate([read_process(child.pid)],log,set(),grace=.05),[])
        self.assertEqual(child.wait(timeout=2),-signal.SIGKILL)
        self.assertIsNone(sentinel.poll())
        self.assertTrue(any(call.kwargs.get('forced') for call in log.emit.call_args_list))
