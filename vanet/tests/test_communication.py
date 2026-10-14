"""Real radio handshake/traffic feedback and overlay geometry regressions."""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import logging
import tempfile

from test_radio_range import run_case, load_config
from cobridge.common.paths import VISUAL_DIR
sys.path.insert(0, str(VISUAL_DIR))
from communication_overlay import COLORS, colored_segments, project_segment


def main():
    logging.basicConfig(level=logging.ERROR)
    cfg = dict(load_config(), rsus=[])
    with tempfile.TemporaryDirectory(prefix='vanet-communication-') as folder:
        tmp = Path(folder)
        for name, options, expected in [
            ('quiet', ['--**.appl.sendProbes=false'], {0}),
            ('first_sends', ['--*.lincoln1.appl.sendProbes=false'], {0, 1}),
            ('second_sends', ['--*.lincoln0.appl.sendProbes=false'], {0, 2}),
            ('duplex', [], {0, 1, 2, 3}),
            ('unconfirmed', ['--*.lincoln1.appl.sendHello=false'], set()),
        ]:
            feedback = []
            rows, _, _ = run_case(tmp, name, cfg, lambda _: [(700, 625), (730, 625)],
                                 extra_args=options, visualization=feedback)
            masks = {link[2] for state in feedback for link in state['links']}
            if name == 'duplex':
                assert 0 in masks and 3 in masks, masks
            else:
                assert masks == expected, (name, masks)
            if name == 'unconfirmed':
                assert rows, 'Data should still traverse the wireless MAC/PHY'
            for state in feedback:
                for a, b, _ in state['links']:
                    assert set((a,b)) == {'lincoln0','lincoln1'}
                    assert state['nodes'][a][2] == 2
            print('PASS:', name, 'actual HELLO/ACK + traffic feedback')

        feedback = []
        run_case(tmp, 'outside', cfg, lambda _: [(700,625),(740.01,625)], visualization=feedback)
        assert feedback and not any(state['links'] for state in feedback)
        print('PASS: out-of-range nodes cannot handshake or display a CARLA line')

        feedback = []
        run_case(tmp, 'dynamic_rsu', cfg, lambda _: [(580,625),(820,625)], duration=9,
                 rsu_provider=lambda frame: [('rsu0',700,625,60)] if 60<=frame<120 else [],
                 visualization=feedback)
        assert any(state['links'] for state in feedback if 60<=state['frame']<120)
        assert not any(state['links'] for state in feedback if state['frame']<60 or state['frame']>=120)
        assert any(link[2] for state in feedback for link in state['links'])
        assert all('rsu0' in link[:2] for state in feedback for link in state['links'])
        print('PASS: live RSU creation/removal updates only the real physical-hop lines')

    nodes = {'a':[0,0,2], 'b':[40,0,60]}
    for mode in (0,1,2,3):
        segments = list(colored_segments([['a','b',mode]], nodes))
        assert len(segments) == (1 if mode in (0,3) else 2)
        if mode in (1,2):
            assert segments[0][1] == segments[1][0] == [20,0,31]
            assert segments[0][2] == COLORS['tx' if mode==1 else 'rx']
            assert segments[1][2] == COLORS['rx' if mode==1 else 'tx']
        else:
            assert segments[0][2] == COLORS['idle' if mode==0 else 'duplex']
        assert all(0<segment[2][3]<255 for segment in segments)
    camera = dict(location=[0,0,0],forward=[1,0,0],right=[0,1,0],up=[0,0,1])
    assert project_segment([10,-2,2],[10,2,2],camera,1000,500) == [(400,150),(600,150)]
    assert project_segment([-1,0,0],[-2,1,0],camera,1000,500) is None
    assert project_segment([-1,0,0],[10,1,0],camera,1000,500) is not None
    print('PASS: midpoint splitting, blue/yellow direction, green duplex, alpha and view projection')


if __name__ == '__main__':
    main()
