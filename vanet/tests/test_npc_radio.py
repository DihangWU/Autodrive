"""Real Veins radios: shared JSON ranges, NPC naming/lifecycle and 200 OBUs."""
import copy
import json
import logging
from pathlib import Path
import tempfile

from test_radio_range import load_config, run_case, received


def main():
    logging.basicConfig(level=logging.ERROR)
    base = dict(load_config(), rsus=[])
    with tempfile.TemporaryDirectory(prefix='vanet-npc-') as directory:
        tmp = Path(directory)
        mixed_names = {101: 'lincoln0', 102: 'npc_testbatch_0'}
        feedback = []
        rows, _, _ = run_case(tmp, 'mixed', base, lambda _: [(700,625),(730,625)],
                              roles=['lincoln_driver','sumo_driver'], vehicle_names=mixed_names,
                              visualization=feedback)
        for a, b in [('lincoln0','npc_testbatch_0'), ('npc_testbatch_0','lincoln0')]:
            assert received(rows, a, b)
        assert any(s['links'] for s in feedback)
        assert all(set(s['nodes']) == set(mixed_names.values()) for s in feedback)
        print('PASS: SUMO-driven NPC and Lincoln use their SUMO names, exchange packets and report animation links.')

        cfg = copy.deepcopy(base)
        cfg['communication']['vehicle_range'] = 65
        cases = [('default60', {}, 60, True),
                 ('default66', {}, 66, False),
                 ('short_npc', {'npc1':20}, 30, False),
                 ('one_long_npc', {'npc1':90}, 75, False),
                 ('both_long', {'npc0':90,'npc1':90}, 75, True)]
        for name, overrides, distance, allowed in cases:
            cfg['communication']['range_overrides'] = overrides
            feedback = []
            rows, log, _ = run_case(tmp, name, cfg, lambda _, d=distance: [(700,625),(700+d,625)],
                                    roles=['sumo_driver','sumo_driver'], visualization=feedback)
            for a, b in [('npc0','npc1'), ('npc1','npc0')]:
                assert bool(received(rows, a, b)) == allowed, (name, a, b)
            assert any(s['links'] for s in feedback) == allowed, name
            print('PASS:', name, 'JSON ranges govern PHY reception, handshake and CARLA links.')

        cfg['communication'] = dict(vehicle_range=5, rsu_range=300, range_overrides={})
        for name, override, distance, allowed in [('rsu_default', {}, 250, True),
                                                  ('rsu_override_inside', {'rsu0':120}, 100, True),
                                                  ('rsu_override_outside', {'rsu0':120}, 150, False)]:
            cfg['communication']['range_overrides'] = override
            feedback = []
            rows, _, _ = run_case(tmp, name, cfg, lambda _, d=distance: [(700+d,625)],
                                  roles=['sumo_driver'], rsu_provider=lambda _: [('rsu0',700,625,60)],
                                  visualization=feedback)
            for a, b in [('npc0','rsu0'), ('rsu0','npc0')]:
                assert bool(received(rows, a, b)) == allowed, (name,a,b)
            assert any(s['links'] for s in feedback) == allowed, name
            print('PASS:', name, 'live RSU uses its own radius in both directions.')

        invalid_path = tmp / 'invalid.json'
        for radio in [dict(vehicle_range=0, rsu_range=220), dict(vehicle_range=True, rsu_range=220),
                      dict(vehicle_range=40, rsu_range=float('nan')),
                      dict(vehicle_range=40, rsu_range=220, range_overrides={'npc*':60}),
                      dict(vehicle_range=40, rsu_range=220, range_overrides={'npc0':-2})]:
            invalid_path.write_text(json.dumps(dict(base, communication=radio)))
            try:
                load_config(invalid_path)
            except ValueError:
                continue
            raise AssertionError('Invalid radio configuration accepted')
        print('PASS: invalid ranges and wildcard overrides are rejected.')

        # Isolated Veins scale test, without adding 200 actors to live CARLA.
        # Keep every synthetic OBU inside Town04's exported 0..899 x,
        # 0..800 y bounds; the former 1000 m row was outside the map.
        points = [(60+80*((i//2)%10)+20*(i%2), 60+70*((i//2)//10)) for i in range(200)]
        feedback = []
        rows, log, _ = run_case(tmp, '200_npcs', base,
                                lambda f: points[:150] if 40<=f<60 else points, duration=5,
                                roles=['sumo_driver']*200,
                                vehicle_names={101+i:f'npc_batch_{i}' for i in range(200)},
                                visualization=feedback, timeout=120,
                                extra_args=['--record-eventlog=false','--cmdenv-express-mode=true'])
        assert {len(s['nodes']) for s in feedback} == {150,200}
        assert received(rows,'npc_batch_0','npc_batch_1')
        assert any(float(r['time'])>3.1 for r in received(rows,'npc_batch_198','npc_batch_199'))
        assert any(s['links'] for s in feedback if 40<=s['frame']<60)
        assert not any('npc_batch_199' in link[:2] for s in feedback
                       if 40<=s['frame']<60 for link in s['links'])
        assert any('npc_batch_199' in link[:2] for s in feedback
                   if s['frame']>=60 for link in s['links'])
        print('PASS: 200 NPC OBUs, removal/recreation of 50, continued packet delivery and animation feedback.')


if __name__ == '__main__':
    main()
