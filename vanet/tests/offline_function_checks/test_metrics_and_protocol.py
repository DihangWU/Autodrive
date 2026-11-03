import unittest
from unittest.mock import Mock
from vanet.experiments.metrics_and_event_recording.metrics import rectangles_overlap, Metrics
from vanet.protocol.message_validation.messages import finite_number,experiment_group


def vehicle(name,x=0,y=0,z=.75,heading=0):
    return dict(id=name,x=x,y=y,z=z,heading=heading,length=4.,width=2.,height=1.5,speed=0.,role='npc')


class GeometryAndEvents(unittest.TestCase):
    def test_height_and_rotation(self):
        self.assertTrue(rectangles_overlap(vehicle('a'),vehicle('b',x=2,heading=90)))
        self.assertFalse(rectangles_overlap(vehicle('a'),vehicle('b',z=8)))
        self.assertFalse(rectangles_overlap(vehicle('a'),vehicle('b',y=2.1)))

    def test_continuous_overlap_is_one_event_and_reentry_is_new(self):
        log=Mock();m=Metrics(log,dict(step_length=.05,hard_brake_mps2=4.))
        sumo=Mock();sumo.simulation.getCollidingVehiclesIDList.return_value=[]
        sumo.simulation.getStartingTeleportIDList.return_value=[]
        for t in (.05,.1,.15):m.sample(t,[vehicle('a'),vehicle('b')],sumo)
        self.assertEqual(m.values['geometric_collision_events'],1)
        m.sample(.2,[vehicle('a'),vehicle('b',x=10)],sumo)
        m.sample(.25,[vehicle('a'),vehicle('b')],sumo)
        self.assertEqual(m.values['geometric_collision_events'],2)
        self.assertEqual(m.values['sumo_collision_events'],0)

    def test_boolean_and_nonfinite_coordinates_rejected(self):
        for value in (True,float('nan'),float('inf')):
            with self.assertRaises(ValueError):finite_number(value,'x')
        with self.assertRaises(ValueError):experiment_group('v2v_only')

if __name__=='__main__':unittest.main()
