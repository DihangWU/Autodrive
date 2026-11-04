"""在 SUMO 画出与真实无线模块相同的 RSU 有效半径。"""
import json
import math
from vanet.project_paths import EXPERIMENT_DEFAULT_CONFIGURATION_FILE


class CoverageCircle:
    OWNER='vanet.communication_coverage'

    def __init__(self,connection,network,parameters=None):
        parameters=parameters or json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
        self.connection=connection;self.network=network
        self.rsu=parameters['rsu'];self.radius=float(parameters['communication_ranges']['rsu_metres'])
        if not math.isfinite(self.radius) or self.radius<=0:raise ValueError('Invalid RSU coverage radius.')
        self.identifier='vanet_coverage_'+self.rsu['id'];self.created=False

    def setup(self,physical_rsus):
        marker=next((entry for entry in physical_rsus if entry['id']==self.rsu['id']),None)
        if marker is None or any(abs(marker[key]-self.rsu[key])>1e-6 for key in ('x','y')):
            raise ValueError('RSU physical marker and wireless center do not match.')
        polygons=self.connection.polygon
        if self.identifier in polygons.getIDList():
            if polygons.getParameter(self.identifier,'owner')!=self.OWNER:
                raise RuntimeError('Coverage polygon belongs to another component.')
            polygons.remove(self.identifier,layer=90)
        x=self.rsu['x']+self.network['offset'][0];y=-self.rsu['y']+self.network['offset'][1]
        shape=[(x+self.radius*math.cos(i*2*math.pi/128),y+self.radius*math.sin(i*2*math.pi/128)) for i in range(129)]
        polygons.add(self.identifier,shape,(20,210,85,255),fill=False,
                     polygonType='vanet_rsu_communication_range',layer=90,lineWidth=1.5)
        self.created=True
        try:
            polygons.setParameter(self.identifier,'owner',self.OWNER)
            polygons.setParameter(self.identifier,'radius_metres',str(self.radius))
            self.connection.poi.setParameter(self.rsu['id'],'communication_radius_metres',str(self.radius))
        except Exception:
            polygons.remove(self.identifier,layer=90);self.created=False;raise

    def close(self):
        polygons=self.connection.polygon
        if self.created and self.identifier in polygons.getIDList() and polygons.getParameter(self.identifier,'owner')==self.OWNER:
            polygons.remove(self.identifier,layer=90)
        self.created=False
