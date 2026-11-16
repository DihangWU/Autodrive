"""显式人工操作入口；一键启动环境不会自动开启紧急状态。"""
import argparse
import json
from foundation.cobridge.control import ControlClient
from vanet.protocol.message_validation.messages import GROUPS
from vanet.project_paths import SUMO_TOWN04_NETWORK_FILE
from foundation.sumo.client import SumoClient
from foundation.cobridge.network import read_network


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=['status','initialize','reset','prepare','start','cancel'],default='status',nargs='?')
    parser.add_argument('--group',choices=GROUPS)
    args=parser.parse_args()
    client=ControlClient()
    try:
        action='vanet.scene.initialize' if args.operation=='initialize' else 'vanet.'+args.operation
        network_file = SUMO_TOWN04_NETWORK_FILE
        if args.operation not in ('status','initialize'):
            sumo = SumoClient()
            try:network_file = read_network(sumo.status()['config'])['network_file']
            finally:sumo.close()
        fields=dict(source='operator_cli',network_file=str(network_file))
        if args.group:fields['group']=args.group
        response=client.command(action,**fields)
        print(json.dumps(response,ensure_ascii=False,indent=2))
    finally:client.close()

if __name__=='__main__':main()
