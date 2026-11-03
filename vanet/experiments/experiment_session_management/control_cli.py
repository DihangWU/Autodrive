"""显式人工操作入口；一键启动环境不会自动开启紧急状态。"""
import argparse
import json
from foundation.cobridge.control import ControlClient
from vanet.project_paths import SUMO_TOWN04_NETWORK_FILE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=['status','initialize','reset','prepare','start','cancel'],default='status',nargs='?')
    parser.add_argument('--group',choices=['normal','emergency','cooperative'])
    args=parser.parse_args()
    client=ControlClient()
    try:
        action='vanet.scene.initialize' if args.operation=='initialize' else 'vanet.'+args.operation
        fields=dict(source='operator_cli',network_file=str(SUMO_TOWN04_NETWORK_FILE))
        if args.group:fields['group']=args.group
        response=client.command(action,**fields)
        print(json.dumps(response,ensure_ascii=False,indent=2))
    finally:client.close()

if __name__=='__main__':main()
