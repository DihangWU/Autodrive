"""NPC 的唯一业务通道：近距离直连，超距且双端覆盖时走 RSU。"""


class NpcCommunicationPath:
    def __init__(self, rsu_id):
        self.rsu_id = rsu_id
        self.configured = False
        self.path = None

    def target(self):
        if not self.configured:return self.rsu_id
        return 'lincoln0' if self.path == 'direct' else self.rsu_id if self.path == 'rsu' else None

    def filter_peers(self, peers):
        if not self.configured:return peers
        target = self.target()
        return [peer for peer in peers if peer == target]
