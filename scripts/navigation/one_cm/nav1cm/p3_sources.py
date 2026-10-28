"""地图源上下文的有界身份生命周期；淘汰不删除世界地图历史。"""

from collections import OrderedDict


class SourceContextLRU:
    def __init__(self):
        self.ids = OrderedDict()
        self.capacity = 8
        self.next_id = 1

    def copy(self):
        duplicate = SourceContextLRU()
        duplicate.ids = self.ids.copy()
        duplicate.capacity = self.capacity
        duplicate.next_id = self.next_id
        return duplicate

    def activate(self, key, update):
        """同步 native 淘汰，再按 LRU 使用当前源；注入后端也保持相同上限。"""
        self.capacity = int(update.cache_capacity)
        if self.capacity <= 0:
            raise ValueError("terrain backend returned invalid cache capacity")
        retired = []
        evicted = set(update.evicted_source_ids)
        for old_key, identity in tuple(self.ids.items()):
            if identity in evicted or (old_key == key and update.source_id is not None and
                                       identity != update.source_id):
                retired.append((old_key, self.ids.pop(old_key)))
        if key in self.ids:
            identity = self.ids[key]
        else:
            identity = self.next_id if update.source_id is None else int(update.source_id)
            if identity < self.next_id:
                raise ValueError("terrain backend reused a retired source identity")
            self.next_id = identity + 1
            self.ids[key] = identity
        self.ids.move_to_end(key)
        while len(self.ids) > self.capacity:
            retired.append(self.ids.popitem(last=False))
        return identity, retired


def retire_owners(previous, identities):
    """失活 owner 归为混合并合并相邻行程，三态地图保持不变。"""
    if not identities:
        return previous
    rows = {}
    for y, intervals in previous.items():
        merged = []
        for left, right, owner in intervals:
            value = 0 if owner in identities else owner
            if merged and merged[-1][1] == left and merged[-1][2] == value:
                merged[-1] = (merged[-1][0], right, value)
            else:
                merged.append((left, right, value))
        rows[y] = tuple(merged)
    return rows
