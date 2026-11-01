"""真实运行实体与DDS图端点QoS观测；未知值保持未知，不修改生产配置。"""
def profile_data(profile):
    def name(value):
        return getattr(value,'name',str(value))
    history=name(profile.history)
    depth=int(profile.depth)
    return dict(reliability=name(profile.reliability),durability=name(profile.durability),
                history=history,depth=depth if history=='KEEP_LAST' and depth>0 else None)

def entity_rows(node):
    rows=[]
    for direction,entities in [('publishers',node.publishers),('subscriptions',node.subscriptions)]:
        for entity in entities:
            rows.append(dict(node_name=node.get_name(),direction=direction,topic=entity.topic_name,
                             source='node_entity',qos=profile_data(entity.qos_profile)))
    return rows
