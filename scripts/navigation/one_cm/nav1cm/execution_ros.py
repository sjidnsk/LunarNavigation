"""执行元信息/状态的ROS薄适配；Path几何始终由标准消息提供。"""
from .execution_contract import ExecutionReference,ExecutionStatus


def stamp_ns(header):
    return int(header.stamp.sec)*1000000000+int(header.stamp.nanosec)


def reference_from_message(message):
    return ExecutionReference(bytes(message.session_id.uuid).hex(),int(message.reference_revision),
        str(message.path_digest),int(message.intent),stamp_ns(message.header),
        float(message.position_tolerance_m),float(message.yaw_tolerance_rad),
        float(message.stopped_linear_mps),float(message.stopped_angular_rps))


def reference_message(reference,frame):
    from lunar_planning_msgs.msg import PathExecutionReference
    message=PathExecutionReference();message.header.frame_id=frame
    message.header.stamp.sec,message.header.stamp.nanosec=divmod(reference.stamp_ns,1000000000)
    message.session_id.uuid=list(bytes.fromhex(reference.session_id))
    message.reference_revision=reference.revision;message.path_digest=reference.path_digest
    message.intent=reference.intent
    for key in ('position_tolerance_m','yaw_tolerance_rad','stopped_linear_mps','stopped_angular_rps'):
        setattr(message,key,getattr(reference,key))
    return message


def status_from_message(message):
    return ExecutionStatus(bytes(message.session_id.uuid).hex(),int(message.segment_revision),
        str(message.reference_digest),int(message.reference_intent),int(message.state),
        bool(message.reference_accepted),stamp_ns(message.header),str(message.reason),
        float(message.linear_speed_mps),float(message.angular_speed_radps))
