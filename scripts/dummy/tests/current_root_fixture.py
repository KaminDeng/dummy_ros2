"""业务夹具用真实反射/中央codec构造当前Root，不定义第二份wire布局。"""
from scripts.dummy import selected_protocol as protocol
import hashlib
PRODUCER_SOURCE={'device':'ab'*16,'boot_id':2,'context_id':37}
REQUESTER_SOURCE={'device':'cd'*16,'boot_id':0x5678,'context_id':91}

def default(owner,field):
    codec=protocol._reader();kind=field['type'];base=kind['base_type']
    if base=='Bool':return False
    if base in ('Float','Double'):return 0.0
    if base=='String':return ''
    if base=='Obj':return defaults(kind['reference'])
    if base=='Vector':
        count=codec._capacity(owner,field)
        if kind['element']=='UByte':return bytes(count)
        if kind['element']=='Obj':
            rows=[defaults(kind['reference']) for _ in range(count)]
            for i,row in enumerate(rows):
                if 'joint_id' in row:row['joint_id']=i
            return rows
        return [0]*count
    return 0

def defaults(owner):
    return {field['name']:default(owner,field) for field in protocol._reader().fields(owner)}

def wire(name,**values):
    values.pop('source',None)  # 容器Source单独绑定，不进入DPC1业务字段。
    values.pop('boot_epoch',None);values.pop('schema_major',None);values.pop('schema_minor',None)
    if 'device_id' in values:values['domain_device_id']=values.pop('device_id')
    owner=protocol._type(name);result=defaults(owner)
    if name=='DummyJointState':
        rows=values.get('joints',result['joints'])
        for key in ('position_rad','estimated_velocity_rad_s','raw_motor_position_rev','feedback_seq','can_rx_age_us','hold_position_rad'):
            if key in values:
                for i,value in enumerate(values.pop(key)):rows[i][key]=value
        for key,quality in [('configured_mask','configured_quality'),('position_valid_mask','position_valid_quality'),('velocity_valid_mask','velocity_valid_quality'),('raw_position_valid_mask','raw_position_valid_quality'),('hold_valid_mask','hold_valid_quality')]:
            if key in values:
                mask=values.pop(key)
                for i,row in enumerate(rows):row[quality]=bool(mask&(1<<i))
        values['joints']=rows
    result.update(values)
    # 业务夹具中的0/1转为真实Bool，向量hex由schema判定，绝不推测偏移。
    for field in protocol._reader().fields(owner):
        key=field['name'];kind=field['type']
        if result.get(key) is None:continue
        if kind['base_type']=='Bool':result[key]=bool(result[key])
        if kind['base_type']=='Vector' and kind['element']=='UByte' and isinstance(result[key],str):result[key]=bytes.fromhex(result[key])
    return protocol.encode(name,result)

def replace(name,raw,**values):
    fields=protocol._reader().decode_record(protocol._type(name),raw);fields.update(values)
    return wire(name,**fields)

def chain(raw,previous):
    raw=replace('DummyAttemptEvidence',raw,previous_chain_sha256=bytes(previous),chain_sha256=bytes(32))
    digest=hashlib.sha256(bytes(previous)+protocol.evidence_parts(raw)['canonical']).digest()
    return replace('DummyAttemptEvidence',raw,chain_sha256=digest)

def chain_hash(raw):return protocol.evidence_parts(raw)['chain']
def previous_hash(raw):return protocol.evidence_parts(raw)['previous']
def canonical(raw):return protocol.evidence_parts(raw)['canonical']


def corrupt_chain(raw):
    digest=bytearray(chain_hash(raw));digest[0]^=1
    return replace('DummyAttemptEvidence',raw,chain_sha256=bytes(digest))
