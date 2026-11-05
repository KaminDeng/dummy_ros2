"""DPRAW02配对的真实Source与当前Root语义访问；线格式只由中央SchemaCodec读取。"""

def validate_source(source):
    if not isinstance(source,dict):raise ValueError('missing actual capture Source')
    device=source.get('device');boot=source.get('boot_id');context=source.get('context_id')
    if (not isinstance(device,str) or len(device)!=32 or type(boot) is not int or not 0<boot<1<<64 or
        type(context) is not int or not 0<context<1<<32):raise ValueError('invalid capture Source')
    try:raw=bytes.fromhex(device)
    except ValueError as error:raise ValueError('invalid Source.device') from error
    if len(raw)!=16 or not any(raw):raise ValueError('zero Source.device')
    return {'device':raw.hex(),'boot_id':boot,'context_id':context}


def root_boot(value):
    # boot_epoch只在独立分析JSON/context中作为既有标签；当前Root没有这个业务字段。
    if 'source' in value:return validate_source(value['source'])['boot_id']
    return value['boot_epoch']


def root_field(fields,key):
    return root_boot(fields) if key=='boot_epoch' else fields[key]


def joints(fields):
    rows=fields.get('joints')
    if not isinstance(rows,list) or len(rows)!=6:raise ValueError('six current Root joint records required')
    ordered=[None]*6
    for row in rows:
        ident=row.get('joint_id') if isinstance(row,dict) else None
        if type(ident) is not int or not 0<=ident<6 or ordered[ident] is not None:raise ValueError('duplicate/missing Root joint_id')
        ordered[ident]=row
    return ordered


def joint_values(fields,name):
    result=[row.get(name) for row in joints(fields)]
    if any(value is None for value in result):raise ValueError('missing joint presence: '+name)
    return result


def quality_mask(fields,name):
    if name=='configured_quality' and 'joints' not in fields:
        value=fields.get('configured_mask') # 当前DeviceStatus本来就是该标量，State才是逐关节quality。
        if type(value) is not int:raise ValueError('missing Status configured_mask')
        return value
    values=joint_values(fields,name)
    if any(type(value) is not bool for value in values):raise ValueError('invalid joint quality')
    return sum((1<<i) for i,value in enumerate(values) if value)


def capture_sources(records):
    remote={};requesters={}
    for record in records:
        source=validate_source(record['source']);key=(source['device'],source['boot_id'],source['context_id'])
        (requesters if record['kind']==4 else remote)[key]=source
    if len(remote)!=1 or len(requesters)!=1:raise ValueError('ambiguous capture Source identity: requires one producer Source and one requester Source')
    return next(iter(remote.values())),next(iter(requesters.values()))


def target_boot(record):
    """关联到目标Boot的标签；不改写请求容器里属于Host的Source/boot。"""
    return record['fields']['expected_boot_epoch'] if record.get('kind')==4 else record['boot']
