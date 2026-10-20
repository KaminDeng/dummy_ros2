"""远端到近端保留5mm间隙；机械连接不统一膨胀，不依赖实机。"""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro

ROOT=Path(__file__).resolve().parents[3]
DESCRIPTION=ROOT/'src/dummy-ros2_description'
PROXIMAL=('base_link','link1_1_1','link2_1_1','step_link2_motor_1','step_link2_motor_2')
DISTAL=('link4_1_1','link5_1_1','step_link4_motor_1','step_link5_motor_1','j6_reducer_housing','link6_1_1')


def test_clearance_bodies_are_fixed_invisible_and_check_every_distal_part():
    model=ET.fromstring(xacro.process_file(str(DESCRIPTION/'urdf/dummy-ros2.xacro')).toxml())
    srdf=ET.parse(ROOT/'src/dummy_moveit_config/config/dummy-ros2.srdf').getroot()
    allowed={frozenset((p.get('link1'),p.get('link2'))) for p in srdf.findall('disable_collisions')}
    names={n.get('name') for n in model.findall('link')}
    for parent in PROXIMAL:
        guard='clearance_'+parent
        link=model.find(f"link[@name='{guard}']")
        assert link is not None,guard
        assert link.find('visual') is None and link.find('collision') is not None
        joint=model.find(f"joint[@name='{guard}_fixed']")
        assert joint.get('type')=='fixed' and joint.find('parent').get('link')==parent
        assert joint.find('origin').get('xyz')=='0 0 0'
        for other in names-{guard}:
            pair=frozenset((guard,other))
            assert (pair not in allowed)==(other in DISTAL),(guard,other)
    assert len([j for j in model.findall('joint') if j.get('type')=='revolute'])==6


def test_clearance_manifest_proves_expansion_and_source_binding():
    import hashlib
    manifest=json.loads((DESCRIPTION/'meshes/clearance-guards.json').read_text())
    assert manifest['minimum_clearance_mm']==5.0
    for row in manifest['meshes']:
        assert row['minimum_expansion_after_stl_roundtrip_mm']>=5.0
        assert row['parts']>0
        for field,digest in [('source',row['source_sha256']),('output',row['output_sha256'])]:
            assert hashlib.sha256((DESCRIPTION/'meshes'/row[field]).read_bytes()).hexdigest()==digest
