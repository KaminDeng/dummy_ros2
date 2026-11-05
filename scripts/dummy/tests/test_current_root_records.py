"""真实生成schema解析DPC1/DPRAW02；字段布局不在测试中另造。"""
import hashlib
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
import selected_protocol as protocol
import join_real_6dof_evidence as joiner
from current_root_records import capture_sources,target_boot

class CurrentRootCapture(unittest.TestCase):
    def header(self,kind,body,source,session,run,seq,when):
        return joiner.RECORD_HEADER.pack(kind,len(body),when,source['boot_id'],session,run,seq,
            bytes.fromhex(source['device']),source['context_id'])+body

    def test_request_source_is_host_and_target_boot_remains_business_expected(self):
        host={'device':'22'*16,'boot_id':0x456789abcdef,'context_id':9}
        producer={'device':'11'*16,'boot_id':0x123456789abc,'context_id':7}
        request=protocol.encode('DummyManageRequest',dict(expected_boot_epoch=producer['boot_id'],
            expected_session_id=8,request_seq=11,client_instance_id=bytes.fromhex('44'*16),
            request_id=1,domain_device_id=bytes.fromhex('33'*16)))
        response=protocol.encode('DummyManageResponse',dict(session_id=8,run_id=77,
            client_instance_id=bytes.fromhex('44'*16),request_id=1,domain_device_id=bytes.fromhex('33'*16)))
        raw=b'DPRAW02\0'+self.header(4,request,host,8,0,11,100)+self.header(5,response,producer,8,77,0,110)
        rows=joiner._raw_records(raw)
        self.assertEqual(joiner.RECORD_HEADER.size,59)
        self.assertEqual(rows[0]['boot'],host['boot_id'])
        self.assertEqual(target_boot(rows[0]),producer['boot_id'])
        self.assertEqual(rows[0]['source'],host)
        self.assertEqual(rows[0]['fields']['domain_device_id'],'33'*16)
        self.assertNotIn('boot_epoch',rows[0]['fields'])
        self.assertEqual(capture_sources(rows),(producer,host))

    def test_default_reader_refuses_history_and_truncation(self):
        for raw in [b'DPRAW01\0',b'DPRAW02\0'+b'\x01'*58]:
            with self.assertRaises(ValueError):joiner._raw_records(raw)

    def test_hash_uses_central_evidence_hash_purpose(self):
        fields=dict(run_id=9,evidence_seq=0,evidence_lost=False,
            previous_chain_sha256=bytes(32),chain_sha256=bytes(32))
        raw=protocol.encode('DummyAttemptEvidence',fields)
        parts=protocol.evidence_parts(raw)
        fields['chain_sha256']=hashlib.sha256(parts['previous']+parts['canonical']).digest()
        final=protocol.encode('DummyAttemptEvidence',fields)
        verified=protocol.evidence_parts(final)
        self.assertEqual(verified['canonical'],parts['canonical'])
        self.assertEqual(verified['chain'],hashlib.sha256(verified['previous']+verified['canonical']).digest())
        self.assertFalse(verified['fields']['evidence_lost'])

    def test_capture_requires_full_context_not_domain_device(self):
        from current_root_records import validate_source
        for value in [dict(device='11'*16,boot_id=1,context_id=0),
                      dict(device='00'*16,boot_id=1,context_id=1)]:
            with self.assertRaises(ValueError):validate_source(value)

if __name__=='__main__':unittest.main()
