# -*- coding: utf-8 -*-
#
# TARGET arch is: ['-I/usr/include/clang/5.0/include', '-Iinclude']
# WORD_SIZE is: 8
# POINTER_SIZE is: 8
# LONGDOUBLE_SIZE is: 16
#
import ctypes


_libraries = {}
_libraries['./libcassie_plugin.so'] = ctypes.CDLL('./libcassie_plugin.so')
# if local wordsize is same as target, keep ctypes pointer function.
if ctypes.sizeof(ctypes.c_void_p) == 8:
    POINTER_T = ctypes.POINTER
else:
    # required to access _ctypes
    import _ctypes
    # Emulate a pointer class using the approriate c_int32/c_int64 type
    # The new class should have :
    # ['__module__', 'from_param', '_type_', '__dict__', '__weakref__', '__doc__']
    # but the class should be submitted to a unique instance for each base type
    # to that if A == B, POINTER_T(A) == POINTER_T(B)
    ctypes._pointer_t_type_cache = {}
    def POINTER_T(pointee):
        # a pointer should have the same length as LONG
        fake_ptr_base_type = ctypes.c_uint64 
        # specific case for c_void_p
        if pointee is None: # VOID pointer type. c_void_p.
            pointee = type(None) # ctypes.c_void_p # ctypes.c_ulong
            clsname = 'c_void'
        else:
            clsname = pointee.__name__
        if clsname in ctypes._pointer_t_type_cache:
            return ctypes._pointer_t_type_cache[clsname]
        # make template
        class _T(_ctypes._SimpleCData,):
            _type_ = 'L'
            _subtype_ = pointee
            def _sub_addr_(self):
                return self.value
            def __repr__(self):
                return '%s(%d)'%(clsname, self.value)
            def contents(self):
                raise TypeError('This is not a ctypes pointer.')
            def __init__(self, **args):
                raise TypeError('This is not a ctypes pointer. It is not instanciable.')
        _class = type('LP_%d_%s'%(8, clsname), (_T,),{}) 
        ctypes._pointer_t_type_cache[clsname] = _class
        return _class

c_int128 = ctypes.c_ubyte*16
c_uint128 = c_int128
void = None
if ctypes.sizeof(ctypes.c_longdouble) == 16:
    c_long_double_t = ctypes.c_longdouble
else:
    c_long_double_t = ctypes.c_ubyte*16



class struct_CassieCoreSim(ctypes.Structure):
    pass

cassie_core_sim_t = struct_CassieCoreSim
cassie_core_sim_alloc = _libraries['./libcassie_plugin.so'].cassie_core_sim_alloc
cassie_core_sim_alloc.restype = POINTER_T(struct_CassieCoreSim)
cassie_core_sim_alloc.argtypes = []
cassie_core_sim_copy = _libraries['./libcassie_plugin.so'].cassie_core_sim_copy
cassie_core_sim_copy.restype = None
cassie_core_sim_copy.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_free = _libraries['./libcassie_plugin.so'].cassie_core_sim_free
cassie_core_sim_free.restype = None
cassie_core_sim_free.argtypes = [POINTER_T(struct_CassieCoreSim)]
cassie_core_sim_setup = _libraries['./libcassie_plugin.so'].cassie_core_sim_setup
cassie_core_sim_setup.restype = None
cassie_core_sim_setup.argtypes = [POINTER_T(struct_CassieCoreSim)]
class struct_c__SA_cassie_user_in_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_out_t(ctypes.Structure):
    pass

class struct_c__SA_cassie_in_t(ctypes.Structure):
    pass

cassie_core_sim_step = _libraries['./libcassie_plugin.so'].cassie_core_sim_step
cassie_core_sim_step.restype = None
cassie_core_sim_step.argtypes = [POINTER_T(struct_CassieCoreSim), POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(struct_c__SA_cassie_in_t)]
class struct_c__SA_elmo_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('controlWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('torque', ctypes.c_double),
     ]

elmo_in_t = struct_c__SA_elmo_in_t
class struct_c__SA_cassie_leg_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_in_t),
    ('hipYawDrive', elmo_in_t),
    ('hipPitchDrive', elmo_in_t),
    ('kneeDrive', elmo_in_t),
    ('footDrive', elmo_in_t),
     ]

cassie_leg_in_t = struct_c__SA_cassie_leg_in_t
class struct_c__SA_radio_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('channel', ctypes.c_int16 * 14),
     ]

radio_in_t = struct_c__SA_radio_in_t
class struct_c__SA_cassie_pelvis_in_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radio', radio_in_t),
    ('sto', ctypes.c_bool),
    ('piezoState', ctypes.c_bool),
    ('piezoTone', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
     ]

cassie_pelvis_in_t = struct_c__SA_cassie_pelvis_in_t
struct_c__SA_cassie_in_t._pack_ = True # source:False
struct_c__SA_cassie_in_t._fields_ = [
    ('pelvis', cassie_pelvis_in_t),
    ('leftLeg', cassie_leg_in_t),
    ('rightLeg', cassie_leg_in_t),
]

cassie_in_t = struct_c__SA_cassie_in_t
DiagnosticCodes = ctypes.c_int16
class struct_c__SA_battery_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 7),
    ('stateOfCharge', ctypes.c_double),
    ('voltage', ctypes.c_double * 12),
    ('current', ctypes.c_double),
    ('temperature', ctypes.c_double * 4),
     ]

battery_out_t = struct_c__SA_battery_out_t
class struct_c__SA_cassie_joint_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
     ]

cassie_joint_out_t = struct_c__SA_cassie_joint_out_t
class struct_c__SA_elmo_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('statusWord', ctypes.c_uint16),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('position', ctypes.c_double),
    ('velocity', ctypes.c_double),
    ('torque', ctypes.c_double),
    ('driveTemperature', ctypes.c_double),
    ('dcLinkVoltage', ctypes.c_double),
    ('torqueLimit', ctypes.c_double),
    ('gearRatio', ctypes.c_double),
     ]

elmo_out_t = struct_c__SA_elmo_out_t
class struct_c__SA_cassie_leg_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('hipRollDrive', elmo_out_t),
    ('hipYawDrive', elmo_out_t),
    ('hipPitchDrive', elmo_out_t),
    ('kneeDrive', elmo_out_t),
    ('footDrive', elmo_out_t),
    ('shinJoint', cassie_joint_out_t),
    ('tarsusJoint', cassie_joint_out_t),
    ('footJoint', cassie_joint_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('reedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte * 3),
     ]

cassie_leg_out_t = struct_c__SA_cassie_leg_out_t
class struct_c__SA_radio_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('radioReceiverSignalGood', ctypes.c_bool),
    ('receiverMedullaSignalGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte * 6),
    ('channel', ctypes.c_double * 16),
     ]

radio_out_t = struct_c__SA_radio_out_t
class struct_c__SA_target_pc_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('etherCatStatus', ctypes.c_int32 * 6),
    ('etherCatNotifications', ctypes.c_int32 * 21),
    ('PADDING_0', ctypes.c_ubyte * 4),
    ('taskExecutionTime', ctypes.c_double),
    ('overloadCounter', ctypes.c_uint32),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('cpuTemperature', ctypes.c_double),
     ]

target_pc_out_t = struct_c__SA_target_pc_out_t
class struct_c__SA_vectornav_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('dataGood', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('vpeStatus', ctypes.c_uint16),
    ('PADDING_1', ctypes.c_ubyte * 4),
    ('pressure', ctypes.c_double),
    ('temperature', ctypes.c_double),
    ('magneticField', ctypes.c_double * 3),
    ('angularVelocity', ctypes.c_double * 3),
    ('linearAcceleration', ctypes.c_double * 3),
    ('orientation', ctypes.c_double * 4),
     ]

vectornav_out_t = struct_c__SA_vectornav_out_t
class struct_c__SA_cassie_pelvis_out_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('targetPc', target_pc_out_t),
    ('battery', battery_out_t),
    ('radio', radio_out_t),
    ('vectorNav', vectornav_out_t),
    ('medullaCounter', ctypes.c_ubyte),
    ('PADDING_0', ctypes.c_ubyte),
    ('medullaCpuLoad', ctypes.c_uint16),
    ('bleederState', ctypes.c_bool),
    ('leftReedSwitchState', ctypes.c_bool),
    ('rightReedSwitchState', ctypes.c_bool),
    ('PADDING_1', ctypes.c_ubyte),
    ('vtmTemperature', ctypes.c_double),
     ]

cassie_pelvis_out_t = struct_c__SA_cassie_pelvis_out_t
struct_c__SA_cassie_out_t._pack_ = True # source:False
struct_c__SA_cassie_out_t._fields_ = [
    ('pelvis', cassie_pelvis_out_t),
    ('leftLeg', cassie_leg_out_t),
    ('rightLeg', cassie_leg_out_t),
    ('isCalibrated', ctypes.c_bool),
    ('PADDING_0', ctypes.c_ubyte),
    ('messages', ctypes.c_int16 * 4),
    ('PADDING_1', ctypes.c_ubyte * 6),
]

cassie_out_t = struct_c__SA_cassie_out_t
pack_cassie_out_t = _libraries['./libcassie_plugin.so'].pack_cassie_out_t
pack_cassie_out_t.restype = None
pack_cassie_out_t.argtypes = [POINTER_T(struct_c__SA_cassie_out_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_out_t = _libraries['./libcassie_plugin.so'].unpack_cassie_out_t
unpack_cassie_out_t.restype = None
unpack_cassie_out_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_out_t)]
struct_c__SA_cassie_user_in_t._pack_ = True # source:False
struct_c__SA_cassie_user_in_t._fields_ = [
    ('torque', ctypes.c_double * 10),
    ('telemetry', ctypes.c_int16 * 9),
    ('PADDING_0', ctypes.c_ubyte * 6),
]

cassie_user_in_t = struct_c__SA_cassie_user_in_t
pack_cassie_user_in_t = _libraries['./libcassie_plugin.so'].pack_cassie_user_in_t
pack_cassie_user_in_t.restype = None
pack_cassie_user_in_t.argtypes = [POINTER_T(struct_c__SA_cassie_user_in_t), POINTER_T(ctypes.c_ubyte)]
unpack_cassie_user_in_t = _libraries['./libcassie_plugin.so'].unpack_cassie_user_in_t
unpack_cassie_user_in_t.restype = None
unpack_cassie_user_in_t.argtypes = [POINTER_T(ctypes.c_ubyte), POINTER_T(struct_c__SA_cassie_user_in_t)]
class struct_c__SA_packet_header_info_t(ctypes.Structure):
    _pack_ = True # source:False
    _fields_ = [
    ('seq_num_out', ctypes.c_char),
    ('seq_num_in_last', ctypes.c_char),
    ('delay', ctypes.c_char),
    ('seq_num_in_diff', ctypes.c_char),
     ]

packet_header_info_t = struct_c__SA_packet_header_info_t
process_packet_header = _libraries['./libcassie_plugin.so'].process_packet_header
process_packet_header.restype = None
process_packet_header.argtypes = [POINTER_T(struct_c__SA_packet_header_info_t), POINTER_T(ctypes.c_ubyte), POINTER_T(ctypes.c_ubyte)]
udp_init_host = _libraries['./libcassie_plugin.so'].udp_init_host
udp_init_host.restype = ctypes.c_int32
udp_init_host.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_init_client = _libraries['./libcassie_plugin.so'].udp_init_client
udp_init_client.restype = ctypes.c_int32
udp_init_client.argtypes = [POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char), POINTER_T(ctypes.c_char)]
udp_close = _libraries['./libcassie_plugin.so'].udp_close
udp_close.restype = None
udp_close.argtypes = [ctypes.c_int32]
ssize_t = ctypes.c_int64
size_t = ctypes.c_uint64
class struct_sockaddr(ctypes.Structure):
    pass

get_newest_packet = _libraries['./libcassie_plugin.so'].get_newest_packet
get_newest_packet.restype = ssize_t
get_newest_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
wait_for_packet = _libraries['./libcassie_plugin.so'].wait_for_packet
wait_for_packet.restype = ssize_t
wait_for_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), POINTER_T(ctypes.c_uint32)]
socklen_t = ctypes.c_uint32
send_packet = _libraries['./libcassie_plugin.so'].send_packet
send_packet.restype = ssize_t
send_packet.argtypes = [ctypes.c_int32, POINTER_T(None), size_t, POINTER_T(struct_sockaddr), socklen_t]
struct_sockaddr._pack_ = True # source:False
struct_sockaddr._fields_ = [
    ('sa_family', ctypes.c_uint16),
    ('sa_data', ctypes.c_char * 14),
]

__all__ = \
    ['DiagnosticCodes', 'battery_out_t', 'cassie_core_sim_alloc',
    'cassie_core_sim_copy', 'cassie_core_sim_free',
    'cassie_core_sim_setup', 'cassie_core_sim_step',
    'cassie_core_sim_t', 'cassie_in_t', 'cassie_joint_out_t',
    'cassie_leg_in_t', 'cassie_leg_out_t', 'cassie_out_t',
    'cassie_pelvis_in_t', 'cassie_pelvis_out_t', 'cassie_user_in_t',
    'elmo_in_t', 'elmo_out_t', 'get_newest_packet',
    'pack_cassie_out_t', 'pack_cassie_user_in_t',
    'packet_header_info_t', 'process_packet_header', 'radio_in_t',
    'radio_out_t', 'send_packet', 'size_t', 'socklen_t', 'ssize_t',
    'struct_CassieCoreSim', 'struct_c__SA_battery_out_t',
    'struct_c__SA_cassie_in_t', 'struct_c__SA_cassie_joint_out_t',
    'struct_c__SA_cassie_leg_in_t', 'struct_c__SA_cassie_leg_out_t',
    'struct_c__SA_cassie_out_t', 'struct_c__SA_cassie_pelvis_in_t',
    'struct_c__SA_cassie_pelvis_out_t',
    'struct_c__SA_cassie_user_in_t', 'struct_c__SA_elmo_in_t',
    'struct_c__SA_elmo_out_t', 'struct_c__SA_packet_header_info_t',
    'struct_c__SA_radio_in_t', 'struct_c__SA_radio_out_t',
    'struct_c__SA_target_pc_out_t', 'struct_c__SA_vectornav_out_t',
    'struct_sockaddr', 'target_pc_out_t', 'udp_close',
    'udp_init_client', 'udp_init_host', 'unpack_cassie_out_t',
    'unpack_cassie_user_in_t', 'vectornav_out_t', 'wait_for_packet']
