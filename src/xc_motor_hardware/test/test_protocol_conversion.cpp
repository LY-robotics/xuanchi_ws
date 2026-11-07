#include "xc_motor_hardware/motor_protocol.hpp"
#include "xc_motor_hardware/joint_conversion.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace xc_motor_hardware;
int main() {
  const auto c2 = MotorProtocol::make_c2_position_command(34, 16384);
  assert(c2.id == 0x122 && c2.dlc == 5 && c2.data[0] == 0xC2 && c2.data[1] == 0x00 && c2.data[2] == 0x40 && c2.data[3] == 0x00 && c2.data[4] == 0x00);
  CanFrame a3{}; a3.id=34; a3.dlc=7; a3.data[0]=0xA3; a3.data[1]=0x34; a3.data[2]=0x12; a3.data[3]=0x00; a3.data[4]=0xC0; a3.data[5]=0xff; a3.data[6]=0xff;
  auto p=MotorProtocol::parse_a3(a3,34); assert(p && p->single_turn_count==0x1234 && p->multi_turn_count==-16384);
  CanFrame a2{}; a2.id=34; a2.dlc=5; a2.data[0]=0xA2; a2.data[1]=0x10; a2.data[2]=0x27; a2.data[3]=0; a2.data[4]=0; auto v=MotorProtocol::parse_a2(a2,34); assert(v && v->speed_centi_rpm==10000);
  MotorConfig cfg; cfg.joint_name="Joint1"; cfg.motor_id=34; cfg.gear_ratio=3.0; cfg.direction=1; std::int32_t count=0; assert(JointConversion::joint_to_count(JointConversion::kTwoPi,cfg,16384,&count)); assert(count==49152); double q=0; assert(JointConversion::count_to_joint(count,cfg,16384,&q)); assert(std::abs(q-JointConversion::kTwoPi)<1e-12);
  cfg.gear_ratio=4.2; assert(JointConversion::joint_to_count(JointConversion::kTwoPi,cfg,16384,&count)); assert(count==68813); cfg.direction=-1; assert(JointConversion::joint_to_count(0.0,cfg,16384,&count)); assert(count==0);
  cfg.direction=1; cfg.joint_offset_rad=0.25; cfg.encoder_zero_count=100; assert(JointConversion::joint_to_count(0.25,cfg,16384,&count)); assert(count==100); double invalid_q=0.0; cfg.gear_ratio=0.0; assert(!JointConversion::count_to_joint(0,cfg,16384,&invalid_q)); cfg.gear_ratio=4.2;
  CanFrame a4{}; a4.id=34; a4.dlc=8; a4.data[0]=0xA4; a4.data[1]=25; a4.data[2]=0x34; a4.data[3]=0x12; a4.data[4]=0x78; a4.data[5]=0x56; a4.data[6]=0x34; a4.data[7]=0x12; auto s=MotorProtocol::parse_a4(a4,34); assert(s && s->temperature_c==25 && s->q_axis_current==0x1234 && s->single_turn_count==0x1234);
  CanFrame ae{}; ae.id=34; ae.dlc=8; ae.data[0]=0xAE; ae.data[1]=0x10; ae.data[2]=0x27; ae.data[3]=0x20; ae.data[4]=0x03; ae.data[5]=30; ae.data[6]=2; ae.data[7]=0x41; auto f=MotorProtocol::parse_ae(ae,34); assert(f && f->bus_voltage==10000 && f->temperature_c==30 && f->fault_code==0x41);
  CanFrame b0{}; b0.id=34; b0.dlc=7; b0.data[0]=0xB0; b0.data[1]=7; b0.data[2]=0x00; b0.data[3]=0x00; b0.data[4]=0x80; b0.data[5]=0x3F; b0.data[6]=3; auto b=MotorProtocol::parse_b0(b0,34); assert(b && b->pole_pairs==7 && std::abs(b->torque_constant-1.0F)<1e-6F && b->protocol_gear_ratio==3);
  assert(!MotorProtocol::valid_motor_id(0)); assert(!MotorProtocol::valid_motor_id(255)); assert(MotorProtocol::make_request(0,0xA3).dlc==0); CanFrame extended=a3; extended.extended=true; assert(!MotorProtocol::parse_a3(extended,34));
  auto faults=MotorProtocol::decode_faults(0xC5); assert(faults.size()==3);
  CanFrame c2_feedback=a3; c2_feedback.data[0]=0xC2; auto c2p=MotorProtocol::parse_position_feedback(c2_feedback,34); assert(c2p && c2p->multi_turn_count==-16384);
  CanFrame bad=a3; bad.dlc=6; assert(!MotorProtocol::parse_a3(bad,34)); std::cout << "protocol/conversion tests passed\n"; return 0;
}
