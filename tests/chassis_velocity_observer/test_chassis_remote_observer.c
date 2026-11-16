#include "chassis_velocity_observer.h"
#include "chassis_imu_can.h"
#include "gimbal.h"
#include "ins_task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick = 1000;
static bool online[4] = {true, true, true, true};
static DJIMotorInstance motors[4];
DJIMotorInstance *motor_lf=&motors[0], *motor_rf=&motors[1];
DJIMotorInstance *motor_lb=&motors[2], *motor_rb=&motors[3];
static ChassisImuCanSnapshot_s remote;
static bool remote_usable;
static unsigned local_imu_reads;
static unsigned captured;
static ChassisFusedVelocitySnapshot_s captured_snapshot;
static const ChassisVelocityFeedback_s target_feedback = {
    .target = {.valid = true, .vx_mps = .5f, .wz_rpm = 30.f},
};
const ChassisVelocityFeedback_s *const volatile chassis_velocity = &target_feedback;

void VofaKalmanCapture(const ChassisFusedVelocitySnapshot_s *snapshot,
    const ChassisImuCanSnapshot_s *imu, bool usable,
    const ChassisVelocity_s *target, uint32_t now_ms)
{
    assert(now_ms == tick && snapshot->diagnostics.tick_ms == now_ms);
    assert(imu->sample.sequence == remote.sample.sequence && usable == remote_usable);
    assert(target->vx_mps == .5f && target->wz_rpm == 30.f);
    captured_snapshot = *snapshot;
    ++captured;
}

uint32_t HAL_GetTick(void) { return tick; }
bool DaemonIsOnline(const bool *value) { return *value; }
/* The remote observer must work without a local IMU or gimbal measurement. */
bool INSGetMotionSnapshot(INSMotionSnapshot_s *out)
{ ++local_imu_reads; memset(out,0,sizeof *out); return false; }
bool GimbalGetEncoderSnapshot(GimbalEncoderSample_s out[2])
{ memset(out,0,2*sizeof *out); return false; }
bool ChassisImuCanReadSnapshot(ChassisImuCanSnapshot_s *out,uint32_t now_ms)
{ assert(now_ms==tick); *out=remote; return remote_usable; }

static const ChassisVelocityConfig_s config={
    .omni_x=true,.wheel_radius_m=.06f,.reduction_ratio=19,
    .wheel_base_m=.35f,.track_width_m=.30f,.direction={1,1,1,1},
};

static ChassisFusedVelocitySnapshot_s Step(bool new_remote)
{
    const unsigned before_capture = captured;
    for(unsigned w=0;w<4;++w) {
        motors[w].daemon=&online[w];
        motors[w].measure.raw_rpm=0;
        motors[w].measure.rx_tick_ms=tick;
        ++motors[w].measure.rx_sequence;
    }
    if(new_remote) {
        remote.available=true;
        remote.sample.valid=true;
        ++remote.sample.sequence;
        remote.sample.tick_ms=900000+tick;
        remote.sample.source_age_ms=2;
        /* Native A-board axes aligned with nav; public X has the same sign. */
        remote.sample.quaternion[0]=1;
        remote.sample.quaternion[3]=0;
        remote.sample.accel_n_mps2[0]=1;
        remote.first_rx_tick_ms=tick-1;
        remote.last_rx_tick_ms=tick;
        remote_usable=true;
    }
    ChassisObserveFusedVelocity(&config);
    assert(captured == before_capture + 1);
    ChassisFusedVelocitySnapshot_s out;
    (void)ChassisGetFusedVelocitySnapshot(&out);
    assert(captured_snapshot.remote.attempt.sequence == out.remote.attempt.sequence);
    assert(captured_snapshot.status == out.status);
    return out;
}

static ChassisFusedVelocitySnapshot_s Run(unsigned count)
{
    ChassisFusedVelocitySnapshot_s out={0};
    for(unsigned i=0;i<count;++i) {
        tick+=5;
        out=Step(tick%20==0);
    }
    return out;
}

int main(void)
{
    ChassisFusedVelocitySnapshot_s out=Run(40);
    assert(out.status==CHASSIS_VELOCITY_FUSED && out.valid);
    /* Zero wheel observations plus measured acceleration must actually enter
       the KF, rather than merely labelling a copied wheel value as fused. */
    assert(out.vx_mps>.0001f && out.vx_mps<.5f);
    assert(fabsf(out.vy_mps)<.00001f);
    assert(out.paired_wheel_mps[0]==0 && out.paired_wheel_mps[1]==0);
    assert(fabsf(out.center_accel_n_mps2[0]-1)<.00001f);
    assert(out.tick_ms==tick-23);
    assert(local_imu_reads==0);
    assert(out.diagnostics.imu_valid && out.diagnostics.imu_sequence==remote.sample.sequence);
    assert(out.diagnostics.imu_accel_n_mps2[0]==1);
    assert(out.remote.attempt.available && out.remote.attempt.output.fused);
    assert(out.remote.attempt.input.accel_n_mps2[0]==1);
    assert(out.remote.attempt.middle_receiver_ms==out.tick_ms);
    assert(out.remote.attempt.receiver_call_ms==tick);
    const uint32_t attempt_sequence=out.remote.attempt.sequence;
    const uint32_t fused_tick=out.tick_ms;
    const float fused_vx=out.vx_mps;
    tick+=5;
    out=Step(false);
    assert(out.status==CHASSIS_VELOCITY_FUSED && out.tick_ms==fused_tick);
    assert(out.vx_mps==fused_vx);
    assert(out.remote.attempt.sequence==attempt_sequence);

    remote_usable=false;
    tick+=5;
    out=Step(false);
    assert(out.status==CHASSIS_VELOCITY_IMU_UNAVAILABLE && out.valid);
    assert(out.vx_mps==0 && out.vy_mps==0);
    assert(!out.diagnostics.imu_valid && local_imu_reads==0);
    assert(out.remote.attempt.available && out.remote.attempt.sequence==attempt_sequence);
    assert(out.remote.attempt.output.fused && out.remote.attempt.middle_receiver_ms==fused_tick);
    out=Run(40);
    assert(out.status==CHASSIS_VELOCITY_FUSED && out.valid);
    online[2]=false;
    tick+=5;
    out=Step(false);
    assert(out.status==CHASSIS_VELOCITY_WHEEL_UNAVAILABLE && !out.valid);
    online[2]=true;
    out=Run(40);
    assert(out.status==CHASSIS_VELOCITY_FUSED && out.valid);
    tick+=21;
    assert(!ChassisGetFusedVelocitySnapshot(&out));
    assert(out.remote.attempt.available && out.remote.attempt.output.fused);
    puts("PASS: remote IMU drives observation KF without local INS/gimbal or control writes");
    return 0;
}
