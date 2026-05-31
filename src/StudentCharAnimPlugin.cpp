// StudentCharAnimPlugin.cpp
// Procedural Character Animation Plugin for N8RO
// States: 1=STAND  2=RUNNING  3=JUMP  4=SLIDING
// Overrides animationModelNathanHuman

#define _CRT_SECURE_NO_WARNINGS
#include "SimCharAnimCustomModelPlugin.h"
#include <model/AnimationModel.h>
#include <model/IModel.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>
#include <cmath>
#include <memory>
#include <string>
#include <windows.h>

namespace arkheon::sample::simcharanimcustommodel {
namespace {

// ── Joint indices ─────────────────────────────────────────────────────────────
enum JointIdx {
    J_HIPS=0, J_SPINE, J_CHEST,
    J_L_UPLEG, J_R_UPLEG,
    J_L_LEG,   J_R_LEG,
    J_L_FOOT,  J_R_FOOT,
    J_L_ARM,   J_R_ARM,
    J_L_FOREARM, J_R_FOREARM,
    JOINT_COUNT
};

enum MotionState { STATE_STAND=0, STATE_RUNNING, STATE_JUMP, STATE_SLIDING };

struct JointRot { float rx=0, ry=0, rz=0; };

static float lerp(float a, float b, float t) { return a+(b-a)*t; }
static float smoothstep(float t) { return t<=0.f?0.f:(t>=1.f?1.f:t*t*(3.f-2.f*t)); }
static void zero_pose(JointRot p[JOINT_COUNT]) { for(int i=0;i<JOINT_COUNT;++i) p[i]={}; }
static void blend_poses(JointRot r[JOINT_COUNT], const JointRot a[JOINT_COUNT],
                        const JointRot b[JOINT_COUNT], float t) {
    for(int i=0;i<JOINT_COUNT;++i){
        r[i].rx=lerp(a[i].rx,b[i].rx,t);
        r[i].ry=lerp(a[i].ry,b[i].ry,t);
        r[i].rz=lerp(a[i].rz,b[i].rz,t);
    }
}

static void pose_stand(JointRot o[JOINT_COUNT], float t) {
    zero_pose(o);
    float breathe = std::sin(t*1.2f)*2.0f;
    float sway    = std::sin(t*0.7f)*2.0f;
    o[J_L_UPLEG].rz=sway*0.5f; o[J_R_UPLEG].rz=sway*0.5f;
    o[J_L_LEG].rz=-sway*0.2f;  o[J_R_LEG].rz=sway*0.2f;
    o[J_SPINE].rx=2.0f+breathe*0.5f; o[J_CHEST].rx=2.0f+breathe*0.5f;
    o[J_L_ARM].ry=85.94f+breathe*1.5f; o[J_L_ARM].rz=-11.46f;
    o[J_R_ARM].ry=85.94f+breathe*1.5f; o[J_R_ARM].rz=11.46f;
    o[J_L_FOREARM].rx=10.0f+std::sin(t*1.2f+0.5f)*2.0f;
    o[J_R_FOREARM].rx=10.0f+std::sin(t*1.2f-0.5f)*2.0f;
}

static void pose_run(JointRot o[JOINT_COUNT], float t) {
    zero_pose(o);
    float stride=std::sin(t*8.3f)*50.0f;
    float sway=std::sin(t*8.3f)*4.5f;
    o[J_L_UPLEG].rx=sway; o[J_L_UPLEG].rx=stride;
    o[J_R_UPLEG].rx=sway; o[J_R_UPLEG].rx=-stride;
    o[J_L_LEG].rx=(stride>0.f)?-std::abs(stride)*0.75f:0.f;
    o[J_R_LEG].rx=(stride<0.f)?-std::abs(stride)*0.75f:0.f;
    o[J_L_FOOT].rz=std::abs(stride)*0.25f;
    o[J_R_FOOT].rz=std::abs(stride)*0.25f;
    o[J_L_ARM].ry=86.0f-stride*0.5f; o[J_R_ARM].ry=86.0f+stride*0.5f;
    o[J_L_FOREARM].rx=30.0f; o[J_R_FOREARM].rx=30.0f;
    o[J_SPINE].rx=15.0f; o[J_CHEST].rx=10.0f;
}

static void pose_jump(JointRot o[JOINT_COUNT]) {
    zero_pose(o);
    o[J_L_UPLEG].rz=57.3f; o[J_R_UPLEG].rz=-57.3f;
    o[J_L_LEG].rz=-37.2f;  o[J_R_LEG].rz=-37.2f;
    o[J_L_ARM].rx=5.7f; o[J_L_ARM].ry=71.6f;
    o[J_R_ARM].rx=5.7f; o[J_R_ARM].ry=71.6f;
    o[J_L_FOREARM].ry=11.5f; o[J_L_FOREARM].rz=-5.7f;
    o[J_R_FOREARM].ry=11.5f; o[J_R_FOREARM].rz=-5.7f;
}

static void pose_sliding(JointRot o[JOINT_COUNT]) {
    zero_pose(o);
    o[J_L_UPLEG].rz=60.0f; o[J_R_UPLEG].rz=60.0f;
    o[J_L_LEG].rz=-80.0f;  o[J_R_LEG].rz=-80.0f;
    o[J_L_FOOT].rx=-15.0f; o[J_R_FOOT].rx=-15.0f;
    o[J_SPINE].rx=-25.0f;  o[J_CHEST].rx=-15.0f;
    o[J_L_ARM].ry=40.0f; o[J_L_ARM].rz=-30.0f;
    o[J_R_ARM].ry=40.0f; o[J_R_ARM].rz=30.0f;
    o[J_L_FOREARM].rx=20.0f; o[J_R_FOREARM].rx=20.0f;
}

static void get_pose(JointRot o[JOINT_COUNT], MotionState s, float t) {
    switch(s){
        case STATE_RUNNING: pose_run(o,t);    break;
        case STATE_JUMP:    pose_jump(o);     break;
        case STATE_SLIDING: pose_sliding(o);  break;
        default:            pose_stand(o,t);  break;
    }
}

class CustomAnimationModel final
    : public arkheon::astsim::IModel
    , public arkheon::astsim::IAnimationModel
{
    MotionState cur_   = STATE_STAND;
    double      tstart_= 0.0;
    float       blend_ = 1.0f;
    bool        trans_ = false;
    JointRot    snap_[JOINT_COUNT] = {};
    bool        held_[4] = {};
    MotionState pend_  = STATE_STAND;
    bool        hasPend_= false;

public:
    [[nodiscard]] std::string getTypeName() const override {
        return "animationModelNathanHuman";
    }

    void pollInput() {
        static const int keys[4]={0x31,0x32,0x33,0x34};
        static const MotionState st[4]={STATE_STAND,STATE_RUNNING,STATE_JUMP,STATE_SLIDING};
        for(int k=0;k<4;++k){
            bool p=(GetAsyncKeyState(keys[k])&0x8000)!=0;
            if(p&&!held_[k]&&st[k]!=cur_){pend_=st[k];hasPend_=true;}
            held_[k]=p;
        }
    }

    [[nodiscard]] bool evaluate(
        const arkheon::astsim::AnimationModelInput& input,
        arkheon::astsim::AnimationModelOutput& output) override
    {
        float dt=(float)(input.deltaTimeSeconds>0.0?input.deltaTimeSeconds:0.02);
        double t=input.simulationTimeSeconds;
        pollInput();

        if(hasPend_){
            hasPend_=false;
            get_pose(snap_,cur_,(float)(t-tstart_));
            cur_=pend_; tstart_=t; trans_=true; blend_=0.0f;
        }
        if(trans_){ blend_+=dt/0.15f; if(blend_>=1.f){blend_=1.f;trans_=false;} }

        float st=(float)(t-tstart_);
        JointRot pose[JOINT_COUNT];
        get_pose(pose,cur_,st);
        if(trans_){
            JointRot blended[JOINT_COUNT];
            blend_poses(blended,snap_,pose,smoothstep(blend_));
            for(int i=0;i<JOINT_COUNT;++i) pose[i]=blended[i];
        }

        output.clearExistingJointOverrides=true;
        auto toRad=[](float d){return(double)(d*3.14159f/180.0f);};
        auto res=[&](const std::string& ext)->std::string{
            for(const auto& j:input.entity.joints)
                if(j.externalJointName==ext) return j.jointId;
            return ext;
        };
        auto push=[&](const std::string& ext,const JointRot& r){
            arkheon::astsim::AnimationJointOverride ov;
            ov.jointId=res(ext); ov.xRad=toRad(r.rx); ov.yRad=toRad(r.ry); ov.zRad=toRad(r.rz);
            output.jointOverrides.push_back(std::move(ov));
        };
        push("Left Hip",      pose[J_L_UPLEG]);
        push("Right Hip",     pose[J_R_UPLEG]);
        push("Left Knee",     pose[J_L_LEG]);
        push("Right Knee",    pose[J_R_LEG]);
        push("Left Ankle",    pose[J_L_FOOT]);
        push("Right Ankle",   pose[J_R_FOOT]);
        push("Spine",        pose[J_SPINE]);
        push("Chest",        pose[J_CHEST]);
        push("Left Shoulder", pose[J_L_ARM]);
        push("Right Shoulder",pose[J_R_ARM]);
        push("Left Elbow",    pose[J_L_FOREARM]);
        push("Right Elbow",   pose[J_R_FOREARM]);
        return true;
    }
};

} // namespace

// ── Plugin lifecycle ──────────────────────────────────────────────────────────
int SimCharAnimCustomModelPlugin::getInterfaceVersion() const { return 1; }

arkheon::astlib::PluginMetadata SimCharAnimCustomModelPlugin::getMetadata() const {
    arkheon::astlib::PluginMetadata m;
    m.setPluginId("sim-char-anim-custom-model");
    m.setVersion("1.0.0");
    m.setAuthor("Student");
    return m;
}

void SimCharAnimCustomModelPlugin::initialize(arkheon::astlib::PluginContext& context) {
    initialized_=true; shutdown_=false; modelOverrideRegistered_=false;
    modelType_="animationModelNathanHuman";
    modelPluginService_=nullptr;
    if(context.services){
        auto* svc=static_cast<arkheon::astsim::IModelPluginService*>(
            context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId));
        modelPluginService_=svc;
    }
    if(!modelPluginService_) return;
    modelOverrideRegistered_=modelPluginService_->setModelOverrideFactory(
        pluginId_, modelType_,
        []()->std::unique_ptr<arkheon::astsim::IModel>{
            return std::make_unique<CustomAnimationModel>();
        });
}

void SimCharAnimCustomModelPlugin::tick(double dt) { static_cast<void>(dt); }

void SimCharAnimCustomModelPlugin::shutdown() {
    if(modelPluginService_){
        if(modelOverrideRegistered_)
            static_cast<void>(modelPluginService_->clearModelOverrideFactory(pluginId_,modelType_));
        static_cast<void>(modelPluginService_->releasePluginModels(pluginId_));
    }
    modelOverrideRegistered_=false; shutdown_=true; modelPluginService_=nullptr;
}

} // namespace arkheon::sample::simcharanimcustommodel

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::sample::simcharanimcustommodel::SimCharAnimCustomModelPlugin();
}
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) { delete plugin; }
ARKHEON_ASTLIB_API const char* get_plugin_signature() { return "ARKHEON_PLUGIN_V1"; }
}


