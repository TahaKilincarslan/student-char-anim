#include "StudentCharAnimPlugin.h"
#include <model/AnimationModel.h>
#include <model/IModel.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>

namespace arkheon::student::charanimstudent {
namespace {

bool hasJoint(const std::unordered_set<std::string>& available, const char* id) {
    if (!id || *id == '\0') return false;
    if (available.empty()) return true;
    return available.find(id) != available.end();
}

class StudentAnimationModel final
    : public arkheon::astsim::IModel
    , public arkheon::astsim::IAnimationModel
{
public:
    [[nodiscard]] std::string getTypeName() const override {
        return "animationModelStudent";
    }

    [[nodiscard]] bool evaluate(
        const arkheon::astsim::AnimationModelInput& input,
        arkheon::astsim::AnimationModelOutput& output) override
    {
        const double t = input.simulationTimeSeconds;

        std::unordered_set<std::string> available;
        available.reserve(input.entity.joints.size());
        for (const auto& j : input.entity.joints)
            available.insert(j.jointId);

        output.clearExistingJointOverrides = false;
        output.jointOverrides.clear();

        // Çok yavaş sinüs — tam bir salınım ~10 saniye sürer
        // sin(2*pi * t / period) = sin(t * 2*pi/10) = sin(t * 0.628)
        const double period        = 60.0; // saniye cinsinden bir tam salınım
        const double phase         = t * (2.0 * 3.14159265358979 / period);
        const double shoulderSwing = std::sin(phase) * 10.0; // ±10 derece, çok küçük
        const double elbowBend     = 10.0; // sabit hafif bükülme

        if (hasJoint(available, "upperarm_l"))
            output.jointOverrides.push_back({"upperarm_l", -shoulderSwing, 0.0, -5.0});
        if (hasJoint(available, "upperarm_r"))
            output.jointOverrides.push_back({"upperarm_r",  shoulderSwing, 0.0,  5.0});
        if (hasJoint(available, "rp_nathan_animated_003_walking_lowerarm_l"))
            output.jointOverrides.push_back({"rp_nathan_animated_003_walking_lowerarm_l", elbowBend, 0.0, 0.0});
        if (hasJoint(available, "rp_nathan_animated_003_walking_lowerarm_r"))
            output.jointOverrides.push_back({"rp_nathan_animated_003_walking_lowerarm_r", elbowBend, 0.0, 0.0});

        return !output.jointOverrides.empty();
    }
};

} // namespace

int StudentCharAnimPlugin::getInterfaceVersion() const { return 1; }

arkheon::astlib::PluginMetadata StudentCharAnimPlugin::getMetadata() const {
    arkheon::astlib::PluginMetadata meta;
    meta.setPluginId("student-char-anim");
    meta.setVersion("1.0.0");
    meta.setAuthor("Student");
    return meta;
}

void StudentCharAnimPlugin::initialize(arkheon::astlib::PluginContext& context) {
    initialized_ = true; shutdown_ = false; modelOverrideRegistered_ = false;
    pluginId_ = context.metadata.pluginId();
    if (pluginId_.empty()) pluginId_ = "student-char-anim";
    modelType_ = "animationModelStudent";
    modelPluginService_ = nullptr;
    if (context.services) {
        auto* svc = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
        modelPluginService_ = static_cast<arkheon::astsim::IModelPluginService*>(svc);
    }
    if (!modelPluginService_) return;
    modelOverrideRegistered_ = modelPluginService_->setModelOverrideFactory(
        pluginId_, modelType_, []() { return std::make_unique<StudentAnimationModel>(); });
}

void StudentCharAnimPlugin::tick(double dt) { static_cast<void>(dt); }

void StudentCharAnimPlugin::shutdown() {
    if (modelPluginService_) {
        if (modelOverrideRegistered_)
            static_cast<void>(modelPluginService_->clearModelOverrideFactory(pluginId_, modelType_));
        static_cast<void>(modelPluginService_->releasePluginModels(pluginId_));
    }
    modelOverrideRegistered_ = false; shutdown_ = true; modelPluginService_ = nullptr;
}

} // namespace arkheon::student::charanimstudent

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::student::charanimstudent::StudentCharAnimPlugin();
}
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) { delete plugin; }
ARKHEON_ASTLIB_API const char* get_plugin_signature() { return "ARKHEON_PLUGIN_V1"; }
}


