#pragma once

#include <plugin/IModelPluginService.h>
#include <plugin/IPlugin.h>

#include <string>

namespace arkheon::student::charanimstudent {

class StudentCharAnimPlugin final : public arkheon::astlib::IPlugin {
public:
    StudentCharAnimPlugin() = default;
    ~StudentCharAnimPlugin() override = default;

    [[nodiscard]] int getInterfaceVersion() const override;
    [[nodiscard]] arkheon::astlib::PluginMetadata getMetadata() const override;

    void initialize(arkheon::astlib::PluginContext& context) override;
    void tick(double dt) override;
    void shutdown() override;

private:
    bool initialized_             = false;
    bool shutdown_                = false;
    bool modelOverrideRegistered_ = false;
    std::string pluginId_         = "student-char-anim";
    std::string modelType_        = "animationModelStudent";
    arkheon::astsim::IModelPluginService* modelPluginService_ = nullptr;
};

} // namespace arkheon::student::charanimstudent

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin();
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin);
ARKHEON_ASTLIB_API const char* get_plugin_signature();
}
