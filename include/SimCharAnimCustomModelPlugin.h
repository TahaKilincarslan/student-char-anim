// Arkheon Simulation Technologies
// Proprietary and Confidential.
// Unauthorized copying of this file, via any medium, is strictly prohibited.
// © Arkheon Simulation Technologies. All rights reserved.

#pragma once

#include <plugin/IModelPluginService.h>
#include <plugin/IPlugin.h>

#include <string>

namespace arkheon::sample::simcharanimcustommodel {

class SimCharAnimCustomModelPlugin final
    : public arkheon::astlib::IPlugin {
public:
    SimCharAnimCustomModelPlugin() = default;
    ~SimCharAnimCustomModelPlugin() override = default;

    [[nodiscard]] int getInterfaceVersion() const override;
    [[nodiscard]] arkheon::astlib::PluginMetadata getMetadata() const override;

    void initialize(arkheon::astlib::PluginContext& context) override;
    void tick(double dt) override;
    void shutdown() override;

private:
    bool initialized_ = false;
    bool shutdown_ = false;
    bool modelOverrideRegistered_ = false;
    std::string pluginId_ = "sim-char-anim-custom-model";
    std::string modelType_ = "animationModelCustom";
    arkheon::astsim::IModelPluginService* modelPluginService_ = nullptr;
};

} // namespace arkheon::sample::simcharanimcustommodel

extern "C" {
ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin();
ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin);
ARKHEON_ASTLIB_API const char* get_plugin_signature();
}
