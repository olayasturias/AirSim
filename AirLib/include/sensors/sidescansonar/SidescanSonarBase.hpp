// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SidescanSonarBase_hpp
#define msr_airlib_SidescanSonarBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr
{
namespace airlib
{

    class SidescanSonarBase : public SensorBase
    {
    public:
        SidescanSonarBase(const std::string& sensor_name = "")
            : SensorBase(sensor_name)
        {
        }

    public:
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            UpdatableObject::reportState(reporter);

            reporter.writeValue("SidescanSonar-Timestamp", output_.time_stamp);
            reporter.writeValue("SidescanSonar-NumPoints", static_cast<int>(output_.point_cloud.size() / 3));
        }

        const SidescanSonarData& getOutput() const
        {
            return output_;
        }

    protected:
        void setOutput(const SidescanSonarData& output)
        {
            output_ = output;
        }

    private:
        SidescanSonarData output_;
    };
}
} //namespace
#endif
