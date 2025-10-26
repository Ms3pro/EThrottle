#pragma once

#include <stdint.h>

enum struct FaultMode
{
    Nominal = 0,
    ShortTerm = 1,
    LongTerm = 2
};

struct ModeWithTransition
{
    FaultMode mode;
    bool transition;
};

class FaultFilter
{
public:
    void
    init(
        uint8_t ltFaultThresh,
        uint8_t stFaultTimeoutMax,
        uint8_t ltFaultTimeoutMax)
    {
        ltFaultThresh_ = ltFaultThresh;
        stFaultTimeoutMax_ = stFaultTimeoutMax;
        ltFaultTimeoutMax_ = ltFaultTimeoutMax;
        reset();
    }

    void
    reset()
    {
        mode_ = FaultMode::Nominal;
        timeout_ = 0;
        ltFaultCount_ = 0;
    }

    FaultMode
    mode() const
    {
        return mode_;
    }

    ModeWithTransition
    process(
        bool faulted)
    {
        FaultMode prevMode = mode_;

        if (faulted)
        {
            switch (mode_)
            {
                case FaultMode::Nominal:
                {
                    // ENTER SHORT TERM FAULT MODE
                    mode_ = FaultMode::ShortTerm;
                    timeout_ = stFaultTimeoutMax_;
                    break;
                }
                case FaultMode::ShortTerm:
                {
                    if (++ltFaultCount_ >= ltFaultThresh_)
                    {
                        // ENTER LONG TERM FAULT MODE
                        mode_ = FaultMode::LongTerm;
                        timeout_ = ltFaultTimeoutMax_;
                    }
                    else
                    {
                        timeout_ = stFaultTimeoutMax_;
                    }
                    break;
                }
                case FaultMode::LongTerm:
                {
                    // reset timeout. must see no faults within window for LT fault to clear
                    timeout_ = ltFaultTimeoutMax_;
                    break;
                }
            }
        }
        else
        {
            if (timeout_ > 0 && --timeout_ == 0)
            {
                // EXIT FAULT MODE
                ltFaultCount_ = 0;
                mode_ = FaultMode::Nominal;
            }
        }

        return {mode_, mode_ != prevMode};
    }

private:
    FaultMode mode_ = FaultMode::Nominal;
    uint8_t timeout_ = 0u;
    uint8_t ltFaultCount_ = 0u;

    uint8_t ltFaultThresh_ = 5u;
    uint8_t stFaultTimeoutMax_ = 10u;
    uint8_t ltFaultTimeoutMax_ = 100u;

};