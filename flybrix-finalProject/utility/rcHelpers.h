#ifndef RC_HELPERS_H
#define RC_HELPERS_H

#include <cstdint>
#include <tuple>

#include "ticker.h"

struct RcCommand final {
    enum class AUX : uint8_t {
        None = 0,
        Low = 1,
        Mid = 2,
        High = 4,
    };

    // bitfield order is {AUX1_low, AUX1_mid, AUX1_high, AUX2_low, AUX2_mid, AUX2_high, x, x} (LSB-->MSB)
    AUX aux1{AUX::None};
    AUX aux2{AUX::None};
    int16_t throttle{0};
    int16_t pitch{0};
    int16_t roll{0};
    int16_t yaw{0};

    void parseBools(bool l1, bool m1, bool h1, bool l2, bool m2, bool h2) {
        aux1 = l1 ? AUX::Low : m1 ? AUX::Mid : h1 ? AUX::High : AUX::None;
        aux2 = l2 ? AUX::Low : m2 ? AUX::Mid : h2 ? AUX::High : AUX::None;
    }

    void parseAuxMask(uint8_t auxmask) {
        parseBools(auxmask & 1, auxmask & 2, auxmask & 4, auxmask & 8, auxmask & 16, auxmask & 32);
    }

    uint8_t auxMask() const {
        return uint8_t(aux1) | (uint8_t(aux2) << 3);
    }
};

enum class RcStatus {
    Ok,
    Waiting,
    Timeout,
};

struct RcState final {
    RcStatus status;
    RcCommand command;
};

class RcFilter final {
   public:
    void update(uint8_t sources) {
        sources_ = sources;
    }

    bool accepts(uint8_t source) const {
        return sources_ & source;
    }

   private:
    uint8_t sources_{255};
};

template <typename Tsrc>
class RcTracker final {
   public:
    RcTracker() {
    }

    explicit RcTracker(Tsrc&& source) : source_(source) {
    }

    RcState query() {
        RcState state(source_.query());
        if (state.status == RcStatus::Ok) {
            tolerance_.reset(Tsrc::refresh_delay_tolerance);
            cache_ = state.command;
        } else {
            state.status = tolerance_.tick() ? RcStatus::Waiting : RcStatus::Timeout;
            state.command = cache_;
        }
        return state;
    }

    static constexpr uint8_t recovery_rate = Tsrc::recovery_rate;

    Tsrc source_;

   private:
    RcCommand cache_;
    Ticker<uint8_t> tolerance_;
};

template <typename... Tsrcs>
class RcMux final {
   public:
    RcMux() {
    }

    RcMux(Tsrcs&... sources) : sources_{sources...} {
    }

    RcState query() {
        RcState state(queryHelper());
        if (invalid_count_ <= 0) {
            invalid_count_ = 0;
            valid_ = true;
        }
        if (invalid_count_ >= 80) {
            invalid_count_ = 80;
            valid_ = false;
        }
        if (!valid_) {
            state.status = RcStatus::Timeout;
        }
        return state;
    }

    template <std::size_t N>
    inline typename std::tuple_element<N, std::tuple<Tsrcs...>>::type& source() {
        return std::get<N>(sources_).source_;
    }

    void setFilter(uint8_t filter) {
        filter_.update(filter);
    }

   private:
    template <std::size_t I = 0>
    inline typename std::enable_if<I == sizeof...(Tsrcs), RcState>::type queryHelper() {
        invalid_count_ += 1;
        return RcState{RcStatus::Timeout, {}};
    }

    template <std::size_t I = 0>
    inline typename std::enable_if<(I < sizeof...(Tsrcs)), RcState>::type queryHelper() {
        RcState state = std::get<I>(sources_).query();
        if (state.status != RcStatus::Timeout && filter_.accepts(1 << I)) {
            invalid_count_ -= std::get<I>(sources_).recovery_rate;
            return state;
        }
        return queryHelper<I + 1>();
    }

    RcFilter filter_;
    std::tuple<RcTracker<Tsrcs>...> sources_;
    bool valid_{true};
    int16_t invalid_count_{0};
};

#endif  // RC_HELPERS_H
