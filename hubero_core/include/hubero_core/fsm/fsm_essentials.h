#pragma once

#include <hubero_common/logger.h>
#include <string>

namespace hubero {

class FsmEssentials {
public:
    void setLoggingVerbosity(bool enable_verbose) {
		logging_verbose_ = enable_verbose;
	}

	void setLoggerPreamble(const std::string& text) {
		logger_preamble_ = text;
	}

protected:
    FsmEssentials(const std::string& fsm_name):
        fsm_name_(fsm_name),
        logging_verbose_(false) {}

    template <typename T>
	void logTransition(const std::string& state_src, const std::string& state_dst, const T& event) const {
        HUBERO_LOG(
            "%s[%s] transition from %s to %s\r\n",
            prepareLogPreamble().c_str(),
            fsm_name_.c_str(),
            state_src.c_str(),
            state_dst.c_str()
        );
        logTransitionConditions(event);
    }

    std::string fsm_name_;
    bool logging_verbose_;
	std::string logger_preamble_;

private:
    /**
	 * @brief Logs @ref event details if @ref logging_verbose_ was set to true
	 */
    template <typename T>
	void logTransitionConditions(const T& event) const {
		if (!logging_verbose_) {
			return;
		}

		HUBERO_LOG(
            "%s[%s] transition conditions\r\n%s\r\n",
            prepareLogPreamble().c_str(),
            fsm_name_.c_str(),
            event.toString().c_str()
        );
	}

    std::string prepareLogPreamble() const {
        std::string preamble_full("");
        if (!logger_preamble_.empty()) {
            preamble_full.append("[" + logger_preamble_ + "].");
        }
        return preamble_full;
    }
};

} // namespace hubero
