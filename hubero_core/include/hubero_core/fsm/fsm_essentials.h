#pragma once

#include <hubero_common/logger.h>
#include <functional>
#include <map>
#include <string>

namespace hubero {

class FsmEssentials {
public:
    void addTransitionHandler(const int& state_src, const int& state_dst, std::function<void()> handler) {
        transition_handlers_map_.insert({std::make_pair(state_src, state_dst), std::move(handler)});
    }

    void setLoggingVerbosity(bool enable_verbose) {
		logging_verbose_ = enable_verbose;
	}

    /**
     * @brief Updates logger preamble - typically actor name will be put there
     */
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

    int transitionHandler(const int& state_src, const int& state_dst) {
        int calls_counter = 0;

        // first, try to find handler with a given key, defined by state_src and state_dst
        auto it_handler = transition_handlers_map_.find(std::make_pair(state_src, state_dst));
        if (it_handler == transition_handlers_map_.end()) {
            return calls_counter;
        }

        // key basically was found, so now find set of entries matching given key
        auto range = transition_handlers_map_.equal_range(std::make_pair(state_src, state_dst));
        for (auto i = range.first; i != range.second; ++i) {
            i->second();
            calls_counter++;
        }
        return calls_counter;
    }

    std::string fsm_name_;
    std::multimap<std::pair<int, int>, std::function<void()>> transition_handlers_map_;

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
