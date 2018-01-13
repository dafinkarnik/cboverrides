/*
*  WEBRTC-CB
*/

#ifndef MODULES_CONGESTION_CONTROLLER_CIRCUIT_BREAKER_H_
#define MODULES_CONGESTION_CONTROLLER_CIRCUIT_BREAKER_H_

#include <initializer_list>

#include "common_types.h"  // NOLINT(build/include)
#include "rtc_base/criticalsection.h"
#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/rtp_rtcp/source/rtp_sender.h"
#include <vector>
#include <ctime>
#include <sstream>

namespace webrtc {

	enum CbTriggerTypes {
		kNone = 0,
		kMediaTimeout = 1,
		kRtcpTimeout = 2,
		kCongestion = 3,
		kMediaUsability = 4
	};

	struct ReportBlockInfo {
		uint32_t dlsr;
		uint8_t fraction_lost;
	};

	// This class controls initiation of probing to estimate initial channel
	// capacity. There is also support for probing during a session when max
	// bitrate is adjusted by an application.
	class CircuitBreaker {
	public:
		CircuitBreaker(RTPSender* rtp_sender, Clock* clock);
		~CircuitBreaker();

		void OnReceivedRtcpReportBlocks(const ReportBlockList& report_blocks, uint32_t receiver_estimated_max_bitrate_bps);

		void RecalculateCbInterval(uint32_t sender_ssrc);

		void ClearCb();

		std::array<bool, 4> CircuitBreaker::GetCbStatus();

		void PushPacketSize(size_t packet_size);

		void SetRtcpInterval(uint32_t rtcp_interval);

		uint32_t GetMediaFramingInterval();

		int64_t FpsToInterval(int fps);
		
	protected:
		/*void UpdateCbInterval(uint32_t cb_interval) {
			cb_interval_ = cb_interval;
		}*/

		void UpdateFrameGroupSize(uint32_t frame_group_size) {
			frame_group_size_ = frame_group_size;
		}

		void UpdateMediaFramingInterval(uint32_t media_framing_interval) {
			media_framing_interval_ = media_framing_interval;
		}
	private:
		bool media_timeout_cb_ = false;
		bool rtcp_timeout_cb_ = false;
		bool congestion_cb_ = false;
		bool media_usability_cb_ = false;
		void CbTriggered(CbTriggerTypes flag);

		RTPSender* rtp_sender_;
		Clock* clock_;
		using ReportBlockInfoList = std::list<ReportBlockInfo>;
		using CbIntervalRttSmoothedPair = std::pair <uint32_t, int64_t>;
		using CbIntrvlRttReportBlockListPair = std::pair<CbIntervalRttSmoothedPair, ReportBlockInfoList>;
		using ReportBlockInfoMap = std::map<uint32_t, CbIntrvlRttReportBlockListPair>;
		ReportBlockInfoMap* previous_report_blocks_;
		uint32_t initial_cb_interval_;
		uint32_t frame_group_size_;
		uint32_t media_framing_interval_;
		uint32_t rtcp_interval_;
		int64_t rtcp_interval_sender_estimate_;
		std::vector<uint32_t> packet_sizes_;
		uint32_t packet_size_;

		void HandleMediaTimeoutCb();
		void HandleRtcpTimeoutCb();
		void HandleCongestionCb();
		void HandleMediaUsabilityCb();

		int64_t CalculateReceiverIntervalEstimateFromBitrate(uint32_t receiver_estimated_max_bitrate_bps);

		//RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(CircuitBreaker);
	};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_CIRCUIT_BREAKER_H_