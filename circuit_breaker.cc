/*
*  WEBRTC-CB
*/

#include "modules/congestion_controller/circuit_breaker.h"

#include "common_types.h"  // NOLINT(build/include)
#include "modules/rtp_rtcp/source/time_util.h"
#include <cmath>
#include <array>
#include "rtc_base/logging.h"


namespace webrtc {
	CircuitBreaker::CircuitBreaker(RTPSender* rtp_sender, Clock* clock)
	{
		rtc::LogMessage::LogToDebug(rtc::LS_INFO);
		LOG(LS_INFO) << "Circuit Breaker created.";
		rtp_sender_ = rtp_sender;
		clock_ = clock;
		previous_report_blocks_ = new ReportBlockInfoMap();
		std::string codec = "unknown";
		switch (rtp_sender->VideoCodecType()) {
		case(kRtpVideoVp8):
			frame_group_size_ = 1;
			media_framing_interval_ = GetMediaFramingInterval();
			codec = "VP8";
			break;
		case(kRtpVideoVp9):
			frame_group_size_ = 1;
			media_framing_interval_ = GetMediaFramingInterval();
			codec = "VP9";
			break;
		case(kRtpVideoH264):			// TODO Implement a framerate timestamp and pick the smallest one in the last 10 seconds
			frame_group_size_ = 1;
			media_framing_interval_ = GetMediaFramingInterval();
			codec = "h264";
			break;
/*		TODO Commented out to test check in RTP Sender
			case(kRtpVideoGeneric):
			// Default config
			break;
		default:
			// Disable CB for all other values 
*/
		}
		LOG(LS_INFO) << "Codec: " << codec;
		rtcp_interval_ = 2.5;				  // Initial guess = Tmin when RTCP packets not yet send || TODO Calculate better value? 
		LOG(LS_INFO) << "Initial RTCP Interval value: " << rtcp_interval_;
		rtcp_interval_sender_estimate_ = 2.5; // Initial guess = Tmin when RTCP packets not yet send || TODO Calculate better value?
		LOG(LS_INFO) << "Initial Sender Estimate of RTCP Interval value: " << rtcp_interval_sender_estimate_;
		int64_t initial_rtt = 100;			  // RTT guess in ms
		LOG(LS_INFO) << "Initial RTT value: " << initial_rtt;
		initial_cb_interval_ = ceil(3 * fmin(fmax(fmax(10 * frame_group_size_ * media_framing_interval_, (uint64_t)(10 * initial_rtt)),
			3 * rtcp_interval_sender_estimate_), fmax(15, 3 * rtcp_interval_)) / (3 * rtcp_interval_sender_estimate_));
		LOG(LS_INFO) << "Initial calculated CB_INTERVAL value: " << initial_cb_interval_;
	}

	CircuitBreaker::~CircuitBreaker() {
		delete previous_report_blocks_;
	}

	void CircuitBreaker::OnReceivedRtcpReportBlocks(
		const ReportBlockList& report_blocks, uint32_t receiver_estimated_max_bitrate_bps) {
			rtcp_interval_sender_estimate_ =														// Update the sender's estimate
				CalculateReceiverIntervalEstimateFromBitrate(receiver_estimated_max_bitrate_bps);
			LOG(LS_INFO) << "Current Estimated Max Bitrate bps: " << receiver_estimated_max_bitrate_bps;
			LOG(LS_INFO) << "Sender Estimate of RTCP Interval updated: " << rtcp_interval_sender_estimate_;
			uint32_t send_time_ntp;
			for (const RTCPReportBlock& report_block : report_blocks) {								// For each report block
				send_time_ntp = report_block.last_sender_report_timestamp;							// Get the last SR timestamp
				if (send_time_ntp != 0) {															// If 0 abort as calculations will be ineffective
					int64_t rtt_new = 0;															// Var to hold the RTT value calc. from this rep. block
					uint32_t delay_ntp = report_block.delay_since_last_sender_report;				// Get the DLSR
					uint32_t receive_time_ntp = CompactNtp(clock_->CurrentNtpTime());				// Get the current local NTP time.
					uint32_t rtt_ntp = receive_time_ntp - delay_ntp - send_time_ntp;				// Get the RTT in 1/(2^16) seconds.
					rtt_new = CompactNtpRttToMs(rtt_ntp);											// Convert RTT to 1/1000 seconds (milliseconds).
					if (previous_report_blocks_->find(report_block.sender_ssrc) ==					// If first time receiving from this SSRC
														previous_report_blocks_->end()) {
						CbIntervalRttSmoothedPair cb_rtt =											// Prepare the new structures to be inserted 
							CbIntervalRttSmoothedPair(initial_cb_interval_, rtt_new);				// in the map of prev. info blocks
						CbIntrvlRttReportBlockListPair cb_rtt_rep_block_list =						// to represent this SSRC
							CbIntrvlRttReportBlockListPair(cb_rtt, ReportBlockInfoList());
						previous_report_blocks_->insert(											// Insert into the map
							std::pair<uint32_t, CbIntrvlRttReportBlockListPair>(report_block.sender_ssrc, cb_rtt_rep_block_list));
					}
					else {																			// Else (if SSRC already present)
						(*previous_report_blocks_)[report_block.sender_ssrc].first.second =			// Blend the new RTT value with the total RTT
							(0.8*(*previous_report_blocks_)[report_block.sender_ssrc].first.second) + (0.2*rtt_new);
					}
						uint8_t fraction_lost;
						fraction_lost = report_block.fraction_lost;									// Get the fraction lost
						ReportBlockInfo report_block_info;											// Prepare the new rep. block info record
						report_block_info.dlsr = report_block.delay_since_last_sender_report;		
						report_block_info.fraction_lost = fraction_lost;
						if ((*previous_report_blocks_)[report_block.sender_ssrc].second.size() >	// If more than CB_INT packets received from this SSRC
							(*previous_report_blocks_)[report_block.sender_ssrc].first.first) {		
							ReportBlockInfoList rep_block_info_list_for_last_interval =				// Grab the rep. block info list for this 
								(*previous_report_blocks_)[report_block.sender_ssrc].second;		// circuit breaker interval
							(*previous_report_blocks_)[report_block.sender_ssrc].second.clear();	// Clear it from the records
							uint32_t avg_fraction_lost = 0;											// Calculate the average fraction lost
							uint16_t count = 0;														// for this circuit breaker interval
							for (const ReportBlockInfo& info : rep_block_info_list_for_last_interval) {	
								avg_fraction_lost += info.fraction_lost;
								count++;
							}
							avg_fraction_lost /= count;
							float loss_event_rate = avg_fraction_lost / 256.0;						// Format it to fit the loss event rate
							uint8_t num_packets_per_ack = 1;										// b = 1 (RFC 8083 Section 3)
							uint64_t tcp_rate = packet_size_ / ((*previous_report_blocks_)[report_block.sender_ssrc].first.second 
								* sqrt(2 * num_packets_per_ack * loss_event_rate / 3));				// Calculate the TCP rate with the same conditions
							LOG(LS_INFO) << "Calculated TCP rate: " << tcp_rate;
							uint16_t sending_rate = rtp_sender_->ActualSendBitrateKbit() * 1000;	// Calculate the actual UDP sending rate
							LOG(LS_INFO) << "Calculated UDP rate: " << sending_rate;
							if (sending_rate > 10 * tcp_rate) {										// If the actual rate is 10 times larger
								LOG(LS_INFO) << "!!! Congestion CB triggered !!! ";
								CbTriggered(kCongestion);											// The Congestion CB has triggered
							}
						}
						(*previous_report_blocks_)[report_block.sender_ssrc].second					// Add the new rep. block info to the records
							.push_back(report_block_info);
						// TODO Check for RTCP & media timeout CBs						
						RecalculateCbInterval(report_block.sender_ssrc);							// Calculate the new CB_INT for this SSRC
				}
			}
	}

	void CircuitBreaker::RecalculateCbInterval(uint32_t sender_ssrc) {
		media_framing_interval_ = GetMediaFramingInterval();
		LOG(LS_INFO) << "Media Framing Interval updated to: " << media_framing_interval_;
		(*previous_report_blocks_)[sender_ssrc].first.first = 
			  ceil(3 * fmin(fmax(fmax(10 * frame_group_size_ * media_framing_interval_, (uint64_t)(10 * (*previous_report_blocks_)[sender_ssrc].first.second)),
				                 3 * rtcp_interval_sender_estimate_), fmax(15, 3 * rtcp_interval_)) / (3 * rtcp_interval_sender_estimate_));
		LOG(LS_INFO) << "CB_INTERVAL for SSRC: " << sender_ssrc << " updated to: " << (*previous_report_blocks_)[sender_ssrc].first.first;
	}

	void CircuitBreaker::CbTriggered(CbTriggerTypes flag) {
		switch (flag) {
			case(0): media_timeout_cb_ = true;
				HandleMediaTimeoutCb();
			case(1): rtcp_timeout_cb_ = true;
				HandleRtcpTimeoutCb();
			case(2): congestion_cb_ = true;
				HandleCongestionCb();
			case(3): media_usability_cb_ = true;
				HandleMediaUsabilityCb();
		}
	}

	void CircuitBreaker::PushPacketSize(size_t packet_size) {
		uint32_t size = static_cast<uint32_t>(packet_size);
		if (packet_sizes_.size() == 4 * frame_group_size_) {
			packet_sizes_.erase(packet_sizes_.begin());
		}
		packet_sizes_.push_back(size);
		uint32_t avg_packet_size = 0;
		for (std::vector<uint32_t>::iterator it = packet_sizes_.begin(); it != packet_sizes_.end(); it++) {
			avg_packet_size += (*it);
		}
		packet_size_ = avg_packet_size / (4 * frame_group_size_);
		LOG(LS_INFO) << "Average packet size updated to: " << packet_size_ << "after receving packet of size: " << packet_size;
	}

	uint32_t CircuitBreaker::GetMediaFramingInterval() {
		int new_frame_rate = rtp_sender_->GetSendFrameRate();
		LOG(LS_INFO) << "Send Frame Rate value updated: " << new_frame_rate;
		return static_cast<uint32_t>(FpsToInterval(new_frame_rate));
	}

	int64_t CircuitBreaker::FpsToInterval(int fps) {
		return fps ? rtc::kNumNanosecsPerSec / fps : rtc::kNumNanosecsPerSec / 10000;
	}

	void CircuitBreaker::HandleMediaTimeoutCb() {}

	void CircuitBreaker::HandleRtcpTimeoutCb() {}

	void CircuitBreaker::HandleCongestionCb() {
		// TODO Implement handling
		// Signal class to reduce rate or cease transmission
		// Log circuit breaker triggered
	}

	void CircuitBreaker::HandleMediaUsabilityCb() {}

	std::array<bool, 4> CircuitBreaker::GetCbStatus() {
		std::array<bool, 4> cb_status;
		cb_status[0] = media_timeout_cb_;
		cb_status[1] = rtcp_timeout_cb_;
		cb_status[2] = congestion_cb_;
		cb_status[3] = media_usability_cb_;
		return cb_status;
	}

	void CircuitBreaker::ClearCb() {
		media_timeout_cb_ = false;
		rtcp_timeout_cb_ = false;
		congestion_cb_ = false;
		media_usability_cb_ = false;
	}

	void CircuitBreaker::SetRtcpInterval(uint32_t rtcp_interval) {
		LOG(LS_INFO) << "RTCP Interval value updated to: " << rtcp_interval;
		rtcp_interval_ = rtcp_interval;
	}

	int64_t CircuitBreaker::CalculateReceiverIntervalEstimateFromBitrate(uint32_t receiver_estimated_max_bitrate_bps) {
		const int kRtcpSize = 80;
		const int64_t kMinFeedbackIntervalMs = 200;
		const int64_t kMaxFeedbackIntervalMs = 1000;
		const int64_t interval = static_cast<int64_t>(
			kRtcpSize * 8.0 * 1000.0 / (0.05 * receiver_estimated_max_bitrate_bps) + 0.5);
		if (interval < kMinFeedbackIntervalMs) {
			return kMinFeedbackIntervalMs;
		}
		else if (interval > kMaxFeedbackIntervalMs) {
			return kMaxFeedbackIntervalMs;
		}
		else {
			return interval;
		}
	}

}  // namespace webrtc
