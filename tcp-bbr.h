/*
 * Copyright (c) 2018 NITK Surathkal
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Vivek Jain <jain.vivek.anand@gmail.com>
 *          Viyom Mittal <viyommittal@gmail.com>
 *          Mohit P. Tahiliani <tahiliani@nitk.edu.in>
 */

#ifndef TCPBBR_H
#define TCPBBR_H

#include "ns3/data-rate.h"
#include "ns3/random-variable-stream.h"
#include "ns3/tcp-congestion-ops.h"
#include "ns3/traced-value.h"
#include "ns3/windowed-filter.h"

class TcpBbrCheckGainValuesTest;

namespace ns3
{

/**
 * \ingroup congestionOps
 *
 * \brief BBR congestion control algorithm
 *
 * This class implement the BBR (Bottleneck Bandwidth and Round-trip propagation time)
 * congestion control type.
 */
class TcpBbr : public TcpCongestionOps
{
  public:
    struct bbr_context {
	    DataRate sample_bw;
    };
    /**
     * \brief The number of phases in the BBR ProbeBW gain cycle.
     */
    static const uint8_t GAIN_CYCLE_LENGTH = 8;

    /**
     * \brief BBR uses an eight-phase cycle with the given pacing_gain value
     * in the BBR ProbeBW gain cycle.
     */
    const static double PACING_GAIN_CYCLE[];
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    TcpBbr();

    /**
     * Copy constructor.
     * \param sock The socket to copy from.
     */
    TcpBbr(const TcpBbr& sock);

    /**
     * \brief BBR has the following 4 modes for deciding how fast to send:
     */
    enum BbrMode_t
    {
        BBR_STARTUP,   /**< Ramp up sending rate rapidly to fill pipe */
        BBR_DRAIN,     /**< Drain any queue created during startup */
        BBR_PROBE_BW,  /**< Discover, share bw: pace around estimated bw */
        BBR_PROBE_RTT, /**< Cut inflight to min to probe min_rtt */
    };

    enum BbrPacingGainPhase_t 
    {
        BBR_BW_PROBE_UP,  /* push up inflight to probe for bw/vol */
        BBR_BW_PROBE_DOWN,  /* drain excess inflight from the queue */
        BBR_BW_PROBE_CRUISE,  /* use pipe, w/ headroom in queue/pipe */
        BBR_BW_PROBE_REFILL,  /* v2: refill the pipe again to 100% */
    };
    enum BbrAckPhase_t 
    {
        BBR_ACKS_INIT,		  /* not probing; not getting probe feedback */
        BBR_ACKS_REFILLING,	  /* sending at est. bw to fill pipe */
        BBR_ACKS_PROBE_STARTING,  /* inflight rising to probe bw */
        BBR_ACKS_PROBE_FEEDBACK,  /* getting feedback from bw probing */
        BBR_ACKS_PROBE_STOPPING,  /* stopped probing; still getting feedback */
    };

    /**
     * \brief Literal names of BBR mode for use in log messages
     */
    static const char* const BbrModeName[BBR_PROBE_RTT + 1];


    /**
     * \brief Literal names of cycle modes for use in log messages
     */
    static const char* const BbrCycleName[BBR_PROBE_RTT + 1];
    /**
     * Assign a fixed random variable stream number to the random variables
     * used by this model.
     *
     * \param stream first stream index to use
     */
    virtual void SetStream(uint32_t stream);

    std::string GetName() const override;
    bool HasCongControl() const override;

    void bbr_update_gains(); 

    void CongControl(Ptr<TcpSocketState> tcb,
                     const TcpRateOps::TcpRateConnection& rc,
                     const TcpRateOps::TcpRateSample& rs) override;
    void CongestionStateSet(Ptr<TcpSocketState> tcb,
                            const TcpSocketState::TcpCongState_t newState) override;
    void CwndEvent(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event) override;
    uint32_t GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight) override;
    Ptr<TcpCongestionOps> Fork() override;

  protected:
    /**
     * \brief TcpBbrCheckGainValuesTest friend class (for tests).
     * \relates TcpBbrCheckGainValuesTest
     */
    friend class TcpBbrCheckGainValuesTest;

    /**
     * \brief Advances pacing gain using cycle gain algorithm, while in BBR_PROBE_BW state
     */
    void AdvanceCyclePhase();

    /**
     * \brief Checks whether to advance pacing gain in BBR_PROBE_BW state,
     *  and if allowed calls AdvanceCyclePhase ()
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void CheckCyclePhase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    bool bbr_check_time_to_probe_bw(Ptr<TcpSocketState> tcb);

    bool bbr_check_time_to_cruise(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, DataRate bw);  


    void bbr_start_bw_probe_up(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context* ctx);

    void bbr_start_bw_probe_down();

    void bbr_start_bw_probe_refill(Ptr<TcpSocketState> tcb, uint32_t bw_probe_up_rounds);

    void bbr_start_bw_probe_cruise();

    bool bbr_has_elapsed_in_phase(Ptr<TcpSocketState> tcb, Time interval);

    bool bbr_is_reno_coexistence_probe_time(Ptr<TcpSocketState> tcb);

    uint32_t bbr_probe_rtt_cwnd(Ptr<TcpSocketState> tcb);

    void bbr_update_cycle_phase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx);

    void bbr_update_latest_delivery_signals(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx);

    void bbr_set_cycle_idx(uint32_t cycle_idx);

    /**
     * \brief Checks whether its time to enter BBR_DRAIN or BBR_PROBE_BW state
     * \param tcb the socket state.
     */
    void bbr_check_drain(Ptr<TcpSocketState> tcb);

    /**
     * \brief Identifies whether pipe or BDP is already full
     * \param rs rate sample.
     */
    void bbr_check_full_bw_reached(const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx);

    bool bbr_full_bw_reached();

    /**
     * \brief This method handles the steps related to the ProbeRTT state
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void CheckProbeRTT(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    /**
     * \brief Updates variables specific to BBR_DRAIN state
     */
    void EnterDrain();

    /**
     * \brief Updates variables specific to BBR_PROBE_BW state
     */
    void EnterProbeBW();

    /**
     * \brief Updates variables specific to BBR_PROBE_RTT state
     */
    void EnterProbeRTT();

    /**
     * \brief Updates variables specific to BBR_STARTUP state
     */
    void EnterStartup();

    /**
     * \brief Called on exiting from BBR_PROBE_RTT state, it eithers invoke EnterProbeBW () or
     * EnterStartup ()
     */
    void ExitProbeRTT();

    /**
     * \brief Gets BBR state.
     * \return returns BBR state.
     */
    uint32_t GetBbrState();

    /**
     * \brief Gets current pacing gain.
     * \return returns current pacing gain.
     */
    double GetPacingGain();

    /**
     * \brief Gets current cwnd gain.
     * \return returns current cwnd gain.
     */
    double GetCwndGain();

    /**
     * \brief Handles the steps for BBR_PROBE_RTT state.
     * \param tcb the socket state.
     */
    void HandleProbeRTT(Ptr<TcpSocketState> tcb);

    /**
     * \brief Updates pacing rate if socket is restarting from idle state.
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void HandleRestartFromIdle(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    /**
     * \brief Estimates the target value for congestion window
     * \param tcb  the socket state.
     * \param gain cwnd gain.
     * \return returns congestion window based on max bandwidth and min RTT.
     */
    uint32_t InFlight(Ptr<TcpSocketState> tcb, DataRate bw, double gain);

    bool bbr_is_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_handle_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_probe_inflight_hi_upward(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_raise_inflight_hi_slope(Ptr<TcpSocketState> tcb);

    uint32_t bbr_target_inflight(Ptr<TcpSocketState> tcb );

    uint32_t bbr_inflight_with_headroom();

    void bbr_bound_cwnd_for_inflight_model(Ptr<TcpSocketState> tcb);

    /**
     * \brief Initializes the full pipe estimator.
     */
    void InitFullPipe();

    /**
     * \brief Initializes the pacing rate.
     * \param tcb  the socket state.
     */
    void InitPacingRate(Ptr<TcpSocketState> tcb);

    bool bbr_is_probing_bandwidth(Ptr<TcpSocketState> tcb);

    /**
     * \brief Initializes the round counting related variables.
     */
    void InitRoundCounting();

    /**
     * \brief Checks whether to move to next value of pacing gain while in BBR_PROBE_BW.
     * \param tcb the socket state.
     * \param rs  rate sample.
     * \returns true if want to move to next value otherwise false.
     */
    bool IsNextCyclePhase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    /**
     * \brief Modulates congestion window in CA_RECOVERY.
     * \param tcb the socket state.
     * \param rs rate sample.
     * \return true if congestion window is updated in CA_RECOVERY.
     */
    bool ModulateCwndForRecovery(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    /**
     * \brief Helper to restore the last-known good congestion window
     * \param tcb the socket state.
     */
    void RestoreCwnd(Ptr<TcpSocketState> tcb);

    /**
     * \brief Helper to remember the last-known good congestion window or
     *        the latest congestion window unmodulated by loss recovery or ProbeRTT.
     * \param tcb the socket state.
     */
    void bbr_save_cwnd(Ptr<const TcpSocketState> tcb);

    /**
     * \brief Updates congestion window based on the network model.
     * \param tcb the socket state.
     * \param rs  rate sample
     */
    void SetCwnd(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    /**
     * \brief Updates pacing rate based on network model.
     * \param tcb the socket state.
     * \param gain pacing gain.
     */
    void SetPacingRate(Ptr<TcpSocketState> tcb, double gain);

    void bbr_init_lower_bounds(Ptr<TcpSocketState> tcb, bool init_bw);

    void bbr_loss_lower_bounds(Ptr<TcpSocketState> tcb, DataRate *bw, uint32_t *inflight);

    /**
     * \brief Updates send quantum based on the network model.
     * \param tcb the socket state.
     */
    void SetSendQuantum(Ptr<TcpSocketState> tcb);

    /**
     * \brief Updates maximum bottleneck.
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void bbr_update_congestion_signals(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx);

    void bbr_check_loss_too_high_in_startup(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_handle_queue_too_high_in_startup(Ptr<TcpSocketState> tcb);

    /**
     * \brief Updates BBR network model (Maximum bandwidth and minimum RTT).
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void bbr_update_model(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx);

    /**
     * \brief Updates round counting related variables.
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    uint32_t bbr_update_round_start(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    DataRate bbr_max_bw();

    void bbr_advance_max_bw_filter();

    void bbr_take_max_bw_sample(DataRate bw); 

    DataRate bbr_bw();

    uint32_t bbr_bdp(Ptr<TcpSocketState> tcb, DataRate bw, double gain);

    void bbr_reset_full_bw();

    void bbr_reset_lower_bounds();

    bool bbr_adapt_upper_bounds(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_adapt_lower_bounds(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs);

    void bbr_reset_congestion_signals();

    void bbr_calculate_bw_sample(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, bbr_context *ctx);

    void bbr_pick_probe_wait();


    /**
     * \brief Updates minimum RTT.
     * \param tcb the socket state.
     */
    void UpdateRTprop(Ptr<TcpSocketState> tcb);

    void bbr_update_min_rtt(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

    void bbr_check_probe_rtt_done(Ptr<TcpSocketState> tcb);

    void bbr_exit_probe_rtt(Ptr<TcpSocketState> tcb);

    /**
     * \brief Updates target congestion window.
     * \param tcb the socket state.
     */
    void UpdateTargetCwnd(Ptr<TcpSocketState> tcb);

    /**
     * \brief Sets BBR state.
     * \param state BBR state.
     */
    void SetBbrState(BbrMode_t state);

    /**
     * \brief Find Cwnd increment based on ack aggregation.
     * \return uint32_t aggregate cwnd.
     */
    uint32_t AckAggregationCwnd();

    /**
     * \brief Estimates max degree of aggregation.
     * \param tcb the socket state.
     * \param rs rate sample.
     */
    void bbr_update_ack_aggregation(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs);

  private:
    //  u32	min_rtt_us;	    /* min RTT in min_rtt_win_sec window */
    TracedValue<Time>  m_rtProp{Time::Max()}; //!< Estimated two-way round-trip propagation delay of the path, estimated from the windowed minimum recent round-trip delay sample.
    //  u32	min_rtt_stamp;  /* timestamp of min_rtt_us */
    Time m_rtPropStamp{Seconds(0)}; //!< The wall clock time at which the current BBR.RTProp sample was obtained
    //  u32	probe_rtt_done_stamp;   /* end time for BBR_PROBE_RTT mode */
    Time m_probeRttDoneStamp{Seconds(0)}; //!< Time to exit from BBR_PROBE_RTT state
    //  u32	probe_rtt_min_us;	/* min RTT in probe_rtt_win_ms win */
    Time m_probeRttMin{Time::Max()}; 
    //  u32	probe_rtt_min_stamp;	/* timestamp of probe_rtt_min_us*/
    Time m_probeRttMinStamp{Seconds(0)}; //!< Timestamp of probe_rtt_min_us
    
    
    // Time m_probeRtPropStamp{Seconds(0)}; //!< The wall clock time at which the current BBR.RTProp sample was obtained.
    
    

    BbrMode_t m_state{BbrMode_t::BBR_STARTUP}; //!< Current state of BBR state machine
    
    uint32_t m_bandwidthWindowLength{0}; //!< A constant specifying the length of the BBR.BtlBw max filter window, default 10 packet-timed round trips.
    uint32_t m_bwProbeUpRounds{0}; //!< Number of round trips to probe for bandwidth
    double m_pacingGain{0};              //!< The dynamic pacing gain factor
    double m_cWndGain{0};                //!< The dynamic congestion window gain factor
    double m_highGain{0};       //!< A constant specifying highest gain factor, default is 2.89
    bool m_fullBwReached{false}; //!< A boolean that records whether BBR has filled the pipe
    uint32_t m_minPipeCwnd{0}; //!< The minimal congestion window value BBR tries to target, default 4 Segment size
    uint32_t m_roundCount{0}; //!< Count of packet-timed round trips
    bool m_roundStart{false}; //!< A boolean that BBR sets to true once per packet-timed round trip
    uint32_t m_nextRoundDelivered{0};           //!< Denotes the end of a packet-timed round trip
    Time m_probeRttDuration{MilliSeconds(200)}; //!< A constant specifying the minimum duration for which ProbeRTT state, default 200 millisecs
    bool m_probeRttRoundDone{false};      //!< True when it is time to exit BBR_PROBE_RTT
    bool m_packetConservation{false};     //!< Enable/Disable packet conservation mode
    uint32_t m_priorCwnd{0};              //!< The last-known good congestion window
    bool m_idleRestart{false};            //!< When restarting from idle, set it true
    uint32_t m_targetCWnd{0}; //!< Target value for congestion window, adapted to the estimated BDP
    DataRate m_fullBandwidth{0};      //!< Value of full bandwidth recorded
    uint32_t m_fullBandwidthCount{0}; //!< Count of full bandwidth recorded consistently
    uint32_t m_sendQuantum{0}; //!< The maximum size of a data aggregate scheduled and transmitted together
    Time m_cycleStamp{Seconds(0)};       //!< Last time gain cycle updated
    uint32_t m_cycleIndex{2};            //!< Current index of gain cycle
    bool m_rtPropExpired{false};         //!< A boolean recording whether the BBR.RTprop has expired
    Time m_rtPropFilterLen{Seconds(10)}; //!< A constant specifying the length of the RTProp min filter window, default 10 secs.
    bool m_isInitialized{false}; //!< Set to true after first time initializtion variables
    Ptr<UniformRandomVariable> m_uv{nullptr}; //!< Uniform Random Variable
    uint64_t m_delivered{0}; //!< The total amount of data in bytes delivered so far
    uint32_t m_appLimited{0}; //!< The index of the last transmitted packet marked as application-limited
    uint32_t m_txItemDelivered{0}; //!< The number of bytes already delivered at the time of new packet transmission
    uint32_t m_extraAckedGain{1};         //!< Gain factor for adding extra ack to cwnd
    uint32_t m_extraAcked[2]{0, 0};       //!< Maximum excess data acked in epoch
    uint32_t m_extraAckedWinRtt{0};       //!< Age of extra acked in rtt
    uint32_t m_extraAckedWinRttLength{5}; //!< Window length of extra acked window
    uint32_t m_ackEpochAckedResetThresh{1 << 17}; //!< Max allowed val for m_ackEpochAcked, after which sampling epoch is reset
    uint32_t m_extraAckedIdx{0};     //!< Current index in extra acked array
    Time m_ackEpochTime{Seconds(0)}; //!< Starting of ACK sampling epoch time
    uint32_t m_ackEpochAcked{0};     //!< Bytes ACked in sampling epoch
    bool m_hasSeenRtt{false};        //!< Have we seen RTT sample yet?

    // bbrv3 states 

    bool m_tryFastPath{false}; //!< Try to use fast path
        // 		full_bw_now:1,		/* recently reached full bw plateau? */
    bool m_fullBandwidthNow{false};     //!< Recently reached full bandwidth plateau
	// 	startup_ecn_rounds:2,	/* consecutive hi ECN STARTUP rounds */[]
    uint32_t m_startupEcnRounds{0};     //!< Consecutive high ECN STARTUP rounds
	// 	loss_in_cycle:1,	/* packet loss in this cycle? */
    bool m_lossInCycle{false};        //!< Packet loss in this cycle
	// 	ecn_in_cycle:1,		/* ECN in this cycle? */
    bool m_ecn_in_cycle{false};       //!<ble ECN in this cycle
	// unused_3:1;
	// u32	loss_round_delivered; /* scb->tx.delivered ending loss round */
    uint32_t m_lossRoundDelivered{0};   //!< Delivered packets at the end of loss round
	// u32	undo_bw_lo;	     /* bw_lo before latest losses */
	DataRate m_undoBwLo{0};             //!< bw_lo before latest losses
    // u32	undo_inflight_lo;    /* inflight_lo before latest losses */
    uint32_t m_undoInflightLo{0};       //!< inflight_lo before latest losses
	// u32	undo_inflight_hi;    /* inflight_hi before latest losses */
    uint32_t m_undoInflightHi{0};       //!< inflight_hi before latest losses
	// u32	bw_latest;	 /* max delivered bw in last round trip */
    DataRate m_bwLatest{std::numeric_limits<int>::max ()};         //!< Maximum delivered bandwidth in last round trip
	// u32	bw_lo;		 /* lower bound on sending bandwidth */
    DataRate m_bwLo{std::numeric_limits<int>::max ()};             //!< Lower bound on sending bandwidth
	// u32	bw_hi[2];	 /* max recent measured bw sample */
    DataRate bw_hi[2]{0, 0};
	// u32	inflight_latest; /* max delivered data in last round trip */
    uint32_t m_inflightLatest{0};   //!< Maximum delivered data in last round trip
	// u32	inflight_lo;	 /* lower bound of inflight data range */
	TracedValue<uint32_t> m_inflightLo{std::numeric_limits<int>::max ()};       //!< Lower bound of inflight data range
    // u32	inflight_hi;	 /* upper bound of inflight data range */
    TracedValue<uint32_t> m_inflightHi{std::numeric_limits<int>::max ()};       //!< Upper bound of inflight data range
	// u32	bw_probe_up_cnt; /* packets delivered per inflight_hi incr */
    uint32_t m_bwProbeUpCount{0};   //!< Packets delivered per inflight_hi incr
	// u32	bw_probe_up_acks;  /* packets (S)ACKed since inflight_hi incr */
	uint32_t m_bwProbeUpAcks{0};    //!< Packets (S)ACKed since inflight_hi incr
    // u32	probe_wait_us;	 /* PROBE_DOWN until next clock-driven probe */
    Time m_probeWaitTime{Seconds(0)};   //!< PROBE_DOWN until next clock-driven probe
	// u32	prior_rcv_nxt;	/* tp->rcv_nxt when CE state last changed */
	// u32	ecn_eligible:1,	/* sender can use ECN (RTT, handshake)? */
    bool m_ecnEligible{false};           //!< Sender can use ECN (RTT, handshake)?
	// 	ecn_alpha:9,	/* EWMA delivered_ce/delivered; 0..256 */
	// 	bw_probe_samples:1,    /* rate samples reflect bw probing? */
    uint32_t m_bwProbeSamples{0};       //!< Rate samples reflect bw probing?
	// 	prev_probe_too_high:1, /* did last PROBE_UP go too high? */
	bool m_prevProbeTooHigh{false};     //!< Did last PROBE_UP go too high?
    // 	stopped_risky_probe:1, /* last PROBE_UP stopped due to risk? */
	bool m_stoppedRiskyProbe{false};    //!< Last PROBE_UP stopped due to risk?
    // 	rounds_since_probe:8,  /* packet-timed rounds since probed bw */
    bool m_roundsSinceProbe{false}; //!< Packet-timed rounds since probed bw
	// 	loss_round_start:1,    /* loss_round_delivered round trip? */
    bool m_lossRoundStart{false};       //!< Loss round delivered round trip?
	// 	loss_in_round:1,       /* loss marked in this round trip? */
    bool m_lossInRound{false};       //!< Loss marked in this round trip?
	// 	ecn_in_round:1,	       /* ECN marked in this round trip? */
    bool m_ecnInRound{false};       //!< ECN marked in this round trip?
	// 	ack_phase:3,	       /* bbr_ack_phase: meaning of ACKs */
    BbrAckPhase_t m_ackPhase{BbrAckPhase_t::BBR_ACKS_PROBE_FEEDBACK}; //!< BBR ack phase
	// 	loss_events_in_round:4,/* losses in STARTUP round */
    uint32_t m_lossEventsInRound{0}; //!< Losses in STARTUP round

    uint32_t bw_probe_up_rounds = 5;
    
    
    const Time bbr_min_rtt_win_sec = Seconds(10);

    const Time bbr_probe_rtt_win  = Seconds(5);

    const uint32_t bbr_bw_probe_max_rounds = 63;

    const double bbr_ecn_reprobe_gain = 1 / 2;

    const double bbr_loss_thresh = 2/100;

    const double bbr_ecn_thresh = 0.5;

    const double bbr_beta = 1 * 30 / 100;

    const double bbr_inflight_headroom = 15 / 100;

    const uint32_t bbr_full_loss_cnt = 6;

    const double bbr_full_bw_thresh = 1 * 5 / 4;

    const uint32_t bbr_full_bw_cnt = 3;

    const double bbr_startup_pacing_gain = 1 * 277 / 100 + 1;

    const uint32_t bbr_startup_cwnd_gain = 2;

    const double bbr_drain_gain = 1 * 1000 / 2885;

    const uint32_t bbr_bw_probe_cwnd_gain = 1;

    const uint32_t bbr_cwnd_gain  = 2;

    const Time bbr_probe_rtt_mode_ms = MilliSeconds(200);



     TracedValue<uint32_t> wildcard{0};

};

} // namespace ns3
#endif // TCPBBR_H