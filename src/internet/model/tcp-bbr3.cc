#include "tcp-bbr3.h"

#include "ns3/log.h"
#include "ns3/simulator.h"


#define  bbr_main CongControl

#define NOW Now().GetSeconds()
#define COUT(log) std::cout << log << std::endl
namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpBbr3");
NS_OBJECT_ENSURE_REGISTERED(TcpBbr3);

const double TcpBbr3::PACING_GAIN_CYCLE[] = {5.0 / 4,  91.0 / 100, 1, 1 };

TypeId
TcpBbr3::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpBbr3")
            .SetParent<TcpCongestionOps>()
            .AddConstructor<TcpBbr3>()
            .SetGroupName("Internet")
            .AddTraceSource("inflightHi",
                "infliht hi",
                MakeTraceSourceAccessor(&TcpBbr3::m_inflightHi),
                "ns3::TracedValueCallback::Uint32")
            .AddTraceSource("inflightLo",
                "infliht low",
                MakeTraceSourceAccessor(&TcpBbr3::m_inflightLo),
                "ns3::TracedValueCallback::Uint32")
            .AddTraceSource("rt_prop",
                "rt prop",
                MakeTraceSourceAccessor(&TcpBbr3::m_rtProp),
                "ns3::TracedValueCallback::Time")
            .AddTraceSource("maxBw",
                "Maximum bw estimate",
                MakeTraceSourceAccessor(&TcpBbr3::maxBw),
                "ns3::TracedValueCallback::Uint32")
            .AddTraceSource("wildcard",
                "wild card value",
                MakeTraceSourceAccessor(&TcpBbr3::wildcard),
                "ns3::TracedValueCallback::Uint32")
            .AddAttribute("Stream",
                          "Random number stream (default is set to 4 to align with Linux results)",
                          UintegerValue(4),
                          MakeUintegerAccessor(&TcpBbr3::SetStream),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HighGain",
                          "Value of high gain",
                          DoubleValue(2.89),
                          MakeDoubleAccessor(&TcpBbr3::m_highGain),
                          MakeDoubleChecker<double>())
            .AddAttribute("BwWindowLength",
                          "Length of bandwidth windowed filter",
                          UintegerValue(10),
                          MakeUintegerAccessor(&TcpBbr3::m_bandwidthWindowLength),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("RttWindowLength",
                          "Length of RTT windowed filter",
                          TimeValue(Seconds(10)),
                          MakeTimeAccessor(&TcpBbr3::m_rtPropFilterLen),
                          MakeTimeChecker())
            .AddAttribute("ProbeRttDuration",
                          "Time to be spent in PROBE_RTT phase",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&TcpBbr3::m_probeRttDuration),
                          MakeTimeChecker())
            .AddAttribute("ExtraAckedRttWindowLength",
                          "Window length of extra acked window",
                          UintegerValue(5),
                          MakeUintegerAccessor(&TcpBbr3::m_extraAckedWinRttLength),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute(
                "AckEpochAckedResetThresh",
                "Max allowed val for m_ackEpochAcked, after which sampling epoch is reset",
                UintegerValue(1 << 12),
                MakeUintegerAccessor(&TcpBbr3::m_ackEpochAckedResetThresh),
                MakeUintegerChecker<uint32_t>());
    return tid;
}

TcpBbr3::TcpBbr3()
    : TcpCongestionOps()
{
    NS_LOG_FUNCTION(this);
    m_uv = CreateObject<UniformRandomVariable>();
}

TcpBbr3::TcpBbr3(const TcpBbr3& sock)
    : TcpCongestionOps(sock),
      m_bandwidthWindowLength(sock.m_bandwidthWindowLength),
      m_pacingGain(sock.m_pacingGain),
      m_cWndGain(sock.m_cWndGain),
      m_highGain(sock.m_highGain),
      m_fullBwReached(sock.m_fullBwReached),
      m_minPipeCwnd(sock.m_minPipeCwnd),
      m_roundCount(sock.m_roundCount),
      m_roundStart(sock.m_roundStart),
      m_nextRoundDelivered(sock.m_nextRoundDelivered),
      m_probeRttDuration(sock.m_probeRttDuration),
      m_probeRttMinStamp(sock.m_probeRttMinStamp),
      m_probeRttDoneStamp(sock.m_probeRttDoneStamp),
      m_probeRttRoundDone(sock.m_probeRttRoundDone),
      m_packetConservation(sock.m_packetConservation),
      m_priorCwnd(sock.m_priorCwnd),
      m_idleRestart(sock.m_idleRestart),
      m_targetCWnd(sock.m_targetCWnd),
      m_fullBandwidth(sock.m_fullBandwidth),
      m_fullBandwidthCount(sock.m_fullBandwidthCount),
      m_rtProp(Time::Max()),
      m_sendQuantum(sock.m_sendQuantum),
      m_cycleStamp(sock.m_cycleStamp),
      m_cycleIndex(sock.m_cycleIndex),
      m_rtPropExpired(sock.m_rtPropExpired),
      m_rtPropFilterLen(sock.m_rtPropFilterLen),
      m_rtPropStamp(sock.m_rtPropStamp),
      m_isInitialized(sock.m_isInitialized),
      m_uv(sock.m_uv),
      m_delivered(sock.m_delivered),
      m_appLimited(sock.m_appLimited),
      m_txItemDelivered(sock.m_txItemDelivered),
      m_extraAckedGain(sock.m_extraAckedGain),
      m_extraAckedWinRtt(sock.m_extraAckedWinRtt),
      m_extraAckedWinRttLength(sock.m_extraAckedWinRttLength),
      m_ackEpochAckedResetThresh(sock.m_ackEpochAckedResetThresh),
      m_extraAckedIdx(sock.m_extraAckedIdx),
      m_ackEpochTime(sock.m_ackEpochTime),
      m_ackEpochAcked(sock.m_ackEpochAcked),
      m_hasSeenRtt(sock.m_hasSeenRtt)
{
    NS_LOG_FUNCTION(this);
}

const char* const TcpBbr3::BbrModeName[BBR_PROBE_RTT + 1] = {
    "BBR_STARTUP",
    "BBR_DRAIN",
    "BBR_PROBE_BW",
    "BBR_PROBE_RTT",
};


const char* const TcpBbr3::BbrCycleName[BBR_BW_PROBE_REFILL + 1] = {
    "BBR_BW_PROBE_UP",
    "BBR_BW_PROBE_DOWN",
    "BBR_BW_PROBE_CRUISE",
    "BBR_BW_PROBE_REFILL",
};
void
TcpBbr3::SetStream(uint32_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_uv->SetStream(stream);
}

void
TcpBbr3::InitRoundCounting()
{
    NS_LOG_FUNCTION(this);
    m_nextRoundDelivered = 0;
    m_roundStart = false;
    m_roundCount = 0;
}

void
TcpBbr3::InitFullPipe()
{
    NS_LOG_FUNCTION(this);
    m_fullBwReached = false;
    m_fullBandwidth = 0;
    m_fullBandwidthCount = 0;
}

void
TcpBbr3::InitPacingRate(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);

    if (!tcb->m_pacing)
    {
        NS_LOG_WARN("BBR must use pacing");
        tcb->m_pacing = true;
    }

    Time rtt;
    if (tcb->m_minRtt != Time::Max())
    {
        rtt = MilliSeconds(std::max<long int>(tcb->m_minRtt.GetMilliSeconds(), 1));
        m_hasSeenRtt = true;
    }
    else
    {
        rtt = MilliSeconds(1);
    }

    DataRate nominalBandwidth(tcb->m_cWnd * 8 / rtt.GetSeconds());
    tcb->m_pacingRate = DataRate(m_pacingGain * nominalBandwidth.GetBitRate());
    bbr_take_max_bw_sample(nominalBandwidth);
}

bool
TcpBbr3::bbr_is_probing_bandwidth(Ptr<TcpSocketState> tcb)
{
    return (m_state == BBR_STARTUP) ||
    (m_state == BBR_PROBE_BW &&
        (m_cycleIndex == BBR_BW_PROBE_REFILL ||
        m_cycleIndex == BBR_BW_PROBE_UP));
}

void
TcpBbr3::bbr_set_pacing_rate(Ptr<TcpSocketState> tcb, double gain)
{
    NS_LOG_FUNCTION(this << tcb << gain);
    DataRate rate((gain * bbr_bw().GetBitRate()) / 100 * 99); //  margin 
    rate = std::min(rate, tcb->m_maxPacingRate);
    if (!m_hasSeenRtt && tcb->m_minRtt != Time::Max())
        InitPacingRate(tcb);
    
    if (m_fullBwReached || rate > tcb->m_pacingRate)
        tcb->m_pacingRate = rate;
}

void 
TcpBbr3::bbr_init_lower_bounds(Ptr<TcpSocketState> tcb, bool init_bw)
{
    if (init_bw && m_bwLo == std::numeric_limits<int>::max ())
        m_bwLo = bbr_max_bw();
    if (m_inflightLo == std::numeric_limits<int>::max ())
        m_inflightLo = tcb->m_cWnd.Get();   
}

void
TcpBbr3::bbr_loss_lower_bounds(Ptr<TcpSocketState> tcb)
{
    m_bwLo =  std::max<DataRate>(m_bwLatest, DataRate(m_bwLo.GetBitRate() / 100 * 70));
    m_inflightLo = std::max<uint32_t>(m_inflightLatest, m_inflightLo / 100 * 70);
}

uint32_t 
TcpBbr3::bbr_inflight(Ptr<TcpSocketState> tcb, DataRate bw, double gain)
{
    NS_LOG_FUNCTION(this << tcb << gain);
    if (m_rtProp == Time::Max())
    {
        return tcb->m_initialCWnd * tcb->m_segmentSize;
    }
    double quanta = 3 * m_sendQuantum;
    double estimatedBdp = bbr_bdp(tcb, bw, gain);
    if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == 0)
    {
        return (estimatedBdp) + quanta + (2 * tcb->m_segmentSize);
    }
    return (estimatedBdp) + quanta;
}

bool 
TcpBbr3::bbr_is_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    // if (rs.delivered_ce > 0 && rs.m_delivered > 0 && m_ecnEligible /* && bbr_param(sk, ecn_thresh)*/)    //ECN FUNCTIONALITY 
    // { 
    //     double ecn_thresh = rs.m_delivered * bbr_ecn_thresh;
    //     if (rs.delivered_ce > static_cast<uint32_t>(ecn_thresh))
    //     {
    //         return true;
    //     }
    // }
    if (rs.m_bytesLoss > 0 && tcb->m_bytesInFlight > 0)
    {
        if (m_cycleIndex == BBR_BW_PROBE_UP)
        COUT("bytes lost " << rs.m_bytesLoss << " bytes in flight " << tcb->m_bytesInFlight.Get() / 100 * 2);
        if (rs.m_bytesLoss > (tcb->m_bytesInFlight.Get() / 100 * 2))
        {
            return true;
        }
            
        
    }
    return false;
}

void 
TcpBbr3::bbr_handle_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, bool rsmode)
{
    m_prevProbeTooHigh = true;
    m_bwProbeSamples = 0;
    if (rsmode && !rs.m_isAppLimited)
    {
        m_inflightHi = std::max(rs.m_priorInFlight, static_cast<uint32_t>(bbr_target_inflight(tcb) * (1 - bbr_beta)));
        goto done2;

    }
    if (!rs.m_isAppLimited)
        //m_inflightHi = std::max(tcb->m_bytesInFlight.Get(), static_cast<uint32_t>(bbr_target_inflight(tcb) * (1 - bbr_beta)));
        m_inflightHi = (bbr_target_inflight(tcb) * (1 - bbr_beta));
    done2:
    if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == BBR_BW_PROBE_UP)
    { 
        bbr_start_bw_probe_down();
    }
}

void
TcpBbr3::bbr_probe_inflight_hi_upward(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (tcb->m_cWnd < m_inflightHi)
    {
        m_bwProbeUpAcks = 0;
        return;
    }


    m_bwProbeUpAcks += rs.m_ackedSacked;
    if (m_bwProbeUpAcks >= m_bwProbeUpCount)
    {

        uint32_t delta = m_bwProbeUpAcks / m_bwProbeUpCount;
        m_bwProbeUpAcks -= delta * m_bwProbeUpCount;
        m_inflightHi += delta * tcb->m_segmentSize;
        m_tryFastPath = false;
    }

    if (m_roundStart)
        bbr_raise_inflight_hi_slope(tcb);

}

void 
TcpBbr3::bbr_raise_inflight_hi_slope(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);   //m_cWnd
    uint32_t growth_this_round = 1 << m_bwProbeUpRounds;
    m_bwProbeUpRounds = std::min<uint32_t>(m_bwProbeUpRounds + 1, 30);
    m_bwProbeUpCount = std::max<uint32_t> (tcb->m_cWnd / growth_this_round, 1);
}

uint32_t 
TcpBbr3::bbr_target_inflight(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    uint32_t bdp = bbr_inflight(tcb, bbr_bw(), 1);   //REPLACE WITH BBR VERSION LATER
    return std::min(bdp, tcb->m_cWnd.Get());
}

uint32_t
TcpBbr3::bbr_inflight_with_headroom()
{
    NS_LOG_FUNCTION (this);
    if (m_inflightHi == std::numeric_limits<int>::max ())
        return std::numeric_limits<int>::max ();

    uint32_t headroom = (m_inflightHi * bbr_inflight_headroom) ;
    headroom = std::max<uint32_t>(headroom, 1);

    return std::max<uint32_t>(m_inflightHi - headroom, m_minPipeCwnd);
}

void
TcpBbr3::bbr_bound_cwnd_for_inflight_model(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (!m_isInitialized)
        return;
    uint32_t cap = std::numeric_limits<int>::max();   
    if (m_state == BBR_PROBE_BW && m_cycleIndex  != BBR_BW_PROBE_CRUISE)
    {
        cap = m_inflightHi;
    } else {
        if (m_state == BBR_PROBE_RTT  || ( m_state == BBR_PROBE_BW && m_cycleIndex  == BBR_BW_PROBE_CRUISE)){
            cap = bbr_inflight_with_headroom();
        }
    }
    cap = std::min(cap, m_inflightLo.Get());
    cap = std::max(cap, m_minPipeCwnd);
    tcb->m_cWnd = std::min(tcb->m_cWnd.Get(), cap); 
}

bool
TcpBbr3::bbr_check_time_to_probe_bw(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    // if (/* m_ecnEligible && m_ecn_in_cycle &&  */!m_lossInCycle && tcb->m_congState == TcpSocketState::CA_OPEN)      // ECN FUNCTIONALITY 
    // {
    //     uint32_t n = static_cast<uint32_t>(log2(m_inflightHi * bbr_ecn_reprobe_gain));
    //     bbr_start_bw_probe_refill(tcb, n);
    //     return true;
    // }
    if (bbr_has_elapsed_in_phase(tcb, m_probeWaitTime) || bbr_is_reno_coexistence_probe_time(tcb))
    {
        bbr_start_bw_probe_refill(tcb, 0);
        return true;
    }
    return false;
}

bool
TcpBbr3::bbr_check_time_to_cruise(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs, DataRate bw)
{
    
    if (rs.m_priorInFlight > bbr_inflight_with_headroom())
        return false;
    return rs.m_priorInFlight <= bbr_inflight(tcb, bw, 1);
}


void
TcpBbr3::bbr_start_bw_probe_up(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context* ctx)
{
	m_ackPhase = BBR_ACKS_PROBE_STARTING;
	m_nextRoundDelivered = m_delivered;
	m_cycleStamp = Simulator::Now();
	bbr_reset_full_bw();
	m_fullBandwidth = ctx->sample_bw;
	bbr_set_cycle_idx(BBR_BW_PROBE_UP);
	bbr_raise_inflight_hi_slope(tcb);
}

void 
TcpBbr3::bbr_start_bw_probe_down()
{
    bbr_reset_congestion_signals();
    m_bwProbeUpCount = std::numeric_limits<int>::max ();
    bbr_pick_probe_wait();
    m_cycleStamp = Simulator::Now();
    m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING;
    m_nextRoundDelivered = m_delivered;
    bbr_set_cycle_idx(BBR_BW_PROBE_DOWN);
}

void
TcpBbr3::bbr_start_bw_probe_refill(Ptr<TcpSocketState> tcb, uint32_t bw_probe_up_rounds)
{
    bbr_reset_lower_bounds();
    m_bwProbeUpRounds = bw_probe_up_rounds;
    m_bwProbeUpAcks = 0;
    m_stoppedRiskyProbe = 0;
    m_cycleStamp = Simulator::Now();
    m_ackPhase = BbrAckPhase_t::BBR_ACKS_REFILLING;
    m_nextRoundDelivered = m_delivered;
    bbr_set_cycle_idx(BBR_BW_PROBE_REFILL);
}

void
TcpBbr3::bbr_start_bw_probe_cruise()
{
    m_cycleStamp = Simulator::Now();
    if (m_inflightLo != std::numeric_limits<int>::max ())
        m_inflightLo = std::min(m_inflightLo.Get(), m_inflightHi.Get());
    bbr_set_cycle_idx(BBR_BW_PROBE_CRUISE);
}

bool
TcpBbr3::bbr_has_elapsed_in_phase(Ptr<TcpSocketState> tcb, Time interval)
{
    return (Simulator::Now() - (m_cycleStamp + interval)) > Seconds(0);
}

bool
TcpBbr3::bbr_is_reno_coexistence_probe_time(Ptr<TcpSocketState> tcb)
{
    int32_t rounds = std::min<int32_t>(bbr_bw_probe_max_rounds, bbr_target_inflight(tcb));
    return m_roundsSinceProbe >= rounds;
}

uint32_t
TcpBbr3::bbr_probe_rtt_cwnd(Ptr<TcpSocketState> tcb)
{
    return std::max<uint32_t>(m_minPipeCwnd, bbr_bdp(tcb, bbr_bw(), 0.5)); // convert to constant later
}

void
TcpBbr3::bbr_update_cycle_phase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs << ctx);
    bool is_bw_probe_done = false;
    uint32_t inflight;
    DataRate bw;

    if (!bbr_full_bw_reached())
        return;

    if (bbr_adapt_upper_bounds(tcb, rs))
         return;
    
    if (m_state != BbrMode_t::BBR_PROBE_BW)
        return;

    //inflight = tcb->m_bytesInFlight; //bbr_packets_in_net_at_edt(tcb, rs.m_priorDelivered); not implemented assuming rs.m_priorInFlight is adequate enough
    inflight = rs.m_priorInFlight;
    bw = bbr_max_bw();
    
    switch(m_cycleIndex)
    {
        case BBR_BW_PROBE_CRUISE:
            if (bbr_check_time_to_probe_bw(tcb))
                return;
            break;

        case BBR_BW_PROBE_REFILL:
            //COUT("BBR_BW_PROBE_REFILL at time " << NOW);
            if (m_roundStart)
            {
                m_bwProbeSamples = 1;
                bbr_start_bw_probe_up(tcb, rs, ctx);
            }     
            break;

        case BBR_BW_PROBE_UP:
            if (m_prevProbeTooHigh && inflight >= m_inflightHi) {
                m_stoppedRiskyProbe = true;
                is_bw_probe_done = true;
            } else {
                if (inflight >= bbr_inflight(tcb, bw, m_pacingGain))   {       //bbr_inflight(tcb, bw, m_pacingGain)) {
                    bbr_reset_full_bw();
                    m_fullBandwidth = ctx->sample_bw;
                    is_bw_probe_done = true;
                } 
                else if (m_fullBandwidthNow) {
                    is_bw_probe_done = true;
                }
            } 
            if (is_bw_probe_done)
            {
                m_prevProbeTooHigh = false;
                bbr_start_bw_probe_down();
            }
            break;
        case BBR_BW_PROBE_DOWN:
            if (bbr_check_time_to_probe_bw(tcb))
                return;		/* already decided state transition */
            if (bbr_check_time_to_cruise(tcb, rs, bw))
                bbr_start_bw_probe_cruise();
            break;
        default:
            break;
    }

}

void 
TcpBbr3::bbr_update_latest_delivery_signals(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs << ctx);

    m_lossRoundStart = false;
    if (rs.m_interval <= Seconds(0) || !rs.m_ackedSacked )
        return; 

    m_bwLatest = std::max<DataRate>(m_bwLatest, ctx->sample_bw);    ////// CHECK THIS
    m_inflightLatest = std::max<uint32_t>(m_inflightLatest, rs.m_delivered);
    //m_inflightLatest =  rs.m_delivered;
    if (rs.m_priorDelivered >= m_lossRoundDelivered) // equivalent to !before
    {
        m_lossRoundDelivered = m_delivered;
        m_lossRoundStart = true;
    }
}

void
TcpBbr3::bbr_set_cycle_idx(uint32_t cycle_idx)
{
    m_cycleIndex = cycle_idx;
    m_tryFastPath = false;
}

void
TcpBbr3::bbr_check_full_bw_reached(const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << rs);

    if (m_fullBandwidthNow || rs.m_isAppLimited)
        return;
    
    /* Check if Bottleneck bandwidth is still growing*/
    if (ctx->sample_bw.GetBitRate() >= m_fullBandwidth.GetBitRate() * 1.25) // REPLACE CONSTANT WITH PARAMETER
    {
        bbr_reset_full_bw();
        m_fullBandwidth = ctx->sample_bw;
        return;
    }
    if (!m_roundStart)
        return;
    m_fullBandwidthCount++;
    m_fullBandwidthNow = m_fullBandwidthCount >= bbr_full_bw_cnt;
    m_fullBwReached |= m_fullBandwidthNow;
}

bool
TcpBbr3::bbr_full_bw_reached()
{
    return m_fullBwReached;
}


void
TcpBbr3::bbr_check_drain(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (m_state == BBR_STARTUP && m_fullBwReached)
    {
        m_state = BBR_DRAIN;	
        tcb->m_ssThresh = bbr_inflight(tcb, bbr_max_bw(), 1);
        bbr_reset_congestion_signals();
    }

    if (m_state == BBR_DRAIN && tcb->m_bytesInFlight <= bbr_inflight(tcb,bbr_max_bw(), 1)){
        m_state = BBR_PROBE_BW;
        bbr_start_bw_probe_down();
    }

}

void
TcpBbr3::UpdateRTprop(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_rtPropExpired = Simulator::Now() > (m_rtPropStamp + m_rtPropFilterLen);
    if (tcb->m_lastRtt >= Seconds(0) && (tcb->m_lastRtt <= m_rtProp || m_rtPropExpired))
    {
        m_rtProp = tcb->m_lastRtt;
        m_rtPropStamp = Simulator::Now();
    }
}

void 
TcpBbr3::bbr_update_min_rtt(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    bool probe_rtt_expired = Simulator::Now() > (m_probeRttMinStamp + bbr_probe_rtt_win);
    if (tcb->m_lastRtt >= Seconds(0) && (tcb->m_lastRtt < m_probeRttMin || (probe_rtt_expired /* && rs.is ack delayed*/ ))) // rs in ns3 does not store min rtt anywhere but the tcb object does
    {
        m_probeRttMin = tcb->m_minRtt;
        m_probeRttMinStamp = Simulator::Now();
    }

    bool min_rtt_expired = Simulator::Now() > (m_rtPropStamp + bbr_min_rtt_win_sec); // some confustion around this 
    if (m_probeRttMin < tcb->m_lastRtt || min_rtt_expired)
    {
        m_rtProp = m_probeRttMin;
        m_rtPropStamp = m_probeRttMinStamp;
    }


    if (/* a check for the minimum rtt param */probe_rtt_expired && !m_idleRestart  && m_state != BbrMode_t::BBR_PROBE_RTT)
    {
        //std::cout << "triggereing probe rtt at time " << Simulator::Now().GetSeconds() << std::endl;
        m_state = BbrMode_t::BBR_PROBE_RTT;
        bbr_save_cwnd(tcb);
        m_probeRttDoneStamp = Seconds(0); 
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING;
        m_nextRoundDelivered = m_delivered;
    }

    if (m_state == BbrMode_t::BBR_PROBE_RTT)
    {
        m_appLimited = (m_delivered + tcb->m_bytesInFlight.Get()) > 0 ;
        if (m_probeRttDoneStamp == Seconds(0) && tcb->m_bytesInFlight <= bbr_probe_rtt_cwnd(tcb) /*bbr_probe_rtt_cwnd(sk) but for now minpipecwnd will do */) 
        {
            m_probeRttDoneStamp = Simulator::Now() + bbr_probe_rtt_mode_ms;
            m_probeRttRoundDone = false;
            m_nextRoundDelivered = m_delivered;
        } 
        else if (m_probeRttDoneStamp != Seconds(0)) 
        {
            if (m_roundStart)
                m_probeRttRoundDone = true;
            if (m_probeRttRoundDone )
                bbr_check_probe_rtt_done(tcb);
        }
    }

    if (rs.m_delivered > 0)
        m_idleRestart = false;
    
}

void 
TcpBbr3::bbr_check_probe_rtt_done(Ptr<TcpSocketState> tcb)
{
    if(!(m_probeRttDoneStamp != Seconds(0) && Simulator::Now() > m_probeRttDoneStamp))
        return;
        
    m_probeRttMinStamp = Simulator::Now(); 
    tcb->m_cWnd = std::max(m_priorCwnd, tcb->m_cWnd.Get()); // prioe restore cwnd
    bbr_exit_probe_rtt(tcb);
}

void 
TcpBbr3::bbr_exit_probe_rtt(Ptr<TcpSocketState> tcb)
{
    bbr_reset_lower_bounds();
    if (bbr_full_bw_reached()){
        m_state = BbrMode_t::BBR_PROBE_BW;
        bbr_start_bw_probe_down();
        bbr_start_bw_probe_cruise();
    } else {
        m_state = BbrMode_t::BBR_STARTUP;
    }
}

void
TcpBbr3::bbr_save_cwnd(Ptr<const TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (tcb->m_congState != TcpSocketState::CA_RECOVERY && m_state != BbrMode_t::BBR_PROBE_RTT)
        m_priorCwnd = tcb->m_cWnd;
    else
        m_priorCwnd = std::max(m_priorCwnd, tcb->m_cWnd.Get());
}

void
TcpBbr3::RestoreCwnd(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    tcb->m_cWnd = std::max(m_priorCwnd, tcb->m_cWnd.Get());
}


void
TcpBbr3::SetSendQuantum(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_sendQuantum = 1 * tcb->m_segmentSize;
}

void
TcpBbr3::UpdateTargetCwnd(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_targetCWnd = bbr_inflight(tcb, bbr_bw() ,m_cWndGain);// + AckAggregationCwnd();
}

uint32_t
TcpBbr3::AckAggregationCwnd()
{
    uint32_t maxAggrBytes; // MaxBW * 0.1 secs
    uint32_t aggrCwndBytes = 0;

    if (m_extraAckedGain && m_fullBwReached)
    {
        maxAggrBytes = bbr_bw().GetBitRate() / (10 * 8);
        aggrCwndBytes = m_extraAckedGain * std::max(m_extraAcked[0], m_extraAcked[1]);
        aggrCwndBytes = std::min(aggrCwndBytes, maxAggrBytes);
    }
    return aggrCwndBytes;
}

void
TcpBbr3::bbr_update_ack_aggregation(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    uint32_t expectedAcked;
    uint32_t extraAck;
    uint32_t epochProp;

    if (!m_extraAckedGain || rs.m_ackedSacked <= 0 || rs.m_delivered < 0)
    {
        return;
    }

    if (m_roundStart)
    {
        m_extraAckedWinRtt = std::min<uint32_t>(31, m_extraAckedWinRtt + 1);
        if (m_extraAckedWinRtt >= m_extraAckedWinRttLength)
        {
            m_extraAckedWinRtt = 0;
            m_extraAckedIdx = m_extraAckedIdx ? 0 : 1;
            m_extraAcked[m_extraAckedIdx] = 0;
        }
    }

    epochProp = Simulator::Now().GetSeconds() - m_ackEpochTime.GetSeconds();
    expectedAcked = bbr_bw().GetBitRate() * epochProp / 8;

    if (m_ackEpochAcked <= expectedAcked ||
        (m_ackEpochAcked + rs.m_ackedSacked >= m_ackEpochAckedResetThresh))
    {
        m_ackEpochAcked = 0;
        m_ackEpochTime = Simulator::Now();
        expectedAcked = 0;
    }

    m_ackEpochAcked = m_ackEpochAcked + rs.m_ackedSacked;
    extraAck = m_ackEpochAcked - expectedAcked;
    extraAck = std::min(extraAck, tcb->m_cWnd.Get());

    if (extraAck > m_extraAcked[m_extraAckedIdx])
    {
        m_extraAcked[m_extraAckedIdx] = extraAck;
    }
}

void
TcpBbr3::bbr_exit_loss_recovery(Ptr<TcpSocketState> tcb) 
{
    tcb->m_cWnd = std::max(tcb->m_cWnd.Get(), m_priorCwnd);
    m_packetConservation = true;
	m_tryFastPath = 0; 
}

bool
TcpBbr3::ModulateCwndForRecovery(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (rs.m_bytesLoss > 0)
    {
        tcb->m_cWnd =
            std::max((int)tcb->m_cWnd.Get() - (int)rs.m_bytesLoss, (int)tcb->m_segmentSize);
    }

    if (m_packetConservation)
    {
        tcb->m_cWnd = std::max(tcb->m_cWnd.Get(), tcb->m_bytesInFlight.Get() + rs.m_ackedSacked);
        return true;
    }
    return false;
}

void
TcpBbr3::bbr_set_cwnd(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    UpdateTargetCwnd(tcb);

    if (!rs.m_ackedSacked)
    {
        goto done;
    }

    if (tcb->m_congState == TcpSocketState::CA_RECOVERY)
    {
        if (ModulateCwndForRecovery(tcb, rs))
        {
            goto done;
        }
    }

    if (!m_packetConservation){
        if (m_fullBwReached)
        {
            tcb->m_cWnd = std::min(tcb->m_cWnd.Get() + (uint32_t)rs.m_ackedSacked, m_targetCWnd);
        }
        else if (tcb->m_cWnd < m_targetCWnd || m_delivered < tcb->m_initialCWnd * tcb->m_segmentSize)
        {
            tcb->m_cWnd = tcb->m_cWnd.Get() + rs.m_ackedSacked;
        }
        tcb->m_cWnd = std::max(tcb->m_cWnd.Get(), m_minPipeCwnd);
    }
done:
    if (m_state == BbrMode_t::BBR_PROBE_RTT)
        tcb->m_cWnd = std::min(tcb->m_cWnd.Get(), bbr_probe_rtt_cwnd(tcb));    
}

uint32_t
TcpBbr3::bbr_update_round_start(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    uint32_t round_delivered = 0;
    m_roundStart = false;

    if (rs.m_interval > Seconds(0) && 
        m_txItemDelivered >= m_nextRoundDelivered) // equivalent to !before
    {
        round_delivered = m_delivered - m_nextRoundDelivered;
        m_nextRoundDelivered = m_delivered; // m delivered is the total number of bytes delivered
        m_roundCount++; // might not need 
        m_roundStart = true;
        m_packetConservation = false;  // might not need
    }
    return round_delivered;
}

DataRate
TcpBbr3::bbr_max_bw()
{
    return std::max<DataRate>(bw_hi[0], bw_hi[1]);
}

void
TcpBbr3::bbr_advance_max_bw_filter()
{
    if(bw_hi[1] == 0)
        return;
    bw_hi[0] = bw_hi[1];
    bw_hi[1] = 0;
}

void
TcpBbr3::bbr_take_max_bw_sample(DataRate bw)
{
    bw_hi[1] = std::max<DataRate>(bw_hi[1], bw);
}

DataRate 
TcpBbr3::bbr_bw()
{
    return std::min<DataRate>(bbr_max_bw(), m_bwLo);
    //return bbr_max_bw();
}

uint32_t 
TcpBbr3::bbr_bdp(Ptr<TcpSocketState> tcb, DataRate bw, double gain)
{
    if (m_rtProp == Time::Max())
        return tcb->m_initialCWnd * tcb->m_segmentSize;
    return ((bw * m_rtProp) * gain) / 8;
}

void
TcpBbr3::bbr_reset_full_bw()
{
    NS_LOG_FUNCTION(this);
    m_fullBandwidth = 0;
    m_fullBandwidthCount = 0;
    m_fullBandwidthNow = false;
}

void
TcpBbr3::bbr_reset_lower_bounds()
{
    m_bwLo = std::numeric_limits<int>::max(); 
    m_inflightLo = std::numeric_limits<int>::max();
}

bool
TcpBbr3::bbr_adapt_upper_bounds(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION (this << tcb << rs);
    if (m_ackPhase == BbrAckPhase_t::BBR_ACKS_PROBE_STARTING && m_roundStart)
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_FEEDBACK;

    if (m_ackPhase == BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING && m_roundStart) 
    {
        m_bwProbeSamples = 0;
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_INIT;
        if (m_state == BbrMode_t::BBR_PROBE_BW && !rs.m_isAppLimited)
            bbr_advance_max_bw_filter();
        
        if (m_state == BbrMode_t::BBR_PROBE_BW && m_stoppedRiskyProbe && !m_prevProbeTooHigh)
        {
            bbr_start_bw_probe_refill(tcb, 0);  
            return true;
        }
    }

    if (bbr_is_inflight_too_high(tcb, rs))
    {   
        if (m_bwProbeSamples)
                bbr_handle_inflight_too_high(tcb, rs, false);
    } else {

        if (m_inflightHi == std::numeric_limits<int>::max ())
            return false;

        if (tcb->m_bytesInFlight > m_inflightHi) 
            m_inflightHi = tcb->m_bytesInFlight;
        
        if (m_state == BBR_PROBE_BW && m_cycleIndex == BBR_BW_PROBE_UP){}
			bbr_probe_inflight_hi_upward(tcb, rs);
    }
    return false;
}

void 
TcpBbr3::bbr_adapt_lower_bounds(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs)
{
    // uint32_t ecn_inflight_lo = 0;
    
    if (bbr_is_probing_bandwidth(tcb))
        return;
    
    // ECN response
    // if (m_ecnInRound)
    // {
    //     bbr_init_lower_bounds(tcb, false);
    //     bbr_ecn_lower_bounds(tcb, &ecn_inflight_lo);
    // }

    // LOSS RESPONSE
    if (m_lossInRound)  
    {
        bbr_init_lower_bounds(tcb, true);
        bbr_loss_lower_bounds(tcb);
        //std::cout << "Inflight Lo: " << m_inflightLo.Get() << std::endl;
    }

    // m_inflightLo = min(m_inflightLo, ecn_inflight_lo);  afetr ecn is implemented
	m_bwLo = std::max<DataRate>(DataRate(1), m_bwLo); 
}

void
TcpBbr3::bbr_reset_congestion_signals()
{
    m_lossInRound = false;
    m_ecnInRound = false;  
    //m_lossInCycle = false;
    m_ecn_in_cycle = false;
    m_bwLatest = 0;
    m_inflightLatest = 0;
}

void
TcpBbr3::bbr_calculate_bw_sample(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs << ctx);

    if (rs.m_interval == Seconds(0) || rs.m_delivered == 0 || rs.m_deliveryRate == 0)
    return;
	ctx->sample_bw = rs.m_deliveryRate;
}

void 
TcpBbr3::bbr_pick_probe_wait()
{
    m_roundsSinceProbe = (uint)m_uv->GetValue(0, 2);
    m_probeWaitTime = Seconds(2) + MilliSeconds(m_uv->GetValue(0, 1000));
    COUT(m_probeWaitTime.GetSeconds());
}


void
TcpBbr3::bbr_update_congestion_signals(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (rs.m_interval <= Seconds(0) || !rs.m_ackedSacked)
        return;

    DataRate bw = ctx->sample_bw;

    if (!rs.m_isAppLimited || bw >= bbr_max_bw())   
        bbr_take_max_bw_sample(bw);
        
    m_lossInRound |= (rs.m_bytesLoss > 0);

    //m_bwLatest = std::max<DataRate>(m_bwLatest, bw);
    //m_inflightLatest = std::max<uint32_t>(m_inflightLatest, rs.m_delivered);     

    if (!m_lossRoundStart)
        return;

    bbr_adapt_lower_bounds(tcb, rs);

    m_lossInRound = 0;
    // ecn_in_round = false; // ECN SUPPORT 
}

void
TcpBbr3::bbr_check_loss_too_high_in_startup(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (m_state != BbrMode_t::BBR_STARTUP)
      return;
    
    if (bbr_full_bw_reached())
		return;

    if (rs.m_bytesLoss > 0 && m_lossEventsInRound < 15)
        m_lossEventsInRound++;
    
    // if (m_state == BbrMode_t::BBR_STARTUP){
    //     std::cout << "start" << std::endl;
    //     std::cout << "Loss round start " << m_lossRoundStart << std::endl;
    //     std::cout << "tcb->m_congState == TcpSocketState::CA_RECOVERY " << tcb->m_congState  << std::endl;
    //     std::cout << "m_lossEventsInRound >= bbr_full_loss_cnt "<< m_lossEventsInRound << " " << bbr_full_loss_cnt << std::endl;
    //     std::cout << "bbr_is_inflight_too_high(tcb, rs)" << bbr_is_inflight_too_high(tcb, rs) <<" " << rs.m_bytesLoss << " " << (tcb->m_bytesInFlight.Get() / 100 * 2) << std::endl;

    // }

    if (m_lossRoundStart 
        && tcb->m_congState == TcpSocketState::CA_RECOVERY 
        && m_lossEventsInRound >= bbr_full_loss_cnt 
        && bbr_is_inflight_too_high(tcb, rs)
    )
    {
        bbr_handle_queue_too_high_in_startup(tcb);
        return;
    }

    if(m_lossRoundStart)
        m_lossEventsInRound = 0;
}

void 
TcpBbr3::bbr_advance_latest_delivery_signals(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
{
    	if (m_lossRoundStart) {
		m_bwLatest = ctx->sample_bw;
		m_inflightLatest = rs.m_delivered;
	}
}

void 
TcpBbr3::bbr_handle_queue_too_high_in_startup(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_fullBwReached = true;
    uint32_t bdp = bbr_inflight(tcb, bbr_max_bw(), 1);
    //m_inflightHi = std::max<uint32_t>(bdp, m_inflightLatest);
    m_inflightHi = bdp; 
}


void
TcpBbr3::bbr_update_model(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    bbr_update_congestion_signals(tcb, rs, ctx);
    bbr_update_ack_aggregation(tcb, rs);
    bbr_check_loss_too_high_in_startup(tcb, rs);
    bbr_check_full_bw_reached(rs, ctx);
    bbr_check_drain(tcb);
    bbr_update_cycle_phase(tcb, rs, ctx);                 
    bbr_update_min_rtt(tcb, rs);
}

uint32_t
TcpBbr3::GetBbrState()
{
    NS_LOG_FUNCTION(this);
    return m_state;
}

double
TcpBbr3::GetCwndGain()
{
    NS_LOG_FUNCTION(this);
    return m_cWndGain;
}

double
TcpBbr3::GetPacingGain()
{
    NS_LOG_FUNCTION(this);
    return m_pacingGain;
}

std::string
TcpBbr3::GetName() const
{
    return "TcpBbr3";
}

bool
TcpBbr3::HasCongControl() const
{
    NS_LOG_FUNCTION(this);
    return true;
}



void
TcpBbr3::bbr_update_gains()
{
    NS_LOG_FUNCTION(this);
	switch (m_state) {
        case BBR_STARTUP:
            m_pacingGain = 2.89;
            m_cWndGain	 = 2.89;
            break;
        case BBR_DRAIN:
            m_pacingGain = 1.0 * 1000.0 / 2885.0;  /* slow, to drain */
            m_cWndGain	 = 2;  /* keep cwnd */
            break;
        case BBR_PROBE_BW:
            m_pacingGain = PACING_GAIN_CYCLE[m_cycleIndex];
            m_cWndGain	 = 2;
            if (bbr_bw_probe_cwnd_gain !=0  && m_cycleIndex == BBR_BW_PROBE_UP)
                m_cWndGain += 1 * bbr_bw_probe_cwnd_gain / 4;
            break;
        case BBR_PROBE_RTT:
            m_pacingGain = 1;
            m_cWndGain	 = 1;
            break;
        default:
            break;
	}
}


void
TcpBbr3::bbr_main(Ptr<TcpSocketState> tcb,
                    const TcpRateOps::TcpRateConnection& rc,
                    const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    //if (rs.m_bytesLoss > 0)
         //std::cout << "LOSS " << rs.m_bytesLoss << " AT TIME  "  << Now().GetSeconds() << " "<< BbrCycleName[m_cycleIndex] << std::endl;

    struct bbr_context ctx = { 0 };
    
    m_delivered = rc.m_delivered;
    m_txItemDelivered = rc.m_txItemDelivered;

    //std::cout << "rs D " << rs.m_delivered << "  m_delivered " << m_delivered << std::endl;
    maxBw = bbr_max_bw().GetBitRate();
    //std::cout << "inflight lo " << m_inflightHi << " at time  " << Now().GetSeconds() << std::endl;

    bbr_update_round_start(tcb, rs);
  
    bbr_calculate_bw_sample(tcb, rs, &ctx);
    
    bbr_update_latest_delivery_signals(tcb, rs, &ctx);
    bbr_update_model(tcb, rs, &ctx);

    bbr_update_gains();

    wildcard = rs.m_bytesLoss;
    

    bbr_set_pacing_rate(tcb, m_pacingGain);
    SetSendQuantum(tcb);
    bbr_set_cwnd(tcb, rs);


    bbr_bound_cwnd_for_inflight_model(tcb);

    bbr_advance_latest_delivery_signals(tcb, rs, &ctx);

	m_prevState = tcb->m_congState;
}

void
TcpBbr3::CongestionStateSet(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
    NS_LOG_FUNCTION(this << tcb << newState);
    if (newState == TcpSocketState::CA_OPEN && !m_isInitialized) {
        if (Now().GetSeconds() > 5){
            startedAfter = true;
        }
        m_rtProp = tcb->m_lastRtt.Get() != Time::Max() ? tcb->m_lastRtt.Get() : Time::Max();
        m_probeRttMinStamp = Simulator::Now();
        m_rtPropStamp = Simulator::Now();
        m_priorCwnd = tcb->m_cWnd;
        tcb->m_ssThresh = tcb->m_initialSsThresh;
        m_targetCWnd = tcb->m_cWnd;
        m_minPipeCwnd = 4 * tcb->m_segmentSize;
        m_sendQuantum = 1 * tcb->m_segmentSize;
        //bbr_init_lower_bounds(tcb, false); // ?? causes cwnd to be stuck ???

        InitRoundCounting();
        InitFullPipe();
        m_state = BbrMode_t::BBR_STARTUP;
        m_pacingGain = m_highGain;
        m_cWndGain = m_highGain;
        InitPacingRate(tcb);
        
        m_isInitialized = true;
    } else if (newState == TcpSocketState::CA_LOSS) {
        bbr_save_cwnd(tcb);
        m_prevState = TcpSocketState::CA_LOSS;
        m_roundStart = true;
        bbr_reset_full_bw();
        if (bbr_is_probing_bandwidth(tcb) && m_inflightLo == std::numeric_limits<int>::max ())
        {
          m_inflightLo = m_priorCwnd;
        }
    } else if (m_prevState == TcpSocketState::CA_LOSS && newState == TcpSocketState::CA_LOSS) {
		bbr_exit_loss_recovery(tcb);
	}  
    //else if (newState == TcpSocketState::CA_RECOVERY) {
    //     bbr_save_cwnd(tcb);
    //     tcb->m_cWnd = tcb->m_bytesInFlight.Get() + std::max(tcb->m_lastAckedSackedBytes, tcb->m_segmentSize);
         //m_packetConservation = true;
    //}
}

void 
TcpBbr3::bbr_inflight_hi_from_lost_skb(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{

}



void 
TcpBbr3::bbr_note_loss(Ptr<TcpSocketState> tcb, uint32_t delivered)
{
    if (!m_lossInRound)  /* first loss in this round trip? */
		m_lossRoundDelivered = delivered;  /* set round trip */
	m_lossInRound = true;
}

void
TcpBbr3::CwndEvent(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
    NS_LOG_FUNCTION(this << tcb << event);
    if (event == TcpSocketState::CA_EVENT_COMPLETE_CWR)
    {
        NS_LOG_DEBUG("CwndEvent triggered to CA_EVENT_COMPLETE_CWR :: " << event);
        m_packetConservation = false;
        //RestoreCwnd(tcb);
    }
    else if (event == TcpSocketState::CA_EVENT_TX_START && m_appLimited)
    {
        NS_LOG_DEBUG("CwndEvent triggered to CA_EVENT_TX_START :: " << event);
        m_idleRestart = true;
        m_ackEpochTime = Simulator::Now();
        m_ackEpochAcked = 0;
        if (m_state == BbrMode_t::BBR_PROBE_BW)
        {
            bbr_set_pacing_rate(tcb, 1);
        }
        else if (m_state == BbrMode_t::BBR_PROBE_RTT)
        {
            bbr_check_probe_rtt_done(tcb);
        }
    }
}

uint32_t
TcpBbr3::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    NS_LOG_FUNCTION(this << tcb << bytesInFlight);
    bbr_save_cwnd(tcb);
    
    return tcb->m_ssThresh;
}

Ptr<TcpCongestionOps>
TcpBbr3::Fork()
{
    return CopyObject<TcpBbr3>(this);
}

} // namespace ns3
