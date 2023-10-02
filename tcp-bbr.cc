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

#include "tcp-bbr.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TcpBbr");
NS_OBJECT_ENSURE_REGISTERED(TcpBbr);

const double TcpBbr::PACING_GAIN_CYCLE[] = {5.0 / 4, 91.0 / 100, 1, 1 , 1, 1, 1, 1};

TypeId
TcpBbr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TcpBbr")
            .SetParent<TcpCongestionOps>()
            .AddConstructor<TcpBbr>()
            .SetGroupName("Internet")
            .AddAttribute("Stream",
                          "Random number stream (default is set to 4 to align with Linux results)",
                          UintegerValue(4),
                          MakeUintegerAccessor(&TcpBbr::SetStream),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("HighGain",
                          "Value of high gain",
                          DoubleValue(2.89),
                          MakeDoubleAccessor(&TcpBbr::m_highGain),
                          MakeDoubleChecker<double>())
            .AddAttribute("BwWindowLength",
                          "Length of bandwidth windowed filter",
                          UintegerValue(10),
                          MakeUintegerAccessor(&TcpBbr::m_bandwidthWindowLength),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("RttWindowLength",
                          "Length of RTT windowed filter",
                          TimeValue(Seconds(10)),
                          MakeTimeAccessor(&TcpBbr::m_rtPropFilterLen),
                          MakeTimeChecker())
            .AddAttribute("ProbeRttDuration",
                          "Time to be spent in PROBE_RTT phase",
                          TimeValue(MilliSeconds(200)),
                          MakeTimeAccessor(&TcpBbr::m_probeRttDuration),
                          MakeTimeChecker())
            .AddAttribute("ExtraAckedRttWindowLength",
                          "Window length of extra acked window",
                          UintegerValue(5),
                          MakeUintegerAccessor(&TcpBbr::m_extraAckedWinRttLength),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute(
                "AckEpochAckedResetThresh",
                "Max allowed val for m_ackEpochAcked, after which sampling epoch is reset",
                UintegerValue(1 << 12),
                MakeUintegerAccessor(&TcpBbr::m_ackEpochAckedResetThresh),
                MakeUintegerChecker<uint32_t>());
    return tid;
}

TcpBbr::TcpBbr()
    : TcpCongestionOps()
{
    NS_LOG_FUNCTION(this);
    m_uv = CreateObject<UniformRandomVariable>();
}

TcpBbr::TcpBbr(const TcpBbr& sock)
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

const char* const TcpBbr::BbrModeName[BBR_PROBE_RTT + 1] = {
    "BBR_STARTUP",
    "BBR_DRAIN",
    "BBR_PROBE_BW",
    "BBR_PROBE_RTT",
};

void
TcpBbr::SetStream(uint32_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_uv->SetStream(stream);
}

void
TcpBbr::InitRoundCounting()
{
    NS_LOG_FUNCTION(this);
    m_nextRoundDelivered = 0;
    m_roundStart = false;
    m_roundCount = 0;
}

void
TcpBbr::InitFullPipe()
{
    NS_LOG_FUNCTION(this);
    m_fullBwReached = false;
    m_fullBandwidth = 0;
    m_fullBandwidthCount = 0;
}

void
TcpBbr::InitPacingRate(Ptr<TcpSocketState> tcb)
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
    // m_maxBwFilter = MaxBandwidthFilter_t(m_bandwidthWindowLength, DataRate(tcb->m_cWnd * 8 / rtt.GetSeconds()), 0);
    bbr_take_max_bw_sample(nominalBandwidth);
}

bool
TcpBbr::bbr_is_probing_bandwidth(Ptr<TcpSocketState> tcb)
{
    // std::cout << "bbr_is_probing_bandwidth" << std::endl;
    return (m_state == BBR_STARTUP) ||
    (m_state == BBR_PROBE_BW &&
        (m_cycleIndex == BBR_BW_PROBE_REFILL ||
        m_cycleIndex == BBR_BW_PROBE_UP));
}

void
TcpBbr::EnterStartup()
{
    NS_LOG_FUNCTION(this);
    SetBbrState(BbrMode_t::BBR_STARTUP);
    m_pacingGain = m_highGain;
    m_cWndGain = m_highGain;
}

void
TcpBbr::HandleRestartFromIdle(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (tcb->m_bytesInFlight.Get() == 0U && rs.m_isAppLimited)
    {
        m_idleRestart = true;
        if (m_state == BbrMode_t::BBR_PROBE_BW)
        {
            SetPacingRate(tcb, 1);
        }
    }
}

void
TcpBbr::SetPacingRate(Ptr<TcpSocketState> tcb, double gain)
{
    NS_LOG_FUNCTION(this << tcb << gain);
    DataRate rate(gain * bbr_bw().GetBitRate());
    rate = std::min(rate, tcb->m_maxPacingRate);

    if (!m_hasSeenRtt && tcb->m_minRtt != Time::Max())
        InitPacingRate(tcb);
    
    if (m_fullBwReached || rate > tcb->m_pacingRate)
        tcb->m_pacingRate = rate;
}

void 
TcpBbr::bbr_init_lower_bounds(Ptr<TcpSocketState> tcb, bool init_bw)
{
    //std::cout << "bbr_init_lower_bounds" << std::endl;
    if (init_bw && m_bwLo == 0)
        m_bwLo = bbr_max_bw();
    if (m_inflightLo == std::numeric_limits<int>::max ())
        m_inflightLo = tcb->m_cWnd.Get();   
    std::cout << "m_inflightLo " << m_inflightLo << std::endl;
    std::cout << "m_bwLo " << m_bwLo << std::endl;
}

void
TcpBbr::bbr_loss_lower_bounds(Ptr<TcpSocketState> tcb, DataRate *bw, uint32_t *inflight)
{
    //std::cout << "bbr_loss_lower_bounds" << std::endl;
    *bw =  std::max<DataRate>(m_bwLatest, DataRate(m_bwLo.GetBitRate() * (1 - bbr_beta)));
    *inflight = std::max<uint32_t>(m_inflightLatest, m_inflightLo * (1 - bbr_beta));
}

uint32_t
TcpBbr::InFlight(Ptr<TcpSocketState> tcb, DataRate bw, double gain)
{
    NS_LOG_FUNCTION(this << tcb << gain);
    if (m_rtProp == Time::Max())
    {
        return tcb->m_initialCWnd * tcb->m_segmentSize;
    }
    double quanta = 3 * m_sendQuantum;
    double estimatedBdp = bbr_bdp(tcb, bw, gain);
    //std::cout <<" INFLGIHT >>>> " <<  (estimatedBdp) + quanta + (2 * tcb->m_segmentSize) << "    GAIN >>> " << gain << std::endl;
    if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == 0)
    {
        return (estimatedBdp) + quanta + (2 * tcb->m_segmentSize);
    }
    return (estimatedBdp) + quanta;
}

bool 
TcpBbr::bbr_is_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    //std::cout << "bbr_is_inflight_too_high" << std::endl;
    if (rs.m_bytesLoss > 0 && tcb->m_bytesInFlight > 0)
    {
        double loss_thresh = rs.m_priorInFlight * bbr_loss_thresh;
        if (rs.m_bytesLoss > (tcb->m_bytesInFlight * static_cast<uint32_t>(loss_thresh)))

            return true;
        
    }
    // if (rs.delivered_ce > 0 && rs.m_delivered > 0 && m_ecnEligible /* && bbr_param(sk, ecn_thresh)*/)    //ECN FUNCTIONALITY 
    // { 
    //     double ecn_thresh = rs.m_delivered * bbr_ecn_thresh;
    //     if (rs.delivered_ce > static_cast<uint32_t>(ecn_thresh))
    //     {
    //         return true;
    //     }
    // }
    return false;
}

void 
TcpBbr::bbr_handle_inflight_too_high(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    std::cout << "bbr_handle_inflight_too_high" << std::endl;
    m_prevProbeTooHigh = true;
    m_bwProbeSamples = 0;
    if (!rs.m_isAppLimited)
        m_inflightHi = std::max(tcb->m_bytesInFlight.Get(), static_cast<uint32_t>(bbr_target_inflight(tcb) * static_cast<double>(bbr_beta)));
        //addapt to ecn later 
    if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex == BBR_BW_PROBE_UP)
        bbr_start_bw_probe_down();
     
}

void
TcpBbr::bbr_probe_inflight_hi_upward(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (!(rs.m_priorInFlight >= m_priorCwnd) || tcb->m_cWnd < m_inflightHi)
    {
        m_bwProbeUpAcks = 0; 
        return;
    }
        
    // rs.m_delivered = 0;
    m_bwProbeUpAcks += rs.m_ackedSacked;
    if (m_bwProbeUpAcks >= m_bwProbeUpCount && m_bwProbeUpCount!= 0)
    {
        uint32_t delta = m_bwProbeUpAcks / m_bwProbeUpCount;
        m_bwProbeUpAcks -= delta * m_bwProbeUpCount;
        m_inflightHi += delta;
        m_tryFastPath = false;
    }
    if (m_roundStart)
        bbr_raise_inflight_hi_slope(tcb);

}

void 
TcpBbr::bbr_raise_inflight_hi_slope(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    uint32_t growth_this_round = 1 << m_bwProbeUpRounds;
    m_bwProbeUpRounds = std::min<uint32_t>(m_bwProbeUpRounds+1, 30);
    m_bwProbeUpCount = std::max<uint32_t> (tcb->m_initialCWnd / growth_this_round, 1);
}

uint32_t 
TcpBbr::bbr_target_inflight(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    uint32_t bdp = InFlight(tcb, bbr_bw(), 1);   //REPLACE WITH BBR VERSION LATER
    return std::max(bdp, tcb->m_cWnd.Get());
}

uint32_t
TcpBbr::bbr_inflight_with_headroom(Ptr<TcpSocketState> tcb)
{
    //std::cout << "bbr_inflight_with_headroom" << std::endl;
    if (m_inflightHi == std::numeric_limits<int>::max ())
        return std::numeric_limits<int>::max ();

    uint32_t headroom = (m_inflightHi * bbr_inflight_headroom) ;
    headroom = std::max<uint32_t>(headroom, 1);
    return std::max<uint32_t>(m_inflightHi - headroom, m_minPipeCwnd);
}

void
TcpBbr::bbr_bound_cwnd_for_inflight_model(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (!m_isInitialized)
        return;
    uint32_t cap = std::numeric_limits<int>::max ();   
    if (m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex  != BBR_BW_PROBE_CRUISE)
    {
        cap = m_inflightHi;
    } else {
        // std::cout << "cap " << cap << std::endl;
        if (m_state == BbrMode_t::BBR_PROBE_RTT  || ( m_state == BbrMode_t::BBR_PROBE_BW && m_cycleIndex  == BBR_BW_PROBE_CRUISE))
            cap = bbr_inflight_with_headroom(tcb);
    }
    cap = std::min(cap, m_inflightLo);
    //std::cout << "inflight lo " << m_inflightLo << std::endl;
    cap = std::max(cap, m_minPipeCwnd);
    // std::cout << "cwnd before bind " << tcb->m_cWnd << std::endl;
    tcb->m_cWnd = std::min(tcb->m_cWnd.Get(), cap); 
    //std::cout << "cwnd after bind " << tcb->m_cWnd << std::endl;
}

void
TcpBbr::AdvanceCyclePhase()
{
    NS_LOG_FUNCTION(this);
    m_cycleStamp = Simulator::Now();
    m_cycleIndex = (m_cycleIndex + 1) % GAIN_CYCLE_LENGTH;
    m_pacingGain = PACING_GAIN_CYCLE[m_cycleIndex];
}

bool
TcpBbr::IsNextCyclePhase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    bool isFullLength = (Simulator::Now() - m_cycleStamp) > m_rtProp;
    if (m_pacingGain == 1)
    {
        return isFullLength;
    }
    else if (m_pacingGain > 1)
    {
        return isFullLength &&
               (rs.m_bytesLoss > 0 || rs.m_priorInFlight >= InFlight(tcb, bbr_bw(), m_pacingGain));
    }
    else
    {
        return isFullLength || rs.m_priorInFlight <= InFlight(tcb,bbr_bw(), 1);
    }
}

void
TcpBbr::CheckCyclePhase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (m_state == BbrMode_t::BBR_PROBE_BW && IsNextCyclePhase(tcb, rs))
    {
        AdvanceCyclePhase();
    }
}

bool
TcpBbr::bbr_check_time_to_probe_bw(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    //std::cout << "bbr_check_time_to_probe_bw" << std::endl;                   

    // if (m_ecnEligible && m_ecn_in_cycle && !m_lossInCycle && tcb->m_congState == TcpSocketState::CA_OPEN)      // ECN FUNCTIONALITY 
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
TcpBbr::bbr_check_time_to_cruise(Ptr<TcpSocketState> tcb, uint32_t inflight, DataRate bw)
{
    //std::cout << "bbr_check_time_to_cruise" << std::endl;
    if (inflight > bbr_inflight_with_headroom(tcb))
        return false;
    return inflight <= InFlight(tcb, bw, 1); // replace with the bbr version later
}


void
TcpBbr::bbr_start_bw_probe_up(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context* ctx)
{
    //std::cout << "bbr_start_bw_probe_up" << std::endl;
	m_ackPhase = BBR_ACKS_PROBE_STARTING;
	m_nextRoundDelivered = m_delivered;
	m_cycleStamp = Simulator::Now();
	bbr_reset_full_bw();
	m_fullBandwidth = ctx->sample_bw;
	bbr_set_cycle_idx(BBR_BW_PROBE_UP);
	bbr_raise_inflight_hi_slope(tcb);
}

void 
TcpBbr::bbr_start_bw_probe_down()
{
    //std::cout << "bbr_start_bw_probe_down" << std::endl;
    bbr_reset_congestion_signals();
    m_bwProbeUpCount = 0;
    bbr_pick_probe_wait();
    m_cycleStamp = Simulator::Now();
    m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING;
    m_nextRoundDelivered = m_delivered;
    bbr_set_cycle_idx(BBR_BW_PROBE_DOWN);
}

void
TcpBbr::bbr_start_bw_probe_refill(Ptr<TcpSocketState> tcb, uint32_t bw_probe_up_rounds)
{
    //std::cout << "bbr_start_bw_probe_refill" << std::endl;
    bbr_reset_lower_bounds();
    m_bwProbeUpRounds = bw_probe_up_rounds;
    m_bwProbeUpAcks = 0;
    m_stoppedRiskyProbe = false;
    m_ackPhase = BbrAckPhase_t::BBR_ACKS_REFILLING;
    m_nextRoundDelivered = m_delivered;
    bbr_set_cycle_idx(BBR_BW_PROBE_REFILL);
}

void
TcpBbr::bbr_start_bw_probe_cruise()
{
    //std::cout << "bbr_start_bw_probe_cruise" << std::endl;
    if (m_inflightLo != std::numeric_limits<int>::max ())
        m_inflightLo = std::min<uint32_t>(m_inflightLo, m_inflightHi);
    bbr_set_cycle_idx(BBR_BW_PROBE_CRUISE);
}

bool
TcpBbr::bbr_has_elapsed_in_phase(Ptr<TcpSocketState> tcb, Time interval)
{
    //std::cout << "bbr_has_elapsed_in_phase" << std::endl;
    return (Simulator::Now() - (m_cycleStamp + interval)) > Seconds(0);
}

bool
TcpBbr::bbr_is_reno_coexistence_probe_time(Ptr<TcpSocketState> tcb)
{
    //std::cout << "bbr_is_reno_coexistence_probe_time" << std::endl;
    int32_t rounds = std::min<int32_t>(bbr_bw_probe_max_rounds, bbr_target_inflight(tcb));
    return m_roundsSinceProbe >= rounds;
}

void
TcpBbr::bbr_update_cycle_phase(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    //std::cout << "bbr_update_cycle_phase" << std::endl;
    bool is_bw_probe_done = false;
    uint32_t inflight;
    DataRate bw;
    if (!bbr_full_bw_reached())
        return;

    if (bbr_adapt_upper_bounds(tcb, rs))
         return;
    
    if (m_state != BbrMode_t::BBR_PROBE_BW)
        return;

    inflight = tcb->m_bytesInFlight; //bbr_packets_in_net_at_edt(tcb, rs.m_priorDelivered); not implemented assuming rs.m_priorInFlight is adequate enough
    bw = bbr_max_bw();
    //std::cout << "CYCLE check" << m_cycleIndex << std::endl;
    switch(m_cycleIndex)
    {
        case BBR_BW_PROBE_CRUISE:
            //std::cout << "case cruise   " << std::endl;
            if (bbr_check_time_to_probe_bw(tcb))
                return;
            
                
            break;

        case BBR_BW_PROBE_REFILL:
            //std::cout << "case refill   " << std::endl;
            if (m_roundStart)
            {
                m_bwProbeSamples = true;
                bbr_start_bw_probe_up(tcb, rs,  ctx);
            }   
            break;

        case BBR_BW_PROBE_UP:
            //std::cout << "case probe up  " << std::endl;
            //std::cout << "inflight  >= m_inflightHi = " << (inflight >= m_inflightHi) << std::endl;
            //std::cout << "m_prevProbeTooHigh = " << m_prevProbeTooHigh << std::endl;

            if (m_prevProbeTooHigh && inflight  >= m_inflightHi)
            {
                m_stoppedRiskyProbe = true;
                is_bw_probe_done = true;
            } else {
                if (tcb->m_cWnd >= m_inflightHi)
                {
                    //std::cout << "cwnd >= m_inflightHi" << tcb->m_cWnd <<  " "<< m_inflightHi << std::endl;

                    bbr_reset_full_bw();
				    m_fullBandwidth = ctx->sample_bw;
                } else if (m_fullBandwidthNow) 
                {
                    //std::cout << "PROBE BW IS DONE" << std::endl;
                    is_bw_probe_done = true;
                }
            }
            //std::cout << "full bw now >>> " << m_fullBandwidthNow << std::endl;
            //std::cout << "is_bw_probe_done = " << is_bw_probe_done << std::endl;
            if (is_bw_probe_done) 
            {
                m_prevProbeTooHigh = false;
                bbr_start_bw_probe_down();
            }
            break;
        case BBR_BW_PROBE_DOWN:
            //std::cout << "case probe down  " << std::endl;
            if (bbr_check_time_to_probe_bw(tcb))
                return;		/* already decided state transition */
            if (bbr_check_time_to_cruise(tcb, inflight, bw))
                bbr_start_bw_probe_cruise();
            break;
        default:
            break;
    }

}

void 
TcpBbr::bbr_update_latest_delivery_signals(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs << ctx);
    //std::cout << "bbr_update_latest_delivery_signals" << std::endl;

    m_lossRoundStart = false;
    if (rs.m_interval <= Seconds(0) || !rs.m_ackedSacked )
        return; 

    m_bwLatest = std::max<DataRate>(m_bwLatest, ctx->sample_bw);    ////// CHECK THIS
    m_inflightLatest = std::max<uint32_t>(m_inflightLatest, rs.m_delivered);
    
    if (rs.m_priorDelivered >= m_lossRoundDelivered) // equivalent to !before
    {
        m_lossRoundDelivered = m_delivered;
        m_lossRoundStart = true;
    }
}

void
TcpBbr::bbr_set_cycle_idx(uint32_t cycle_idx)
{
    //std::cout << "bbr_set_cycle_idx" << std::endl;
    m_cycleIndex = cycle_idx;
    m_tryFastPath = false;
}

void
TcpBbr::bbr_check_full_bw_reached(const TcpRateOps::TcpRateSample& rs, const struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << rs);
    
    uint32_t full_cnt = 3;//REPLACE CONSTANT WITH PARAMETER 

    if (m_fullBandwidthNow || rs.m_isAppLimited)
        return;
    
    /* Check if Bottleneck bandwidth is still growing*/
    if (ctx->sample_bw.GetBitRate() >= bbr_bw().GetBitRate() * 1.25) // REPLACE CONSTANT WITH PARAMETER
    {
        bbr_reset_full_bw();
        m_fullBandwidth = ctx->sample_bw;
        return;
    }

    if (!m_roundStart)
        return;
    

    m_fullBandwidthCount++;
    m_fullBandwidthNow = m_fullBandwidthCount >= full_cnt;
    m_fullBwReached |= m_fullBandwidthNow;
}

bool
TcpBbr::bbr_full_bw_reached()
{
    //std::cout << "bbr_full_bw_reached" << std::endl;
    return m_fullBwReached;
}

void
TcpBbr::EnterDrain()
{
    NS_LOG_FUNCTION(this);
    SetBbrState(BBR_DRAIN);
    m_pacingGain = 1.0 / m_highGain;
    m_cWndGain = m_highGain;
}

void
TcpBbr::EnterProbeBW()
{
    NS_LOG_FUNCTION(this);
    std::cout << "ENTER PROBE BW" << std::endl;
    SetBbrState(BBR_PROBE_BW);
    m_pacingGain = 1;
    m_cWndGain = 2;
    m_cycleIndex = GAIN_CYCLE_LENGTH - 1 - (int)m_uv->GetValue(0, 6);
    AdvanceCyclePhase();
}

void
TcpBbr::bbr_check_drain(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (m_state == BBR_STARTUP && m_fullBwReached)
    {
        m_state = BBR_DRAIN;	
        tcb->m_ssThresh = InFlight(tcb, bbr_max_bw(), 1);
        bbr_reset_congestion_signals();
    }

    if (m_state == BBR_DRAIN && tcb->m_bytesInFlight <= InFlight(tcb,bbr_max_bw(), 1)){
        m_state = BBR_PROBE_BW;
        bbr_start_bw_probe_down();
    }

}

void
TcpBbr::UpdateRTprop(Ptr<TcpSocketState> tcb)
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
TcpBbr::bbr_update_min_rtt(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    // std::cout << "bbr_update_min_rtt" << std::endl;
    NS_LOG_FUNCTION(this << tcb << rs);

    bool probe_rtt_expired = Simulator::Now() > (m_probeRttMinStamp + bbr_probe_rtt_win);
    if (tcb->m_minRtt >= Seconds(0) && (tcb->m_minRtt < m_probeRttMin || (probe_rtt_expired /* && rs.is ack delayed*/ ))) // rs in ns3 does not store min rtt anywhere but the tcb object does
    {
        m_probeRttMin = tcb->m_minRtt;
        m_probeRttMinStamp = Simulator::Now();
    }

    bool min_rtt_expired = Simulator::Now() > (m_rtPropStamp + bbr_min_rtt_win_sec); // some confustion around this 
    if (m_probeRttMin <= tcb->m_lastRtt || min_rtt_expired)
    {
        m_rtProp = m_probeRttMin;
        m_rtPropStamp = m_probeRttMinStamp;
    }


    if (/* a check for the minimum rtt param */probe_rtt_expired && !m_idleRestart  && m_state != BbrMode_t::BBR_PROBE_RTT)
    {
        m_state = BbrMode_t::BBR_PROBE_RTT;
        bbr_save_cwnd(tcb);
        m_probeRttDoneStamp = Seconds(0); 
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING;
        m_nextRoundDelivered = m_delivered;
    }

    if (m_state == BbrMode_t::BBR_PROBE_RTT)
    {
        m_appLimited = (m_delivered + tcb->m_bytesInFlight.Get()) > 0 ;
        if (m_probeRttDoneStamp == Seconds(0) && tcb->m_bytesInFlight <= m_minPipeCwnd /*bbr_probe_rtt_cwnd(sk) but for now minpipecwnd will do */) 
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

    if (rs.m_delivered > (int)m_nextRoundDelivered)
        m_idleRestart = false;
    
}

void 
TcpBbr::bbr_check_probe_rtt_done(Ptr<TcpSocketState> tcb)
{
    //std::cout << "bbr_check_probe_rtt_done" << std::endl;
    if(!(m_probeRttDoneStamp != Seconds(0) && Simulator::Now() > m_probeRttDoneStamp))
        return;
        
    m_probeRttMinStamp = Simulator::Now(); 
    tcb->m_cWnd = std::max(m_priorCwnd, tcb->m_cWnd.Get()); // prioe restore cwnd
    //ExitProbeRTT();
    bbr_exit_probe_rtt(tcb);
}

void 
TcpBbr::bbr_exit_probe_rtt(Ptr<TcpSocketState> tcb)
{
    //std::cout << "bbr_exit_probe_rtt" << std::endl;
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
TcpBbr::EnterProbeRTT()
{
    NS_LOG_FUNCTION(this);
    SetBbrState(BbrMode_t::BBR_PROBE_RTT);
    //m_pacingGain = 1;
    //m_cWndGain = 1;
}

void
TcpBbr::bbr_save_cwnd(Ptr<const TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    if (tcb->m_congState != TcpSocketState::CA_RECOVERY && m_state != BbrMode_t::BBR_PROBE_RTT)
        m_priorCwnd = tcb->m_cWnd;
    else
        m_priorCwnd = std::max(m_priorCwnd, tcb->m_cWnd.Get());
}

void
TcpBbr::RestoreCwnd(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    tcb->m_cWnd = std::max(m_priorCwnd, tcb->m_cWnd.Get());
}

void
TcpBbr::ExitProbeRTT()
{
    NS_LOG_FUNCTION(this);
    if (m_fullBwReached)
    {
        EnterProbeBW();
    }
    else
    {
        std::cout << "Enter Startup" << std::endl;
        EnterStartup();
    }
}

void
TcpBbr::HandleProbeRTT(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);

    uint32_t totalBytes = m_delivered + tcb->m_bytesInFlight.Get();
    m_appLimited = (totalBytes > 0 ? totalBytes : 1);

    if (m_probeRttDoneStamp == Seconds(0) && tcb->m_bytesInFlight <= m_minPipeCwnd)
    {
        m_probeRttDoneStamp = Simulator::Now() + m_probeRttDuration;
        m_probeRttRoundDone = false;
        m_nextRoundDelivered = m_delivered;
    }
    else if (m_probeRttDoneStamp != Seconds(0))
    {
        if (m_roundStart)
        {
            m_probeRttRoundDone = true;
        }
        if (m_probeRttRoundDone && Simulator::Now() > m_probeRttDoneStamp)
        {
            m_rtPropStamp = Simulator::Now();
            RestoreCwnd(tcb);
            ExitProbeRTT();
        }
    }
}

void
TcpBbr::CheckProbeRTT(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb);
    if (m_state != BbrMode_t::BBR_PROBE_RTT && m_rtPropExpired && !m_idleRestart)
    {
        EnterProbeRTT();
        bbr_save_cwnd(tcb);
        m_probeRttDoneStamp = Seconds(0);
    }

    if (m_state == BbrMode_t::BBR_PROBE_RTT)
    {
        HandleProbeRTT(tcb);
    }

    if (rs.m_delivered)
    {
        m_idleRestart = false;
    }
}

void
TcpBbr::SetSendQuantum(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_sendQuantum = 1 * tcb->m_segmentSize;
}

void
TcpBbr::UpdateTargetCwnd(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_targetCWnd = InFlight(tcb, bbr_bw() ,m_cWndGain) + AckAggregationCwnd();
}

uint32_t
TcpBbr::AckAggregationCwnd()
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
TcpBbr::bbr_update_ack_aggregation(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
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

bool
TcpBbr::ModulateCwndForRecovery(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
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
TcpBbr::SetCwnd(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);

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

    UpdateTargetCwnd(tcb);

    if (m_fullBwReached)
    {
        tcb->m_cWnd = std::min(tcb->m_cWnd.Get() + (uint32_t)rs.m_ackedSacked, m_targetCWnd);
    }
    else if (tcb->m_cWnd < m_targetCWnd || m_delivered < tcb->m_initialCWnd * tcb->m_segmentSize)
    {
        tcb->m_cWnd = tcb->m_cWnd.Get() + rs.m_ackedSacked;
    }
    tcb->m_cWnd = std::max(tcb->m_cWnd.Get(), m_minPipeCwnd);

done:
    if (m_state == BbrMode_t::BBR_PROBE_RTT)
        tcb->m_cWnd = std::min(tcb->m_cWnd.Get(), m_minPipeCwnd);
    
}

uint32_t
TcpBbr::bbr_update_round_start(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    //std::cout << "bbr_update_round_start" << std::endl;
    uint32_t round_delivered = 0;
    m_roundStart = false;

    if (rs.m_interval > Seconds(0) &&
        rs.m_priorDelivered >= m_nextRoundDelivered) // equivalent to !before
    {
        round_delivered = m_delivered - m_nextRoundDelivered;
        m_nextRoundDelivered = m_delivered; // m delivered is the total number of bytes delivered
        m_roundCount++; // might not need 
        m_roundStart = true;
        //m_packetConservation = false;  // might not need
    }
    return round_delivered;
}

DataRate
TcpBbr::bbr_max_bw()
{
    //std::cout << "bbr_max_bw" << std::endl;
    //std::cout << "bw_hi[0] = " << bw_hi[0] << std::endl;
    //std::cout << "bw_hi[1] = " << bw_hi[1] << std::endl;
    return std::max<DataRate>(bw_hi[0], bw_hi[1]);
}

void
TcpBbr::bbr_advance_max_bw_filter()
{
    //std::cout << "bbr_advance_max_bw_filter" << std::endl;
    if(bw_hi[1] == 0)
    {
        return;
    }
    bw_hi[0] = bw_hi[1];
    bw_hi[1] = 0;
}

void
TcpBbr::bbr_take_max_bw_sample(DataRate bw)
{
    //std::cout << "bbr_take_max_bw_sample" << std::endl;
    bw_hi[1] = std::max<DataRate>(bw_hi[1], bw);
}

DataRate 
TcpBbr::bbr_bw()
{
    // std::cout << "bbr_bw" << std::endl;
    // DataRate bw = std::max<DataRate>(bbr_max_bw(), m_bwLo);
    // std::cout << "bbr_bw >> " << bw << std::endl;
    //std::cout << "bbr_bwlow >> " << m_bwLo << std::endl;
    //return std::min<DataRate>(bbr_max_bw(), m_bwLo);
    return bbr_max_bw();
}

uint32_t 
TcpBbr::bbr_bdp(Ptr<TcpSocketState> tcb, DataRate bw, double gain)
{
    if (m_rtProp == Time::Max())
        return tcb->m_initialCWnd * tcb->m_segmentSize;
    return ((bw * m_rtProp) * gain) / 8;
}

void
TcpBbr::bbr_reset_full_bw()
{
    NS_LOG_FUNCTION(this);
    //std::cout << "bbr_reset_full_bw" << std::endl;
    m_fullBandwidth = 0;
    m_fullBandwidthCount = 0;
    m_fullBandwidthNow = false;
}

void
TcpBbr::bbr_reset_lower_bounds()
{
    //std::cout << "bbr_reset_lower_bounds" << std::endl;
    m_bwLo = 0; 
    m_inflightLo = std::numeric_limits<int>::max ();
}

bool
TcpBbr::bbr_adapt_upper_bounds(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    //std::cout << "bbr_adapt_upper_bounds" << std::endl;
    if (m_ackPhase == BbrAckPhase_t::BBR_ACKS_PROBE_STARTING && m_roundStart)
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_PROBE_FEEDBACK;

    if (m_ackPhase == BbrAckPhase_t::BBR_ACKS_PROBE_STOPPING && m_roundStart) 
    {
        m_bwProbeSamples = true;
        m_ackPhase = BbrAckPhase_t::BBR_ACKS_INIT;
        if (m_state == BbrMode_t::BBR_PROBE_BW && !rs.m_isAppLimited)
            bbr_advance_max_bw_filter();
        
        if (m_state == BbrMode_t::BBR_PROBE_BW && m_stoppedRiskyProbe && m_prevProbeTooHigh)
        {
            bbr_start_bw_probe_refill(tcb, 0);  
            return true;
        }
    }

    if (bbr_is_inflight_too_high(tcb, rs))
    {   
        if (m_bwProbeSamples > 0)
                bbr_handle_inflight_too_high(tcb, rs);
    } else {

        if (m_inflightHi == std::numeric_limits<int>::max ())
            return false;

        if (tcb->m_bytesInFlight > m_inflightHi) 
            m_inflightHi = tcb->m_bytesInFlight;
        
        if (m_state == BBR_PROBE_BW && m_cycleIndex == BBR_BW_PROBE_UP)
			bbr_probe_inflight_hi_upward(tcb, rs);
    }
    return false;
}

void 
TcpBbr::bbr_adapt_lower_bounds(Ptr<TcpSocketState> tcb,  const TcpRateOps::TcpRateSample& rs)
{
    //std::cout << "bbr_adapt_lower_bounds" << std::endl;
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
        bbr_init_lower_bounds(tcb, false);
        bbr_loss_lower_bounds(tcb, &m_bwLo, &m_inflightLo);
    }

    // m_inflightLo = min(m_inflightLo, ecn_inflight_lo);  afetr ecn is implemented
	m_bwLo = std::max<DataRate>(DataRate(1), m_bwLo); // ????????????????????
}

void
TcpBbr::bbr_reset_congestion_signals()
{
    //std::cout << "bbr_reset_congestion_signals" << std::endl;
    m_lossInRound = false;
    m_ecnInRound = false;  
    m_lossInCycle = false;
    m_ecn_in_cycle = false;
    m_bwLatest = 0;
    m_inflightLatest = 0;
}

void
TcpBbr::bbr_calculate_bw_sample(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, bbr_context *ctx)
{
    //std::cout << "bbr_calculate_bw_sample" << std::endl;
    NS_LOG_FUNCTION(this << tcb << rs << ctx);
    DataRate bw = 0;
	if (rs.m_interval > Seconds(0)) {
		if (rs.m_delivered < 0)
			return;
		bw = DataRate(rs.m_delivered / rs.m_interval.GetSeconds() * 8);

	}
	ctx->sample_bw = bw;
}

void 
TcpBbr::bbr_pick_probe_wait()
{
    //std::cout << "bbr_pick_probe_wait" << std::endl;
    m_roundsSinceProbe = (uint)m_uv->GetValue(0, 2);
    m_probeWaitTime = MilliSeconds(m_uv->GetValue(0, 1000));
}


//formerly known as UpdateBtlBw in ns3
void
TcpBbr::bbr_update_congestion_signals(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    if (rs.m_interval <= Seconds(0) || !rs.m_ackedSacked)
        return;

    DataRate bw = ctx->sample_bw;

    if (!rs.m_isAppLimited || bw >= bbr_max_bw())   
        bbr_take_max_bw_sample(rs.m_deliveryRate);
        
    m_lossInRound |= (rs.m_bytesLoss > 0);
        
    if (!m_lossRoundStart)
        return;

    //bbr_adapt_lower_bounds(tcb, rs);

    m_lossInRound = false;
    // ecn_in_round = false; // ECN SUPPORT 
}

// void
// TcpBbr::bbr_update_congestion_signals(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
// {
//     NS_LOG_FUNCTION(this << tcb << rs);
//     if (rs.m_deliveryRate == 0 )
//         return;
//     if (rs.m_deliveryRate >= bbr_bw() || !rs.m_isAppLimited)
//     {
//         if (m_state != BBR_PROBE_RTT)
//             bbr_take_max_bw_sample(rs.m_deliveryRate);   
//         //m_maxBwFilter.Update(rs.m_deliveryRate, m_roundCount);
//     }
// }

void
TcpBbr::bbr_check_loss_too_high_in_startup(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    if (m_state != BbrMode_t::BBR_STARTUP)
      return;
    
    if (bbr_full_bw_reached())
		return;

    if (rs.m_bytesLoss > 0 && m_lossEventsInRound < 15)
        m_lossEventsInRound++;

    if (m_lossRoundStart && tcb->m_congState == TcpSocketState::CA_RECOVERY && m_lossEventsInRound >= bbr_full_loss_cnt)
    {
        bbr_handle_queue_too_high_in_startup(tcb);
        return;
    }


    if(m_lossRoundStart)
        m_lossEventsInRound = 0;
}

void 
TcpBbr::bbr_handle_queue_too_high_in_startup(Ptr<TcpSocketState> tcb)
{
    NS_LOG_FUNCTION(this << tcb);
    m_fullBwReached = true;
    uint32_t bdp = InFlight(tcb, bbr_max_bw(), 1);
    m_inflightHi = std::max<uint32_t>(bdp, m_inflightLatest);
}




void
TcpBbr::bbr_update_model(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs, struct bbr_context *ctx)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    bbr_update_congestion_signals(tcb, rs, ctx);
    bbr_update_ack_aggregation(tcb, rs);
    bbr_check_loss_too_high_in_startup(tcb, rs);
    bbr_check_full_bw_reached(rs, ctx);
    bbr_check_drain(tcb);
    // CheckCyclePhase(tcb, rs);    // legacy cycling      
    bbr_update_cycle_phase(tcb, rs, ctx);   //CheckCyclePhase(tcb, rs);    // legacy cycling                    
    bbr_update_min_rtt(tcb, rs);
}

void
TcpBbr::UpdateControlParameters(Ptr<TcpSocketState> tcb, const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    std::cout << "PACING GAIN "<< m_pacingGain << std::endl;
    SetPacingRate(tcb, m_pacingGain);
    SetSendQuantum(tcb);
    SetCwnd(tcb, rs);
}

void
TcpBbr::SetBbrState(BbrMode_t mode)
{
    NS_LOG_FUNCTION(this << mode);
    NS_LOG_DEBUG(Simulator::Now() << " Changing from " << BbrModeName[m_state] << " to "
                                  << BbrModeName[mode]);
    m_state = mode;
}

uint32_t
TcpBbr::GetBbrState()
{
    NS_LOG_FUNCTION(this);
    return m_state;
}

double
TcpBbr::GetCwndGain()
{
    NS_LOG_FUNCTION(this);
    return m_cWndGain;
}

double
TcpBbr::GetPacingGain()
{
    NS_LOG_FUNCTION(this);
    return m_pacingGain;
}

std::string
TcpBbr::GetName() const
{
    return "TcpBbr";
}

bool
TcpBbr::HasCongControl() const
{
    NS_LOG_FUNCTION(this);
    return true;
}

void
TcpBbr::bbr_update_gains()
{
    NS_LOG_FUNCTION(this);
    //std::cout << "bbr_update_gains" << std::endl;
	switch (m_state) {
	case BBR_STARTUP:
		m_pacingGain = 2.77;
		m_cWndGain	 = 2;
		break;
	case BBR_DRAIN:
		m_pacingGain = 1.0 * 1000.0 / 2885.0;  /* slow, to drain */
		m_cWndGain	 = 2;  /* keep cwnd */
		break;
	case BBR_PROBE_BW:
        //std::cout << "before m_pacingGain from update gains>> " << m_pacingGain << std::endl;
		m_pacingGain = PACING_GAIN_CYCLE[m_cycleIndex];
        //std::cout << "m_pacingGain from update gains>> " << m_pacingGain << std::endl;
		m_cWndGain	 = 2;
		//if (bbr_bw_probe_cwnd_gain !=0  && m_cycleIndex == BBR_BW_PROBE_UP)
		//    m_cWndGain += 1 * bbr_bw_probe_cwnd_gain / 4;
		break;
	case BBR_PROBE_RTT:
		m_pacingGain = 1;
		m_cWndGain	 = 1;
		break;
	default:
		break;
	}

}

// void
// TcpBbr::bbr_update_gains()
// {
//     NS_LOG_FUNCTION(this);
//     //std::cout << "bbr_update_gains" << std::endl;
// 	switch (m_state) {
// 	case BBR_STARTUP:
// 		m_pacingGain = 2.77;
// 		m_cWndGain	 = 2;
// 		break;
// 	case BBR_DRAIN:
// 		m_pacingGain = 1.0 * 1000.0 / 2885.0;  /* slow, to drain */
// 		m_cWndGain	 = 2;  /* keep cwnd */
// 		break;
// 	case BBR_PROBE_BW:
//         //std::cout << "before m_pacingGain from update gains>> " << m_pacingGain << std::endl;
// 		m_pacingGain = PACING_GAIN_CYCLE[m_cycleIndex];
//         //std::cout << "m_pacingGain from update gains>> " << m_pacingGain << std::endl;
// 		m_cWndGain	 = 2;
// 		if (bbr_bw_probe_cwnd_gain !=0  && m_cycleIndex == BBR_BW_PROBE_UP)
// 			m_cWndGain += 1 * bbr_bw_probe_cwnd_gain / 4;
// 		break;
// 	case BBR_PROBE_RTT:
// 		m_pacingGain = 1;
// 		m_cWndGain	 = 1;
// 		break;
// 	default:
// 		break;
// 	}

// }

//bbr_main

void
TcpBbr::CongControl(Ptr<TcpSocketState> tcb,
                    const TcpRateOps::TcpRateConnection& rc,
                    const TcpRateOps::TcpRateSample& rs)
{
    NS_LOG_FUNCTION(this << tcb << rs);
    struct bbr_context ctx = { 0 };
    //std::cout << "CYCLE INDEX IS >>>>> "<< m_cycleIndex << std::endl;
    //std::cout << "INFLIGH HI     >>>>> "<< m_inflightHi << std::endl;
    //std::cout << "start" << std::endl;
    //if (m_roundStart)
    //   std::cout << "ROUND START " << std::endl;
    m_delivered = rc.m_delivered;
    m_txItemDelivered = rc.m_txItemDelivered;
    //if (m_inflightHi == 0)
    //    m_inflightHi = std::max(m_inflightHi, tcb->m_bytesInFlight.Get());

    bbr_update_round_start(tcb, rs);

    bbr_calculate_bw_sample(tcb, rs, &ctx);
    bbr_update_latest_delivery_signals(tcb, rs, &ctx);

    bbr_update_model(tcb, rs, &ctx);

    bbr_update_gains();

    UpdateControlParameters(tcb, rs);

    // std::cout << "BEFORE INFLIGHT MODEL" << std::endl;
    // std::cout << "inflight HI    >>>>" << m_inflightHi << std::endl;
    // std::cout << "inflight LO    >>>>" << m_inflightLo   << std::endl;

    bbr_bound_cwnd_for_inflight_model(tcb);
    // std::cout << "AFTER INFLIGHT MODEL" << std::endl;
    // std::cout << "inflight HI    >>>>" << m_inflightHi << std::endl;
    // std::cout << "inflight LO    >>>>" << m_inflightLo   << std::endl;
    // //std::cout << "CWND GAIN    >>>>" << m_pacingGain << std::endl;
    //std::cout << "PACING  GAIN >>>>" << m_cWndGain   << std::endl;


}

void
TcpBbr::CongestionStateSet(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
    NS_LOG_FUNCTION(this << tcb << newState);
    if (newState == TcpSocketState::CA_OPEN && !m_isInitialized)
    {

        m_rtProp = tcb->m_lastRtt.Get() != Time::Max() ? tcb->m_lastRtt.Get() : Time::Max();
        m_rtPropStamp = Simulator::Now();
        m_priorCwnd = tcb->m_cWnd;
        tcb->m_ssThresh = tcb->m_initialSsThresh;
        m_targetCWnd = tcb->m_cWnd;
        m_minPipeCwnd = 4 * tcb->m_segmentSize;
        m_sendQuantum = 1 * tcb->m_segmentSize;
        bbr_init_lower_bounds(tcb, false);
        InitRoundCounting();

        InitFullPipe();
        EnterStartup();
        //SetBbrState(BbrMode_t::BBR_STARTUP);
        InitPacingRate(tcb);
        
        m_ackEpochTime = Simulator::Now();
        m_extraAckedWinRtt = 0;
        m_extraAckedIdx = 0;
        m_ackEpochAcked = 0;
        m_extraAcked[0] = 0;
        m_extraAcked[1] = 0;
        m_isInitialized = true;
    }
    else if (newState == TcpSocketState::CA_LOSS)
    {
        bbr_save_cwnd(tcb);
        m_roundStart = true;
        if (bbr_is_probing_bandwidth(tcb) && m_inflightLo == std::numeric_limits<int>::max ())
        {
          m_inflightLo = m_priorCwnd;
        }
    }
    else if (newState == TcpSocketState::CA_RECOVERY)
    {
        bbr_save_cwnd(tcb);
        tcb->m_cWnd =
            tcb->m_bytesInFlight.Get() + std::max(tcb->m_lastAckedSackedBytes, tcb->m_segmentSize);
        m_packetConservation = true;
    }
}

void
TcpBbr::CwndEvent(Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCAEvent_t event)
{
    NS_LOG_FUNCTION(this << tcb << event);
    if (event == TcpSocketState::CA_EVENT_COMPLETE_CWR)
    {
        NS_LOG_DEBUG("CwndEvent triggered to CA_EVENT_COMPLETE_CWR :: " << event);
        m_packetConservation = false;
        RestoreCwnd(tcb);
    }
    else if (event == TcpSocketState::CA_EVENT_TX_START && m_appLimited)
    {
        NS_LOG_DEBUG("CwndEvent triggered to CA_EVENT_TX_START :: " << event);
        m_idleRestart = true;
        m_ackEpochTime = Simulator::Now();
        m_ackEpochAcked = 0;
        if (m_state == BbrMode_t::BBR_PROBE_BW)
        {
            SetPacingRate(tcb, 1);
        }
        else if (m_state == BbrMode_t::BBR_PROBE_RTT)
        {
            if (m_probeRttRoundDone && Simulator::Now() > m_probeRttDoneStamp)
            {
                m_rtPropStamp = Simulator::Now();
                RestoreCwnd(tcb);
                bbr_exit_probe_rtt(tcb);
            }
        }
    }
}

uint32_t
TcpBbr::GetSsThresh(Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
    NS_LOG_FUNCTION(this << tcb << bytesInFlight);
    bbr_save_cwnd(tcb);
    return tcb->m_ssThresh;
}

Ptr<TcpCongestionOps>
TcpBbr::Fork()
{
    return CopyObject<TcpBbr>(this);
}

} // namespace ns3