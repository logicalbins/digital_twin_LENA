// Copyright (c) 2023 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \ingroup examples
 * \file cttc-nr-mimo-demo.cc
 * \brief An example that shows how to setup and use MIMO
 *
 * This example describes how to setup a simulation using MIMO. The scenario
 * consists of a simple topology, in which there
 * is only one gNB and one UE. An additional pair of gNB and UE can be enabled
 * to simulate the interference (see enableInterfNode).
 * Example creates one DL flow that goes through only BWP.
 *
 * The example prints on-screen and into the file the end-to-end result of the flow.
 * To see all the input parameters run:
 *
 * \code{.unparsed}
$ ./ns3 run cttc-nr-mimo-demo -- --PrintHelp
    \endcode
 *
 *
 * MIMO is enabled by default. To disable it run:
 *  * \code{.unparsed}
$  ./ns3 run cttc-nr-mimo-demo -- --enableMimoFeedback=0
    \endcode
 *
 *
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-socket.h"
#include <limits>
#include <cmath>
#include <cstdint>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("CttcNrMimoDemo");

// ---- Windowed goodput counters ----
static uint64_t g_dlBytesWin = 0;
static uint64_t g_ulBytesWin = 0;

// These will be set in main using your times
static ns3::Time g_dlWinStart, g_dlWinStop;
static ns3::Time g_ulWinStart, g_ulWinStop;

static uint32_t g_pingTx = 0, g_pingRx = 0;
static double   g_rttSumMs = 0.0, g_rttSqSumMs = 0.0;
static double   g_rttMinMs = std::numeric_limits<double>::infinity();
static double   g_rttMaxMs = 0.0;

// V4Ping "Tx": seq, packet
static void PingTxTracer(uint16_t /*seq*/, ns3::Ptr<ns3::Packet> /*p*/)
{
    ++g_pingTx;
}

// V4Ping "Rtt": seq, rtt
static void PingRttTracer(uint16_t /*seq*/, ns3::Time rtt)
{
    const double ms = rtt.GetMilliSeconds();
    ++g_pingRx;
    g_rttSumMs   += ms;
    g_rttSqSumMs += ms * ms;
    if (ms < g_rttMinMs) g_rttMinMs = ms;
    if (ms > g_rttMaxMs) g_rttMaxMs = ms;
}

static void DlRxTrace(Ptr<const Packet> p, const Address &from)
{
    ns3::Time now = ns3::Simulator::Now();
    if (now >= g_dlWinStart && now <= g_dlWinStop)
        g_dlBytesWin += p->GetSize();
}

static void UlRxTrace(Ptr<const Packet> p, const Address &from)
{
    ns3::Time now = ns3::Simulator::Now();
    if (now >= g_ulWinStart && now <= g_ulWinStop)
        g_ulBytesWin += p->GetSize();
}

int
main(int argc, char* argv[])
{
    Config::SetDefault("ns3::NrHelper::EnableMimoFeedback", BooleanValue(true));
    Config::SetDefault("ns3::NrPmSearch::SubbandSize", UintegerValue(16));
    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping", EnumValue(NrGnbRrc::RLC_AM_ALWAYS));
    // PHY/MAC link adaptation & scheduling
    // LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
    // LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
    // LogComponentEnable("NrAmc", LOG_LEVEL_INFO);


    // LogComponentEnable("NrGnbMac", LOG_LEVEL_INFO);
    // LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

    // RLC layer
    // LogComponentEnable("NrRlcUm", LOG_LEVEL_INFO);  // or NrRlcUm if using UM

    // TCP behavior
    // LogComponentEnable("TcpSocketBase", LOG_LEVEL_INFO);
    // LogComponentEnable("TcpCongestionOps", LOG_LEVEL_INFO);
    // LogComponentEnable("TcpTxBuffer", LOG_LEVEL_INFO);
    // LogComponentEnable("TcpRxBuffer", LOG_LEVEL_INFO);
    bool useMimoPmiParams = true;

    NrHelper::AntennaParams apUe;
    NrHelper::AntennaParams apGnb;
    apUe.antennaElem = "ns3::ThreeGppAntennaModel";
    apUe.nAntCols = 2;
    apUe.nAntRows = 2;
    apUe.nHorizPorts = 2;
    apUe.nVertPorts = 1;
    apUe.isDualPolarized = false;
    apGnb.antennaElem = "ns3::ThreeGppAntennaModel";
    apGnb.nAntCols = 4;
    apGnb.nAntRows = 2;
    apGnb.nHorizPorts = 2;
    apGnb.nVertPorts = 1;
    apGnb.isDualPolarized = false;

    // The polarization slant angle in degrees in case of x-polarized
    double polSlantAngleGnb = 0.0;
    double polSlantAngleUe = 90.0;
    // The bearing angles in degrees
    double bearingAngleGnb = 0.0;
    double bearingAngleUe = 180.0;

    // Traffic parameters
    uint32_t udpPacketSize = 1000;
    // For 2x2 MIMO and NR MCS table 2, packet interval is 40000 ns to
    // reach 200 mb/s
    Time packetInterval = NanoSeconds(40000);
    Time udpAppStartTime = MilliSeconds(400);

    // Interference
    bool enableInterfNode = false; // if true an additional pair of gNB and UE will be created
                                   // to create an interference towards the original pair
    double interfDistance = 100.0; // the distance in meters between the original node pair, and the
                                   // interfering node pair
    double interfPolSlantDelta = 0; // the difference between the pol. slant angle between the
                                    // original node and the interfering one

    // Other simulation scenario parameters
    Time simTime = MilliSeconds(7000);
    Time warmupTimeDL = MilliSeconds(2000);
    Time warmupTimeUL = MilliSeconds(1000);

    Time dlStart = udpAppStartTime;
    Time dlStop  = MilliSeconds(5500);       // stop DL before simTime
    Time ulStart = dlStop + MilliSeconds(200);
    Time ulStop  = simTime;

    // Skip warm-up parts
    g_dlWinStart = dlStart + warmupTimeDL;
    g_dlWinStop  = dlStop;

    g_ulWinStart = ulStart + warmupTimeUL;
    g_ulWinStop  = ulStop;

    double gnbUeDistance = 4.2; // meters
    uint16_t numerology = 1;
    double centralFrequency = 4.15e9;
    double bandwidth = 100e6;
    double txPowerGnb = 20; // dBm
    double txPowerUe = 23;  // dBm
    uint16_t updatePeriodMs = 0;
    std::string errorModel = "ns3::NrEesmIrT2";
    std::string scheduler = "ns3::NrMacSchedulerOfdmaPF";
    std::string beamformingMethod = "ns3::DirectPathBeamforming";
    /**
     *   UMi_StreetCanyon,      //!< UMi_StreetCanyon
     *   UMi_StreetCanyon_LoS,  //!< UMi_StreetCanyon where all the nodes will be in Line-of-Sight
     *   UMi_StreetCanyon_nLoS, //!< UMi_StreetCanyon where all the nodes will not be in
     *
     */

    uint16_t losCondition = 0;

    // Where the example stores the output files.
    std::string simTag = "default";
    std::string outputDir = "./";
    bool logging = false;

    CommandLine cmd(__FILE__);
    /**
     * The main parameters for testing MIMO
     */
    cmd.AddValue("enableMimoFeedback", "ns3::NrHelper::EnableMimoFeedback");
    cmd.AddValue("pmSearchMethod", "ns3::NrHelper::PmSearchMethod");
    cmd.AddValue("fullSearchCb", "ns3::NrPmSearchFull::CodebookType");
    cmd.AddValue("rankLimit", "ns3::NrPmSearch::RankLimit");
    cmd.AddValue("subbandSize", "ns3::NrPmSearch::SubbandSize");
    cmd.AddValue("downsamplingTechnique", "ns3::NrPmSearch::DownsamplingTechnique");
    cmd.AddValue("numRowsGnb", "Number of antenna rows at the gNB", apGnb.nAntRows);
    cmd.AddValue("numRowsUe", "Number of antenna rows at the UE", apUe.nAntRows);
    cmd.AddValue("numColumnsGnb", "Number of antenna columns at the gNB", apGnb.nAntCols);
    cmd.AddValue("numColumnsUe", "Number of antenna columns at the UE", apUe.nAntCols);
    cmd.AddValue("numVPortsGnb",
                 "Number of vertical ports of the antenna at the gNB",
                 apGnb.nVertPorts);
    cmd.AddValue("numVPortsUe",
                 "Number of vertical ports of the antenna at the UE",
                 apUe.nVertPorts);
    cmd.AddValue("numHPortsGnb",
                 "Number of horizontal ports of the antenna the gNB",
                 apGnb.nHorizPorts);
    cmd.AddValue("numHPortsUe",
                 "Number of horizontal ports of the antenna at the UE",
                 apUe.nHorizPorts);
    cmd.AddValue("xPolGnb",
                 "Whether the gNB antenna array has the cross polarized antenna "
                 "elements.",
                 apGnb.isDualPolarized);
    cmd.AddValue("xPolUe",
                 "Whether the UE antenna array has the cross polarized antenna "
                 "elements.",
                 apUe.isDualPolarized);
    cmd.AddValue("polSlantAngleGnb",
                 "Polarization slant angle of gNB in degrees",
                 polSlantAngleGnb);
    cmd.AddValue("polSlantAngleUe", "Polarization slant angle of UE in degrees", polSlantAngleUe);
    cmd.AddValue("bearingAngleGnb", "Bearing angle of gNB in degrees", bearingAngleGnb);
    cmd.AddValue("bearingAngleUe", "Bearing angle of UE in degrees", bearingAngleUe);
    cmd.AddValue("enableInterfNode", "Whether to enable an interfering node", enableInterfNode);
    cmd.AddValue(
        "interfDistance",
        "The distance between the pairs of gNB and UE (the original and the interfering one)",
        interfDistance);
    cmd.AddValue("interfPolSlantDelta",
                 "The difference between the pol. slant angles of the original pairs of gNB and UE "
                 "and the interfering one",
                 interfPolSlantDelta);

    /**
     * Other simulation parameters
     */
    cmd.AddValue("packetSize",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSize);
    cmd.AddValue("packetInterval", "Inter-packet interval for CBR traffic", packetInterval);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerology", "The numerology to be used", numerology);
    cmd.AddValue("centralFrequency", "The system frequency to be used in band 1", centralFrequency);
    cmd.AddValue("bandwidth", "The system bandwidth to be used", bandwidth);
    cmd.AddValue("txPowerGnb", "gNB TX power", txPowerGnb);
    cmd.AddValue("txPowerUe", "UE TX power", txPowerUe);
    cmd.AddValue("gnbUeDistance",
                 "The distance between the gNB and the UE in the scenario",
                 gnbUeDistance);
    cmd.AddValue(
        "updatePeriodMs",
        "Channel update period in ms. If set to 0 then the channel update will be disabled",
        updatePeriodMs);
    cmd.AddValue("errorModel",
                 "Error model: ns3::NrEesmCcT1, ns3::NrEesmCcT2, "
                 "ns3::NrEesmIrT1, ns3::NrEesmIrT2, ns3::NrLteMiErrorModel",
                 errorModel);
    cmd.AddValue("scheduler",
                 "The scheduler: ns3::NrMacSchedulerTdmaRR, "
                 "ns3::NrMacSchedulerTdmaPF, ns3::NrMacSchedulerTdmaMR,"
                 "ns3::NrMacSchedulerTdmaQos, ns3::NrMacSchedulerOfdmaRR, "
                 "ns3::NrMacSchedulerOfdmaPF, ns3::NrMacSchedulerOfdmaMR,"
                 "ns3::NrMacSchedulerOfdmaQos",
                 scheduler);
    cmd.AddValue("beamformingMethod",
                 "The beamforming method: ns3::CellScanBeamforming,"
                 "ns3::CellScanBeamformingAzimuthZenith,"
                 "ns3::CellScanQuasiOmniBeamforming,"
                 "ns3::DirectPathBeamforming,"
                 "ns3::QuasiOmniDirectPathBeamforming,"
                 "ns3::DirectPathQuasiOmniBeamforming",
                 beamformingMethod);
    cmd.AddValue("losCondition",
                 "0 - for 3GPP channel condition model,"
                 "1 - for always LOS channel condition model,"
                 "2 - for always NLOS channel condition model",
                 losCondition);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("useMimoPmiParams", "Configure via the MimoPmiParams structure", useMimoPmiParams);
    // Parse the command line
    cmd.Parse(argc, argv);

    // convert angle values into radians
    apUe.bearingAngle = bearingAngleUe * (M_PI / 180);
    apUe.polSlantAngle = polSlantAngleUe * (M_PI / 180);
    apGnb.bearingAngle = bearingAngleGnb * (M_PI / 180);
    apGnb.polSlantAngle = polSlantAngleGnb * (M_PI / 180);

    NS_ABORT_IF(centralFrequency < 0.5e9 && centralFrequency > 100e9);
    NS_ABORT_UNLESS(losCondition < 3);

    if (logging)
    {
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
        LogComponentEnable("UdpServer", LOG_LEVEL_INFO);
        LogComponentEnable("NrPdcp", LOG_LEVEL_INFO);
    }

    Config::SetDefault("ns3::NrRlcAm::MaxTxBufferSize", UintegerValue(6 * 1024 * 1024));
    // Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
    //                    TimeValue(MilliSeconds(updatePeriodMs)));

    uint16_t pairsToCreate = 1;
    if (enableInterfNode)
    {
        pairsToCreate = 2;
    }

    NodeContainer gnbContainer;
    gnbContainer.Create(pairsToCreate);
    NodeContainer ueContainer;
    ueContainer.Create(pairsToCreate);

    /**
     * We configure the mobility model to ConstantPositionMobilityModel.
     * The default topology is the following:
     *
     *         gNB .........(20 m) .........UE
     *    (0.0, h, 10.0)              (d, h, 1.5)
     *
     *
     *         gNB..........(20 m)..........UE
     *   (0.0, 0.0, 10.0)               (d, 0.0, 1.5)
     */
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 3.0));
    positionAlloc->Add(Vector(gnbUeDistance, 0.0, 1.5));
    // the positions for the second interfering pair of gNB and UE
    if (enableInterfNode)
    {
        positionAlloc->Add(Vector(0.0, interfDistance, 10.0));
        positionAlloc->Add(Vector(gnbUeDistance, interfDistance, 1.5));
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbContainer.Get(0));
    mobility.Install(ueContainer.Get(0));
    // install mobility of the second pair of gNB and UE
    if (enableInterfNode)
    {
        mobility.Install(gnbContainer.Get(1));
        mobility.Install(ueContainer.Get(1));
    }

    /**
     * Create the NR helpers that will be used to create and setup NR devices, spectrum, ...
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
    /**
     * Prepare spectrum. Prepare one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     *
     *
     * The configured spectrum division is:
     * ------------Band--------------
     * ------------CC1----------------
     * ------------BWP1---------------
     */

    BandwidthPartInfo::Scenario scenario =
        BandwidthPartInfo::Scenario(BandwidthPartInfo::InH_OfficeMixed + losCondition);

    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   numCcPerBand,
                                                   scenario);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    /**
     * Configure NrHelper, prepare most of the parameters that will be used in the simulation.
     */
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                TimeValue(MilliSeconds(updatePeriodMs)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));


    // 256-QAM (MCS Table 2) via EESM IR Table 2
    nrHelper->SetGnbDlAmcAttribute("ErrorModelType", TypeIdValue(ns3::NrEesmIrT2::GetTypeId()));
    nrHelper->SetGnbUlAmcAttribute("ErrorModelType", TypeIdValue(ns3::NrEesmIrT2::GetTypeId()));

    nrHelper->SetSchedulerTypeId(TypeId::LookupByName(scheduler));
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(TypeId::LookupByName(beamformingMethod)));
    // Core latency
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(10)));

    // We can configure not only via Configure::SetDefault, but also via the MimoPmiParams structure
    if (useMimoPmiParams)
    {
        ns3::NrHelper::MimoPmiParams params;
        params.subbandSize = 16;
        params.fullSearchCb = "ns3::NrCbTypeOneSp";
        params.pmSearchMethod = "ns3::NrPmSearchFull";
        nrHelper->SetupMimoPmi(params);
    }

    /**
     * Configure gNb antenna
     */
    nrHelper->SetupGnbAntennas(apGnb);
    /**
     * Configure UE antenna
     */
    nrHelper->SetupUeAntennas(apUe);

    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPowerGnb));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPowerUe));

    uint32_t bwpId = 0;
    // gNb routing between bearer type and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(bwpId));
    // UE routing between bearer type and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT", UintegerValue(bwpId));
    /**
     * Initialize channel and pathloss, plus other things inside band.
     */
    nrHelper->InitializeOperationBand(&band);
    BandwidthPartInfoPtrVector allBwps;
    allBwps = CcBwpCreator::GetAllBwps({band});

    /**
     * Finally, create the gNB and the UE device.
     */
    // NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbContainer, allBwps);

    // TDD 4DL:1UL pattern
    const std::string tddPattern = "DL|DL|DL|UL|DL|";
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetAttribute("Pattern", StringValue(tddPattern));
    }

    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueContainer, allBwps);

    if (enableInterfNode && interfPolSlantDelta != 0)
    {
        // reconfigure the polarization slant angle of the interferer
        nrHelper->GetGnbPhy(gnbNetDev.Get(1), 0)
            ->GetSpectrumPhy()
            ->GetAntenna()
            ->SetAttribute("PolSlantAngle",
                           DoubleValue((polSlantAngleGnb + interfPolSlantDelta) * (M_PI / 180)));
        nrHelper->GetUePhy(ueNetDev.Get(1), 0)
            ->GetSpectrumPhy()
            ->GetAntenna()
            ->SetAttribute("PolSlantAngle",
                           DoubleValue((polSlantAngleUe + interfPolSlantDelta) * (M_PI / 180)));
    }

    /**
     * Fix the random stream throughout the nr, propagation, and spectrum
     * modules classes. This configuration is extremely important for the
     * reproducibility of the results.
     */
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    // When all the configuration is done, explicitly call UpdateConfig ()
    // TODO: Check if this is necessary to call when we do not reconfigure anything after devices
    // have been created
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // create the Internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.00001)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueContainer);
    Ipv4InterfaceContainer ueIpIface =
        nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    // Set the default gateway for the UE
    Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(ueContainer.Get(0)->GetObject<Ipv4>());
    ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);

    // --- PING START: ICMP echo from UE[0] -> remoteHost
    {
    Ipv4Address rhAddr = Ipv4Address::ConvertFrom(internetIpIfaces.GetAddress(1));

    PingHelper pingUl(rhAddr);                      // <-- use PingHelper
    pingUl.SetAttribute("Interval", TimeValue(Seconds(0.1)));
    pingUl.SetAttribute("Size", UintegerValue(56));
    pingUl.SetAttribute("Count", UintegerValue(5));

    ApplicationContainer pingUlApps = pingUl.Install(ueContainer.Get(0));
    Ptr<Application> pingApp = pingUlApps.Get(0);
    pingApp->TraceConnectWithoutContext("Tx",  MakeCallback(&PingTxTracer));
    pingApp->TraceConnectWithoutContext("Rtt", MakeCallback(&PingRttTracer));


    pingUlApps.Start(udpAppStartTime + MilliSeconds(100));
    pingUlApps.Stop(simTime - MilliSeconds(50));
    }
    // --- PING END

    // attach each UE to its gNB according to desired scenario
    nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));
    if (enableInterfNode)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(1), gnbNetDev.Get(1));
    }

    /**
     * Install DL traffic part.
     */
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    // The sink will always listen to the specified ports
    // UdpServerHelper dlPacketSink(dlPort);
    // The server, that is the application which is listening, is installed in the UE
    // serverApps.Add(dlPacketSink.Install(ueContainer));
    /**
     * Configure attributes for the CBR traffic generator, using user-provided
     * parameters
     */
    // UdpClientHelper dlClient;
    // dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    // dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    // dlClient.SetAttribute("PacketSize", UintegerValue(udpPacketSize));
    // dlClient.SetAttribute("Interval", TimeValue(packetInterval));

    ApplicationContainer clientApps;

    BulkSendHelper dlClientHelper(
                            "ns3::TcpSocketFactory",
                            InetSocketAddress(ueIpIface.GetAddress(0), dlPort));
    dlClientHelper.SetAttribute("MaxBytes", UintegerValue(0));
    dlClientHelper.SetAttribute("SendSize", UintegerValue(1400));
    clientApps.Add(dlClientHelper.Install(remoteHost));
    PacketSinkHelper dlPacketSinkHelper(
        "ns3::TcpSocketFactory",
        InetSocketAddress(Ipv4Address::GetAny(), dlPort));
    serverApps.Add(dlPacketSinkHelper.Install(ueContainer.Get(0)));

    // DL sink is in 'serverApps' (UE)
    Ptr<Application> dlApp = serverApps.Get(0);
    Ptr<PacketSink> dlSinkApp = DynamicCast<PacketSink>(dlApp);
    dlSinkApp->TraceConnectWithoutContext("Rx", MakeCallback(&DlRxTrace));

    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(6 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(6 * 1024 * 1024));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1400));

    // The filter for the traffic
    Ptr<NrEpcTft> dlTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlPktFilter;
    dlPktFilter.localPortStart = dlPort;
    dlPktFilter.localPortEnd = dlPort;
    dlTft->Add(dlPktFilter);

    // The bearer that will carry the traffic
    NrEpsBearer bearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(0), bearer, dlTft);

    /**
     * Let's install the applications!
     */

    // for (uint32_t i = 0; i < ueContainer.GetN(); ++i)
    // {
    //     Ptr<Node> ue = ueContainer.Get(i);
    //     Ptr<NetDevice> ueDevice = ueNetDev.Get(i);
    //     Address ueAddress = ueIpIface.GetAddress(i);

    //     // The client, who is transmitting, is installed in the remote host,
    //     // with destination address set to the address of the UE
    //     dlClient.SetAttribute("RemoteAddress", AddressValue(ueAddress));
    //     clientApps.Add(dlClient.Install(remoteHost));

    //     // Activate a dedicated bearer for the traffic
    //     nrHelper->ActivateDedicatedEpsBearer(ueDevice, epsBearer, dlTft);
    // }

    // start UDP server and client apps
    serverApps.Start(dlStart);
    clientApps.Start(dlStart);
    serverApps.Stop(dlStop);
    clientApps.Stop(dlStop);

    // --- UL TCP apps (UE -> RemoteHost), start after DL finishes
    ApplicationContainer ulServerApps; // sink on RemoteHost
    ApplicationContainer ulClientApps; // bulk on UE

    uint16_t ulPort = 2345;
    PacketSinkHelper ulSink("ns3::TcpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
    ulServerApps.Add(ulSink.Install(remoteHost));

    // UL sink is in 'ulServerApps' (RemoteHost)
    Ptr<Application> ulApp = ulServerApps.Get(0);
    Ptr<PacketSink> ulSinkApp = DynamicCast<PacketSink>(ulApp);
    ulSinkApp->TraceConnectWithoutContext("Rx", MakeCallback(&UlRxTrace));


    Ipv4Address rhAddr = Ipv4Address::ConvertFrom(internetIpIfaces.GetAddress(1));
    BulkSendHelper ulBulk("ns3::TcpSocketFactory",
                        InetSocketAddress(rhAddr, ulPort));
    ulBulk.SetAttribute("MaxBytes", UintegerValue(0));
    ulBulk.SetAttribute("SendSize", UintegerValue(1400));
    ulClientApps.Add(ulBulk.Install(ueContainer.Get(0)));

    // UL bearer/TFT (match remote/destination port for UL)
    Ptr<NrEpcTft> ulTft = Create<NrEpcTft>(); NrEpcTft::PacketFilter ulPf;
    ulPf.remotePortStart = ulPort; ulPf.remotePortEnd = ulPort; ulTft->Add(ulPf);
    NrEpsBearer ulBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(0), ulBearer, ulTft);

    // Schedule UL only in (ulStart, ulStop]
    ulServerApps.Start(ulStart);
    ulClientApps.Start(ulStart);
    ulServerApps.Stop(ulStop);
    ulClientApps.Stop(ulStop);

    // enable the traces provided by the nr module
    // nrHelper->EnableTraces();

    // FlowMonitorHelper flowmonHelper;
    // NodeContainer endpointNodes;
    // endpointNodes.Add(remoteHost);
    // endpointNodes.Add(ueContainer);
    // Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    // monitor->SetAttribute("StartTime", TimeValue(warmupTime + udpAppStartTime));
    // monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    // monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    // monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    Simulator::Stop(simTime);
    Simulator::Run();

    // Print per-flow statistics
    // monitor->CheckForLostPackets();
    // Ptr<Ipv4FlowClassifier> classifier =
    //     DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    // FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    // double averageFlowThroughput = 0.0;
    // double averageFlowDelay = 0.0;

    std::ofstream outFile;
    std::string filename = outputDir + "/" + simTag;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    // ---- Windowed goodput ----
    double dlWinDur = (g_dlWinStop - g_dlWinStart).GetSeconds();
    double ulWinDur = (g_ulWinStop - g_ulWinStart).GetSeconds();
    if (dlWinDur <= 0.0) dlWinDur = 1e-9;
    if (ulWinDur <= 0.0) ulWinDur = 1e-9;

    double dlGoodputMbps = g_dlBytesWin * 8.0 / dlWinDur / 1e6;
    double ulGoodputMbps = g_ulBytesWin * 8.0 / ulWinDur / 1e6;

    // Summary line after each phase block:
    outFile << "\n  (Windowed) DL throughput: " << dlGoodputMbps << " Mbps\n";
    outFile << "  (Windowed) UL throughput: " << ulGoodputMbps << " Mbps\n";

    // ---- Ping summary (avg RTT etc.) ----
    outFile << "\n=== Ping results ===\n";
    outFile << "  Packets: tx=" << g_pingTx << " rx=" << g_pingRx;
    if (g_pingTx > 0) {
        double lossPct = 100.0 * (double)(g_pingTx - g_pingRx) / (double)g_pingTx;
        outFile << "  loss=" << lossPct << "%\n";
    } else {
        outFile << "  loss=0%\n";
    }

    if (g_pingRx > 0) {
        double avg = g_rttSumMs / g_pingRx;
        double var = (g_rttSqSumMs / g_pingRx) - (avg * avg);
        if (var < 0) var = 0;
        double mdev = std::sqrt(var);

        outFile << "  rtt min/avg/max/mdev = "
                << g_rttMinMs << "/" << avg << "/" << g_rttMaxMs << "/" << mdev << " ms\n";
    } else {
        outFile << "  rtt min/avg/max/mdev = 0/0/0/0 ms\n";
    }

    outFile.close();
    std::ifstream f(filename.c_str());
    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();
    return 0;
}
