/// @file
/// @brief Contains definition of <b>"MsgId"</b> field.

#pragma once

#include <algorithm>
#include <cstdint>
#include <iterator>
#include "comms/field/EnumValue.h"
#include "comms/options.h"
#include "ublox/DefaultOptions.h"
#include "ublox/MsgId.h"
#include "ublox/field/FieldBase.h"

namespace ublox
{

namespace field
{

/// @brief Definition of <b>"MsgId"</b> field.
/// @tparam TOpt Protocol options.
/// @tparam TExtraOpts Extra options.
template <typename TOpt = ublox::DefaultOptions, typename... TExtraOpts>
class MsgId : public
    comms::field::EnumValue<
        ublox::field::FieldBase<comms::option::BigEndian>,
        ublox::MsgId,
        TExtraOpts...
    >
{
    using Base = 
        comms::field::EnumValue<
            ublox::field::FieldBase<comms::option::BigEndian>,
            ublox::MsgId,
            TExtraOpts...
        >;
public:
    /// @brief Name of the field.
    static const char* name()
    {
        return "MsgId";
    }
    
    /// @brief Validity check function.
    bool valid() const
    {
        if (!Base::valid()) {
            return false;
        }
    
        static const typename Base::ValueType Values[] = {
            ublox::MsgId_NavPosecef,
            ublox::MsgId_NavPosllh,
            ublox::MsgId_NavStatus,
            ublox::MsgId_NavDop,
            ublox::MsgId_NavAtt,
            ublox::MsgId_NavSol,
            ublox::MsgId_NavPvt,
            ublox::MsgId_NavOdo,
            ublox::MsgId_NavResetodo,
            ublox::MsgId_NavVelecef,
            ublox::MsgId_NavVelned,
            ublox::MsgId_NavHpposecef,
            ublox::MsgId_NavHpposllh,
            ublox::MsgId_NavTimegps,
            ublox::MsgId_NavTimeutc,
            ublox::MsgId_NavClock,
            ublox::MsgId_NavTimeglo,
            ublox::MsgId_NavTimebds,
            ublox::MsgId_NavTimegal,
            ublox::MsgId_NavTimels,
            ublox::MsgId_NavSvinfo,
            ublox::MsgId_NavDgps,
            ublox::MsgId_NavSbas,
            ublox::MsgId_NavOrb,
            ublox::MsgId_NavSat,
            ublox::MsgId_NavGeofence,
            ublox::MsgId_NavSvin,
            ublox::MsgId_NavRelposned,
            ublox::MsgId_NavEkfstatus,
            ublox::MsgId_NavAopstatus,
            ublox::MsgId_NavEoe,
            ublox::MsgId_RxmRaw,
            ublox::MsgId_RxmSfrb,
            ublox::MsgId_RxmSfrbx,
            ublox::MsgId_RxmMeasx,
            ublox::MsgId_RxmRawx,
            ublox::MsgId_RxmSvsi,
            ublox::MsgId_RxmAlm,
            ublox::MsgId_RxmEph,
            ublox::MsgId_RxmRtcm,
            ublox::MsgId_RxmPmreq,
            ublox::MsgId_RxmRlm,
            ublox::MsgId_RxmImes,
            ublox::MsgId_InfError,
            ublox::MsgId_InfWarning,
            ublox::MsgId_InfNotice,
            ublox::MsgId_InfTest,
            ublox::MsgId_InfDebug,
            ublox::MsgId_AckNak,
            ublox::MsgId_AckAck,
            ublox::MsgId_CfgPrt,
            ublox::MsgId_CfgMsg,
            ublox::MsgId_CfgInf,
            ublox::MsgId_CfgRst,
            ublox::MsgId_CfgDat,
            ublox::MsgId_CfgTp,
            ublox::MsgId_CfgRate,
            ublox::MsgId_CfgCfg,
            ublox::MsgId_CfgFxn,
            ublox::MsgId_CfgRxm,
            ublox::MsgId_CfgEkf,
            ublox::MsgId_CfgAnt,
            ublox::MsgId_CfgSbas,
            ublox::MsgId_CfgNmea,
            ublox::MsgId_CfgUsb,
            ublox::MsgId_CfgTmode,
            ublox::MsgId_CfgOdo,
            ublox::MsgId_CfgNvs,
            ublox::MsgId_CfgNavx5,
            ublox::MsgId_CfgNav5,
            ublox::MsgId_CfgEsfgwt,
            ublox::MsgId_CfgTp5,
            ublox::MsgId_CfgPm,
            ublox::MsgId_CfgRinv,
            ublox::MsgId_CfgItfm,
            ublox::MsgId_CfgPm2,
            ublox::MsgId_CfgTmode2,
            ublox::MsgId_CfgGnss,
            ublox::MsgId_CfgLogfilter,
            ublox::MsgId_CfgTxslot,
            ublox::MsgId_CfgPwr,
            ublox::MsgId_CfgHnr,
            ublox::MsgId_CfgEsrc,
            ublox::MsgId_CfgDosc,
            ublox::MsgId_CfgSmgr,
            ublox::MsgId_CfgGeofence,
            ublox::MsgId_CfgDgnss,
            ublox::MsgId_CfgTmode3,
            ublox::MsgId_CfgFixseed,
            ublox::MsgId_CfgDynseed,
            ublox::MsgId_CfgPms,
            ublox::MsgId_UpdSos,
            ublox::MsgId_MonIo,
            ublox::MsgId_MonVer,
            ublox::MsgId_MonMsgpp,
            ublox::MsgId_MonRxbuf,
            ublox::MsgId_MonTxbuf,
            ublox::MsgId_MonHw,
            ublox::MsgId_MonHw2,
            ublox::MsgId_MonRxr,
            ublox::MsgId_MonPatch,
            ublox::MsgId_MonGnss,
            ublox::MsgId_MonSmgr,
            ublox::MsgId_AidReq,
            ublox::MsgId_AidIni,
            ublox::MsgId_AidHui,
            ublox::MsgId_AidData,
            ublox::MsgId_AidAlm,
            ublox::MsgId_AidEph,
            ublox::MsgId_AidAlpsrv,
            ublox::MsgId_AidAop,
            ublox::MsgId_AidAlp,
            ublox::MsgId_TimTp,
            ublox::MsgId_TimTm2,
            ublox::MsgId_TimVrfy,
            ublox::MsgId_TimSvin,
            ublox::MsgId_TimDosc,
            ublox::MsgId_TimTos,
            ublox::MsgId_TimSmeas,
            ublox::MsgId_TimVcocal,
            ublox::MsgId_TimFchg,
            ublox::MsgId_TimHoc,
            ublox::MsgId_EsfMeas,
            ublox::MsgId_EsfRaw,
            ublox::MsgId_EsfStatus,
            ublox::MsgId_EsfIns,
            ublox::MsgId_MgaGps,
            ublox::MsgId_MgaGal,
            ublox::MsgId_MgaBds,
            ublox::MsgId_MgaQzss,
            ublox::MsgId_MgaGlo,
            ublox::MsgId_MgaAno,
            ublox::MsgId_MgaFlash,
            ublox::MsgId_MgaIni,
            ublox::MsgId_MgaAck,
            ublox::MsgId_MgaDbd,
            ublox::MsgId_LogErase,
            ublox::MsgId_LogString,
            ublox::MsgId_LogCreate,
            ublox::MsgId_LogInfo,
            ublox::MsgId_LogRetrieve,
            ublox::MsgId_LogRetrievepos,
            ublox::MsgId_LogRetrievestring,
            ublox::MsgId_LogFindtime,
            ublox::MsgId_LogRetrieveposextra,
            ublox::MsgId_SecSign,
            ublox::MsgId_SecUniqid,
            ublox::MsgId_HnrPvt
        };
    
        auto iter = std::find(std::begin(Values), std::end(Values), Base::value());
        if ((iter == std::end(Values)) || (*iter != Base::value())) {
            return false;
        }
    
        return true;
    }
};

} // namespace field

} // namespace ublox


