// Copyright (c) 2022-2023 Robert A. Alfieri
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// GDL90.h - single header file implementing GDL90 message encoding and decoding 
//
// Refer to README.md for usage.
//
#ifndef _GDL90_h
#define _GDL90_h

#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

class GDL90
{
public:
    GDL90( void );
    ~GDL90();

    // bool set this to true to abort when an error occurs rather than returning false
    bool abort_on_error;

    // Call this once to perform self-testing of these functions.
    // Be sure to check the return value to see if it passed.
    bool     self_test( void );               

    using RawBytes = std::vector<uint8_t>;

    // pack   - take encoded rawbytes and add escape sequences, CRC, and start/end delimiters so message is ready to send out
    // unpack - inverse; returns false if message is corrupted or has an incorrect format
    bool     pack(         RawBytes& packed, const RawBytes& unpacked );
    bool     unpack( const RawBytes& packed,       RawBytes& unpacked );

    // message types
    enum class MESSAGE_ID
    {
        HEARTBEAT                       = 0,
        INITIALIZATION                  = 2,
        UPLINK_DATA                     = 7,
        HEIGHT_ABOVE_TERRAIN            = 9,
        OWNERSHIP_REPORT                = 10,
        OWNERSHIP_GEOMETRIC_ALTITUDE    = 11,
        TRAFFIC_REPORT                  = 20,
        BASIC_UAT_REPORT                = 30,
        LONG_UAT_REPORT                 = 31,

        // popular extensions
        FOREFLIGHT                      = 0x65,
    };
    enum class MESSAGE_FOREFLIGHT_SUBID
    {
        ID                              = 0,
        AHRS                            = 1,
    };    

    // predecode helpers
    bool     id_decode( MESSAGE_ID& id, const RawBytes& unpacked );
    bool     foreflight_subid_decode( MESSAGE_FOREFLIGHT_SUBID& subid, const RawBytes& unpacked );

    // HEARTBEAT
    static constexpr uint32_t HEARTBEAT_STATUS_UAT_INITIALIZED_MASK = 0x0001;
    static constexpr uint32_t HEARTBEAT_STATUS_RATCS_MASK           = 0x0004;
    static constexpr uint32_t HEARTBEAT_STATUS_GPS_BAT_LOW_MASK     = 0x0008;
    static constexpr uint32_t HEARTBEAT_STATUS_ADDR_TYPE_MASK       = 0x0010;
    static constexpr uint32_t HEARTBEAT_STATUS_IDENT_MASK           = 0x0020;
    static constexpr uint32_t HEARTBEAT_STATUS_MAINT_REQD_MASK      = 0x0040;
    static constexpr uint32_t HEARTBEAT_STATUS_GPS_POS_VALID_MASK   = 0x0080;
    static constexpr uint32_t HEARTBEAT_STATUS_UAT_OK_MASK          = 0x0100;
    static constexpr uint32_t HEARTBEAT_STATUS_CSA_NOT_AVAIL_MASK   = 0x2000;
    static constexpr uint32_t HEARTBEAT_STATUS_CSA_REQUESTED_MASK   = 0x4000;
    static constexpr uint32_t HEARTBEAT_STATUS_ALLOWED_MASK         = HEARTBEAT_STATUS_UAT_INITIALIZED_MASK |
                                                                            HEARTBEAT_STATUS_RATCS_MASK |
                                                                            HEARTBEAT_STATUS_GPS_BAT_LOW_MASK |
                                                                            HEARTBEAT_STATUS_ADDR_TYPE_MASK |
                                                                            HEARTBEAT_STATUS_IDENT_MASK |
                                                                            HEARTBEAT_STATUS_MAINT_REQD_MASK |
                                                                            HEARTBEAT_STATUS_GPS_POS_VALID_MASK |
                                                                            HEARTBEAT_STATUS_UAT_OK_MASK |
                                                                            HEARTBEAT_STATUS_CSA_NOT_AVAIL_MASK |
                                                                            HEARTBEAT_STATUS_CSA_REQUESTED_MASK;
    static constexpr uint32_t HEARTBEAT_STATUS_DISALLOWED_MASK      = ~HEARTBEAT_STATUS_ALLOWED_MASK;

    bool     heartbeat_encode(       RawBytes& unpacked, uint32_t  status, uint32_t  timestamp, uint32_t  msg_count_uplink, uint32_t  msg_count_basic_and_long );
    bool     heartbeat_decode( const RawBytes& unpacked, uint32_t& status, uint32_t& timestamp, uint32_t& msg_count_uplink, uint32_t& msg_count_basic_and_long );

    // INITIALIZATION
    static constexpr uint32_t INIT_CONFIG_CDTI_OK_MASK              = 0x0001;
    static constexpr uint32_t INIT_CONFIG_AUDIO_INHIBIT_MASK        = 0x0002;
    static constexpr uint32_t INIT_CONFIG_AUDIO_TEST_MASK           = 0x0040;
    static constexpr uint32_t INIT_CONFIG_CSA_DISABLE_MASK          = 0x0100;
    static constexpr uint32_t INIT_CONFIG_CSA_AUDIO_DISABLE_MASK    = 0x0200;
    static constexpr uint32_t INIT_CONFIG_ALLOWED_MASK              = INIT_CONFIG_CDTI_OK_MASK |
                                                                            INIT_CONFIG_AUDIO_INHIBIT_MASK |
                                                                            INIT_CONFIG_AUDIO_TEST_MASK |
                                                                            INIT_CONFIG_CSA_DISABLE_MASK |
                                                                            INIT_CONFIG_CSA_AUDIO_DISABLE_MASK;
    static constexpr uint32_t INIT_CONFIG_DISALLOWED_MASK           = ~INIT_CONFIG_ALLOWED_MASK;

    bool     initialization_encode(       RawBytes& unpacked, uint32_t  config );
    bool     initialization_decode( const RawBytes& unpacked, uint32_t& config );

    // UPLINK_DATA, BASIC_UAT_REPORT, LONG_UAT_REPORT
    static constexpr uint32_t TIME_OF_RECEPTION_FRAC_ENCODED_INVALID = 0xffffff;
    bool     time_of_reception_frac_encode( uint32_t& frac_encoded, double  frac );  // frac is fractions of a second since the heartbeat timestamp
    bool     time_of_reception_frac_decode( uint32_t  frac_encoded, double& frac );  // frac set to NaN  if frac_encoded==TIME_OF_RECEPTION_FRAC_ENCODED_INVALID

    bool     uplink_data_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload );
    bool     uplink_data_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload );

    bool     basic_uat_report_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload );
    bool     basic_uat_report_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload );

    bool     long_uat_report_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload );
    bool     long_uat_report_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload );


    // OWNERSHIP_REPORT, TRAFFIC_REPORT
    enum class ALERT_STATUS
    {
        INACTIVE                = 0,
        ACTIVE                  = 1,
        __LAST                  = 1,
    };

    enum class ADDR_TYPE
    {
        ADSB_WITH_ICAO_ADDR     = 0,
        ADSB_WITH_SELF_ADDR     = 1,
        TISB_WITH_ICAO_ADDR     = 2,
        TISB_WITH_TRACK_FILE_ID = 3,
        SURFACE_VEHICLE         = 4,
        GROUND_STATION_BEACON   = 5,
        __LAST                  = 5,
    };

    static constexpr uint32_t MISC_TT_NOT_VALID_MASK               = 0b0000;
    static constexpr uint32_t MISC_TT_TRUE_TRACK_ANGLE_MASK        = 0b0001;
    static constexpr uint32_t MISC_TT_HEADING_MAGNETIC_MASK        = 0b0010;
    static constexpr uint32_t MISC_TT_HEADING_TRUE_MASK            = 0b0011;
    static constexpr uint32_t MISC_REPORT_UPDATED_MASK             = 0b0000;
    static constexpr uint32_t MISC_REPORT_EXTRAPOLATED_MASK        = 0b0100;
    static constexpr uint32_t MISC_ON_GROUND_MASK                  = 0b0000;
    static constexpr uint32_t MISC_AIRBORNE_MASK                   = 0b1000;
    static constexpr uint32_t MISC_ALLOWED_MASK                    = 0b1111;

    enum class NIC
    {
        UNKNOWN                 = 0,
        LT_20_0_NM              = 1,
        LT_8_0_NM               = 2,
        LT_4_0_NM               = 3,
        LT_2_0_NM               = 4,
        LT_1_0_NM               = 5,
        LT_0_6_NM               = 6,
        LT_0_2_NM               = 7,
        LT_0_1_NM               = 8,
        HPL_LT_75_VPL_LT_112    = 9,
        HPL_LT_25_VPL_LT_37_5   = 10,
        HPL_LT_7_5_VPL_LT_11    = 11,
        __LAST                  = 11,
    };

    enum class NACP
    {
        UNKNOWN                 = 0,
        LT_10_0_NM              = 1,
        LT_4_0_NM               = 2,
        LT_2_0_NM               = 3,
        LT_1_0_NM               = 4,
        LT_0_5_NM               = 5,
        LT_0_3_NM               = 6,
        LT_0_1_NM               = 7,
        LT_0_01_NM              = 8,
        HFOM_LT_30_VFOM_LT_45   = 9,
        HFOM_LT_10_VFOM_LT_15   = 10,
        HFOM_LT_3_VFOM_LT_4     = 11,
        __LAST                  = 11,
    };

    enum class EMITTER
    {
        UNKNOWN                 = 0,
        LIGHT                   = 1,
        SMALL                   = 2,
        LARGE                   = 3,
        HIGH_VORTEX_LARGE       = 4,
        HEAVY                   = 5,
        HIGHLY_MANEUVERABLE     = 6,
        ROTOCRAFT               = 7,
        GLIDER_SAILPLANE        = 8,
        LIGHTER_THAN_AIR        = 10,
        PARACHUTIST             = 11,
        ULTRA_LIGHT             = 12,
        UNMANNED                = 14,
        SPACE                   = 15,
        SURFACE_EMERGENCY       = 17,
        SURFACE_SERVICE         = 18,
        POINT_OBSTACLE          = 19,
        CLUSTER_OBSTACLE        = 20,
        LINE_OBSTACLE           = 21,
        __LAST                  = 21,
    };

    enum class EMERGENCY_PRIO
    {
        NO_EMERGENCY            = 0,
        GENERAL_EMERGENCY       = 1,
        MEDICAL_EMERGENCY       = 2,
        MINIMUM_FUEL            = 3,
        NO_COMMUNICATION        = 4,
        UNLAWFUL_INTERFERENCE   = 5,
        DOWNED_AIRCRAFT         = 6,
        __LAST                  = 6,
    };

    bool     latlon_encode( uint32_t& latlon_encoded, double  latlon );  // latlon must be -180.0 .. 180.0(minus LSB epsilon); north and east are considered positive
    bool     latlon_decode( uint32_t  latlon_encoded, double& latlon );  

    static constexpr uint32_t ALTITUDE_ENCODED_INVALID                = 0xfff;  
    bool     altitude_encode( uint32_t& altitude_encoded, double  altitude );  // altitude must be -1000 ft to +101,350 ft
    bool     altitude_decode( uint32_t  altitude_encoded, double& altitude );  // altitude is set to NaN if altitude_encoded==ALTITUDE_ENCODED_INVALID

    static constexpr uint32_t HORIZONTAL_VELOCITY_ENCODED_INVALID          = 0xfff;  
    bool     horizontal_velocity_encode( uint32_t& velocity_encoded, double  velocity );  // valid range is 0 .. 4094 knots; if exceeded, then it is hammered to 4094
    bool     horizontal_velocity_decode( uint32_t  velocity_encoded, double& velocity );  // velocity is set to NaN if velocity_encoded==HORIZONTAL_VELOCITY_ENCODED_INVALID

    static constexpr uint32_t VERTICAL_VELOCITY_ENCODED_INVALID          = 0x800;
    bool     vertical_velocity_encode( uint32_t& velocity_encoded, double  velocity );  // valid range is +/- 32,576 FPM; if exceeded, then it is hammered to +/- 32,640
    bool     vertical_velocity_decode( uint32_t  velocity_encoded, double& velocity );  // velocity is set to NaN if velocity_encoded==VERTICAL_VELOCITY_ENCODED_INVALID

    bool     track_hdg_encode( uint32_t& track_hdg_encoded, double  track_hdg );        // valid range is 0-360 deg
    bool     track_hdg_decode( uint32_t  track_hdg_encoded, double& track_hdg );        

    bool     is_valid_call_sign( std::string call_sign );

    bool     ownership_or_traffic_report_encode(       RawBytes& unpacked, bool is_ownership, ALERT_STATUS alert_status, ADDR_TYPE addr_type, uint32_t participant_address, 
                                                                                 uint32_t latitude, uint32_t longitude, uint32_t altitude, uint32_t misc, 
                                                                                 NIC nic, NACP nacp, uint32_t horiz_velocity, uint32_t vert_velocity, uint32_t track_hdg, 
                                                                                 EMITTER emitter, std::string call_sign, EMERGENCY_PRIO emergency_prio_code );
    bool     ownership_or_traffic_report_decode( const RawBytes& unpacked, bool is_ownership, ALERT_STATUS& alert_status, ADDR_TYPE& addr_type, uint32_t& participant_address, 
                                                                                 uint32_t& latitude, uint32_t& longitude, uint32_t& altitude, uint32_t& misc, 
                                                                                 NIC& nic, NACP& nacp, uint32_t& horiz_velocity, uint32_t& vert_velocity, uint32_t& track_hdg, 
                                                                                 EMITTER& emitter, std::string& call_sign, EMERGENCY_PRIO& emergency_prio_code );

    // HEIGHT_ABOVE_TERRAIN
    static constexpr uint32_t HEIGHT_ENCODED_INVALID = 0x80000;
    bool     height_encode( uint32_t& height_encoded, double  height );  // height must be -32767 .. 32767; if NaN, height_encoded is set to HEIGHT_ENCODED_INVALID
    bool     height_decode( uint32_t  height_encoded, double& height );  // height is set to NaN if height_encoded==HEIGHT_ENCODED_INVALID

    bool     height_above_terrain_encode(       RawBytes& unpacked, uint32_t  height );  
    bool     height_above_terrain_decode( const RawBytes& unpacked, uint32_t& height );  
    
    // OWNERSHIP_GEOMETRIC_ALTITUDE
    bool     geo_altitude_encode( uint32_t& geo_altitude_encoded, double  geo_altitude );  // geo_altitude must be -5*32768 .. 5*32767; 
    bool     geo_altitude_decode( uint32_t  geo_altitude_encoded, double& geo_altitude );  

    static constexpr uint32_t VERTICAL_FIGURE_OF_MERIT_NOT_AVAIL = 0x7fff;
    static constexpr uint32_t VERTICAL_FIGURE_OF_MERIT_GE_32766  = 0x7ffe;
    bool     vertical_figure_of_merit_encode( uint32_t& vertical_figure_of_merit_encoded, double  vertical_figure_of_merit ); // must be in meters and >= 0; NaN => NOT_AVAIL
    bool     vertical_figure_of_merit_decode( uint32_t  vertical_figure_of_merit_encoded, double& vertical_figure_of_merit ); // set to NaN if VERTICAL_FIGURE_OF_MERIT_NOT_AVAIL

    bool     ownership_geometric_altitude_encode(       RawBytes& unpacked, uint32_t  geo_altitude, bool  vertical_warning, uint32_t  vertical_figure_of_merit );
    bool     ownership_geometric_altitude_decode( const RawBytes& unpacked, uint32_t& geo_altitude, bool& vertical_warning, uint32_t& vertical_figure_of_merit );

    // FOREFLIGHT ID
    static constexpr uint32_t FOREFLIGHT_CAPABILITIES_GEO_ALTITUDE_USED_MASK = 0x00000001; // in OWNERSHIP_GEOMETRIC_ALTITUDE message
    static constexpr uint32_t FOREFLIGHT_CAPABILITIES_WGS84_ELLIPSOID_MASK   = 0x00000002; // per GDL90 spec
    static constexpr uint32_t FOREFLIGHT_CAPABILITIES_ALLOWED_MASK           = FOREFLIGHT_CAPABILITIES_GEO_ALTITUDE_USED_MASK |
                                                                               FOREFLIGHT_CAPABILITIES_WGS84_ELLIPSOID_MASK;
    static constexpr uint32_t FOREFLIGHT_CAPABILITIES_DISALLOWED_MASK        = ~FOREFLIGHT_CAPABILITIES_ALLOWED_MASK;

    static constexpr uint64_t FOREFLIGHT_DEVICE_SERIAL_NUMBER_INVALID = 0xffffffffffffffffULL;

    bool     foreflight_id_encode(       RawBytes& unpacked, uint64_t  device_serial_number, std::string  device_name, std::string  device_long_name, uint32_t  capabilities_mask );
    bool     foreflight_id_decode( const RawBytes& unpacked, uint64_t& device_serial_number, std::string& device_name, std::string& device_long_name, uint32_t& capabilities_mask );

    // FOREFLIGHT AHRS
    static constexpr uint32_t FOREFLIGHT_ROLL_PITCH_INVALID = 0x7fff;
    bool     foreflight_roll_pitch_encode( uint32_t& roll_pitch_encoded, double  roll_pitch );  // roll/pitch must be -180.0 .. 180.0; NaN => FOREFLIGHT_ROLL_PITCH_INVALID
    bool     foreflight_roll_pitch_decode( uint32_t  roll_pitch_encoded, double& roll_pitch );  // if FOREFLIGHT_ROLL_PITCH_INVALID, roll_pitch will be set to NaN

    static constexpr uint32_t FOREFLIGHT_HEADING_INVALID = 0xffff;
    bool     foreflight_heading_encode( uint32_t& heading_encoded, double  heading, bool  is_magnetic );  // heading must be -360.0 .. 360.0; NaN => FOREFLIGHT_HEADING_INVALID
    bool     foreflight_heading_decode( uint32_t  heading_encoded, double& heading, bool& is_magnetic );  // if FOREFLIGHT_HEADING_INVALID, heading will be set to NaN

    static constexpr uint32_t FOREFLIGHT_AIRSPEED_INVALID = 0xffff;

    bool     foreflight_ahrs_encode(       RawBytes& unpacked, uint32_t  roll, uint32_t  pitch, uint32_t  heading, uint32_t  ias, uint32_t  tas );
    bool     foreflight_ahrs_decode( const RawBytes& unpacked, uint32_t& roll, uint32_t& pitch, uint32_t& heading, uint32_t& ias, uint32_t& tas );

private:
    uint16_t crc_table[256]; 
    void     crc_init( void );                            // computes crc_table once
    uint16_t crc_compute( const RawBytes& unpacked, size_t length );

    inline bool error( std::string msg="would return false" )
    {
        if ( abort_on_error ) {
            std::cout << "ERROR: " << msg << "\n";
            exit( 1 );
        }
        return false;
    }
};

GDL90::GDL90( void )
{
   abort_on_error = false;
   crc_init();
}

GDL90::~GDL90()
{
}

bool GDL90::self_test( void )
{
    //-----------------------------------------------------------
    // Encode+pack and unpack+decode all message types.
    //-----------------------------------------------------------
    RawBytes unpacked;
    RawBytes packed;

    // PACK/UNPACK (using raw CRC example in GDL90 section 2.2.4)
    unpacked = {0x00, 0x81, 0x41, 0xdb, 0xd0, 0x08, 0x02};
    if ( !pack( packed, unpacked ) ) return error();
    if ( packed[0] != 0x7e ) return error();
    if ( packed[8] != 0xb3 ) return error();
    if ( packed[9] != 0x8b ) return error();
    if ( packed[10] != 0x7e ) return error();
    RawBytes unpacked2;

    if ( !unpack( packed, unpacked2 ) ) return error();
    if ( unpacked2.size() != unpacked.size() ) return error();
    for( size_t i = 0; i < unpacked2.size(); i++ ) 
    {
        if ( unpacked2[i] != unpacked[i] ) return error();
    }

    // HEARTBEAT
    uint32_t status = HEARTBEAT_STATUS_ALLOWED_MASK & 0xaa55;
    uint32_t timestamp = 0x1aa55;
    uint32_t msg_count_uplink = 29;
    uint32_t msg_count_basic_and_long = 1020;
    if ( !heartbeat_encode( unpacked, status, timestamp, msg_count_uplink, msg_count_basic_and_long ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();
    
    if ( !unpack( packed, unpacked ) ) return error();
    if ( !heartbeat_decode( unpacked, status, timestamp, msg_count_uplink, msg_count_basic_and_long ) ) return error();
    if ( status != (HEARTBEAT_STATUS_ALLOWED_MASK & 0xaa55) ) return error();
    if ( timestamp != 0x1aa55 ) return error();
    if ( msg_count_uplink != 29 ) return error();
    if ( msg_count_basic_and_long != 1020 ) return error();

    // INITIALIATION
    uint32_t config = INIT_CONFIG_ALLOWED_MASK & 0xaa55;
    if ( !initialization_encode( unpacked, config ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !initialization_decode( unpacked, config ) ) return error();
    if ( config != (INIT_CONFIG_ALLOWED_MASK & 0xaa55) ) return error();

    // UPLINK_DATA
    double   time_of_reception_frac_f = 0.99999992;
    uint32_t time_of_reception_frac;
    if ( !time_of_reception_frac_encode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    if ( !time_of_reception_frac_decode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    if ( std::abs( time_of_reception_frac_f - 0.99999992 ) > 0x00000001 ) return error();
    RawBytes payload;
    for( uint32_t i = 0; i < 432; i++ )
    {
        payload.push_back( i & 0xff );
    }
    if ( !uplink_data_encode( unpacked, time_of_reception_frac, payload ) ) return error();
    payload.clear();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !uplink_data_decode( unpacked, time_of_reception_frac, payload ) ) return error();
    if ( !time_of_reception_frac_decode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    if ( std::abs( time_of_reception_frac_f - 0.99999992 ) > 0x00000001 ) return error();
    if ( payload.size() != 432 ) return error();
    for( uint32_t i = 0; i < payload.size(); i++ ) 
    {
        if ( payload[i] != (i & 0xff) ) return error();
    }

    // BASIC_UAT_REPORT
    time_of_reception_frac_f = 0.99999992;
    if ( !time_of_reception_frac_encode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    payload.clear();
    for( uint32_t i = 0; i < 18; i++ )
    {
        payload.push_back( i & 0xff );
    }
    if ( !basic_uat_report_encode( unpacked, time_of_reception_frac, payload ) ) return error();
    payload.clear();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !basic_uat_report_decode( unpacked, time_of_reception_frac, payload ) ) return error();
    if ( !time_of_reception_frac_decode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    if ( std::abs( time_of_reception_frac_f - 0.99999992 ) > 0x00000001 ) return error();
    if ( payload.size() != 18 ) return error();
    for( uint32_t i = 0; i < payload.size(); i++ ) 
    {
        if ( payload[i] != (i & 0xff) ) return error();
    }

    // LONG_UAT_REPORT
    time_of_reception_frac_f = 0.99999992;
    if ( !time_of_reception_frac_encode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    payload.clear();
    for( uint32_t i = 0; i < 34; i++ )
    {
        payload.push_back( i & 0xff );
    }
    if ( !long_uat_report_encode( unpacked, time_of_reception_frac, payload ) ) return error();
    payload.clear();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !long_uat_report_decode( unpacked, time_of_reception_frac, payload ) ) return error();
    if ( !time_of_reception_frac_decode( time_of_reception_frac, time_of_reception_frac_f ) ) return error();
    if ( std::abs( time_of_reception_frac_f - 0.99999992 ) > 0x00000001 ) return error();
    if ( payload.size() != 34 ) return error();
    for( uint32_t i = 0; i < payload.size(); i++ ) 
    {
        if ( payload[i] != (i & 0xff) ) return error();
    }

    // OWNERSHIP_REPORT or TRAFFIC_REPORT
    bool           is_ownership = true;
    ALERT_STATUS   alert_status = ALERT_STATUS::__LAST;
    ADDR_TYPE      addr_type = ADDR_TYPE::__LAST;
    uint32_t       participant_address = 0xffaa55;
    double         latitude_f = -179.2255;
    double         longitude_f = +179.4357; 
    double         altitude_f = 101349;
    uint32_t       misc = MISC_ALLOWED_MASK;
    NIC            nic = NIC::__LAST;
    NACP           nacp = NACP::__LAST;
    double         horiz_velocity_f = 125.4462;
    double         vert_velocity_f = -800.333;
    double         track_hdg_f = 358.3674;
    EMITTER        emitter = EMITTER::__LAST;
    std::string    call_sign = "N53587";
    EMERGENCY_PRIO emergency_prio_code = EMERGENCY_PRIO::__LAST;
    uint32_t       latitude;
    uint32_t       longitude;
    uint32_t       altitude;
    uint32_t       horiz_velocity;
    uint32_t       vert_velocity;
    uint32_t       track_hdg;
    if ( !latlon_encode( latitude, latitude_f ) ) return error();
    if ( !latlon_encode( longitude, longitude_f ) ) return error();
    if ( !altitude_encode( altitude, altitude_f ) ) return error();
    if ( !horizontal_velocity_encode( horiz_velocity, horiz_velocity_f ) ) return error();
    if ( !vertical_velocity_encode( vert_velocity, vert_velocity_f ) ) return error();
    if ( !track_hdg_encode( track_hdg, track_hdg_f ) ) return error();
    if ( !ownership_or_traffic_report_encode( unpacked, is_ownership, alert_status, addr_type, participant_address, 
                                              latitude, longitude, altitude, misc, nic, nacp, horiz_velocity, vert_velocity, track_hdg, 
                                              emitter, call_sign, emergency_prio_code ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !ownership_or_traffic_report_decode( unpacked, is_ownership, alert_status, addr_type, participant_address, 
                                              latitude, longitude, altitude, misc, nic, nacp, horiz_velocity, vert_velocity, track_hdg, 
                                              emitter, call_sign, emergency_prio_code ) ) return error();
    if ( !latlon_decode( latitude, latitude_f ) ) return error();
    if ( !latlon_decode( longitude, longitude_f ) ) return error();
    if ( !altitude_decode( altitude, altitude_f ) ) return error();
    if ( !horizontal_velocity_decode( horiz_velocity, horiz_velocity_f ) ) return error();
    if ( !vertical_velocity_decode( vert_velocity, vert_velocity_f ) ) return error();
    if ( !track_hdg_decode( track_hdg, track_hdg_f ) ) return error();
    if ( alert_status != ALERT_STATUS::__LAST ) return error();
    if ( addr_type != ADDR_TYPE::__LAST ) return error();
    if ( participant_address != 0xffaa55 ) return error();
    if ( std::abs( latitude_f - -179.2255 ) > 0.0001 ) return error();
    if ( std::abs( longitude_f - 179.4357 ) > 0.0001 ) return error();
    if ( std::abs( altitude_f - 101349 ) < 1 ) return error();
    if ( misc != MISC_ALLOWED_MASK ) return error();
    if ( nic != NIC::__LAST ) return error();
    if ( nacp != NACP::__LAST ) return error();
    if ( std::abs( horiz_velocity_f - 125.4462 ) > 1.0 ) return error();
    if ( std::abs( vert_velocity_f - -800.333 ) > 64.0 ) return error();
    if ( std::abs( track_hdg_f - 358.3674 ) > 2.0 ) return error();
    if ( emitter != EMITTER::__LAST ) return error();
    if ( call_sign != "N53587  " ) return error();
    if ( emergency_prio_code != EMERGENCY_PRIO::__LAST ) return error();

    // TRAFFIC_REPORT (raw example from GDL90 section 3.5.2)
    const RawBytes unpacked_expected = { 0x14, 0x00, 0xab, 0x45, 0x49, 0x1f, 0xef, 0x15, 0xa8, 0x89, 0x78, 0x0f, 0x09, 0xa9,
                                         0x07, 0xb0, 0x01, 0x20, 0x01, 0x4e, 0x38, 0x32, 0x35, 0x56, 0x20, 0x20, 0x20, 0x00 };
    is_ownership = false;
    alert_status = ALERT_STATUS::INACTIVE;
    addr_type = ADDR_TYPE::ADSB_WITH_ICAO_ADDR;
    participant_address = 052642511;
    latitude_f = 44.90708;
    longitude_f = -122.99488;
    altitude_f = 5000;
    misc = MISC_TT_TRUE_TRACK_ANGLE_MASK | MISC_AIRBORNE_MASK;
    nic = NIC::HPL_LT_25_VPL_LT_37_5;
    nacp = NACP::HFOM_LT_30_VFOM_LT_45;
    horiz_velocity_f = 123.0;
    vert_velocity_f = 64.0;
    track_hdg_f = 45.0;
    emitter = EMITTER::LIGHT;
    call_sign = "N825V";
    emergency_prio_code = EMERGENCY_PRIO::NO_EMERGENCY;
    if ( !latlon_encode( latitude, latitude_f ) ) return error();
    if ( !latlon_encode( longitude, longitude_f ) ) return error();
    if ( !altitude_encode( altitude, altitude_f ) ) return error();
    if ( !horizontal_velocity_encode( horiz_velocity, horiz_velocity_f ) ) return error();
    if ( !vertical_velocity_encode( vert_velocity, vert_velocity_f ) ) return error();
    if ( !track_hdg_encode( track_hdg, track_hdg_f ) ) return error();
    if ( !ownership_or_traffic_report_encode( unpacked, is_ownership, alert_status, addr_type, participant_address, 
                                              latitude, longitude, altitude, misc, nic, nacp, horiz_velocity, vert_velocity, track_hdg, 
                                              emitter, call_sign, emergency_prio_code ) ) return error();
    if ( unpacked.size() != unpacked_expected.size() ) return error();
    for( size_t i = 0; i < unpacked.size(); i++ )
    {
        if ( unpacked[i] != unpacked_expected[i] ) return error();
    }

    // HEIGHT_ABOVE_TERRAIN
    double height_f = -32767.0;
    uint32_t height = 0xffffffff;
    if ( !height_encode( height, height_f ) ) return error();
    if ( !height_above_terrain_encode( unpacked, height ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !height_above_terrain_decode( unpacked, height ) ) return error();
    if ( !height_decode( height, height_f ) ) return error();
    if ( height_f != -32767.0 ) return error();

    // OWNERSHIP_GEOMETRIC_ALTITUDE
    double geo_altitude_f = -5*32765;
    uint32_t geo_altitude;
    bool vertical_warning = true;
    double vertical_figure_of_merit_f = 32765;
    uint32_t vertical_figure_of_merit;
    if ( !geo_altitude_encode( geo_altitude, geo_altitude_f ) ) return error();
    if ( !vertical_figure_of_merit_encode( vertical_figure_of_merit, vertical_figure_of_merit_f ) ) return error();
    if ( !ownership_geometric_altitude_encode( unpacked, geo_altitude, vertical_warning, vertical_figure_of_merit ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !ownership_geometric_altitude_decode( unpacked, geo_altitude, vertical_warning, vertical_figure_of_merit ) ) return error();
    if ( !geo_altitude_decode( geo_altitude, geo_altitude_f ) ) return error();
    if ( !vertical_figure_of_merit_decode( vertical_figure_of_merit, vertical_figure_of_merit_f ) ) return error();
    if ( geo_altitude_f != (-5*32765) ) return error();
    if ( !vertical_warning ) return error();
    if ( vertical_figure_of_merit_f != 32765 ) return error();

    // FOREFLIGHT ID
    uint64_t device_serial_number = 0xffaa55aa046655aaULL;
    std::string device_name = "iAviate ";
    std::string device_long_name = "iAviate a lot!";
    uint32_t capabilities_mask = FOREFLIGHT_CAPABILITIES_ALLOWED_MASK;
    if ( !foreflight_id_encode( unpacked, device_serial_number, device_name, device_long_name, capabilities_mask ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !foreflight_id_decode( unpacked, device_serial_number, device_name, device_long_name, capabilities_mask ) ) return error();
    if ( device_serial_number != 0xffaa55aa046655aaULL ) return error();
    if ( device_name != "iAviate " ) return error();
    if ( device_long_name != "iAviate a lot!  " ) return error();

    // FOREFLIGHT AHRS
    double roll_f = -179.6;
    double pitch_f = 179.7;
    double heading_f = -359.9;
    bool is_magnetic = true;
    uint32_t roll;
    uint32_t pitch;
    uint32_t heading;
    uint32_t ias = 129;
    uint32_t tas = 0x7fff;
    if ( !foreflight_roll_pitch_encode( roll, roll_f ) ) return error();
    if ( !foreflight_roll_pitch_encode( pitch, pitch_f ) ) return error();
    if ( !foreflight_heading_encode( heading, heading_f, is_magnetic ) ) return error();
    if ( !foreflight_ahrs_encode( unpacked, roll, pitch, heading, ias, tas ) ) return error();
    if ( !pack( packed, unpacked ) ) return error();

    if ( !unpack( packed, unpacked ) ) return error();
    if ( !foreflight_ahrs_decode( unpacked, roll, pitch, heading, ias, tas ) ) return error();
    if ( !foreflight_roll_pitch_decode( roll, roll_f ) ) return error();
    if ( !foreflight_roll_pitch_decode( pitch, pitch_f ) ) return error();
    if ( !foreflight_heading_decode( heading, heading_f, is_magnetic ) ) return error();
    if ( std::abs( roll_f - -179.6 ) > 0.01 ) return error();
    if ( std::abs( pitch_f - 179.7 ) > 0.01 ) return error();
    if ( std::abs( heading_f - -359.9 ) > 0.01 ) return error();
    if ( !is_magnetic ) return error();
    if ( ias != 129 ) return error();
    if ( tas != 0x7fff ) return error();

    // PASSED
    return true;
};

void GDL90::crc_init( void )
{
    for( uint32_t i = 0; i < 256; i++ )
    {
        uint16_t crc = i << 8;
        for( uint32_t b = 0; b < 8; b++ )
        {
            crc = (crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0);
        }
        crc_table[i] = crc;
    }
}

uint16_t GDL90::crc_compute( const RawBytes& unpacked, size_t length )
{
    uint16_t crc = 0;
    for( size_t i = 0; i < length; i++ )
    {
        crc = crc_table[crc >> 8] ^ (crc << 8) ^ unpacked[i];
    }
    return crc;
}

bool GDL90::pack( RawBytes& packed, const RawBytes& unpacked )
{
    //-----------------------------------------------------------
    // Compute CRC.
    //-----------------------------------------------------------
    uint16_t crc = crc_compute( unpacked, unpacked.size() );

    //-----------------------------------------------------------
    // Start and end message with 0x7e byte.
    // Any 0x7d or 0x7e byte in the message or CRC gets escaped with
    // 0x7e followed by byte^0x20.
    //-----------------------------------------------------------
    packed.clear();
    packed.push_back( 0x7e );
    for( size_t i = 0; i < unpacked.size(); i++ )
    {
        uint8_t byte = unpacked[i];
        if ( byte == 0x7d || byte == 0x7e ) {
            packed.push_back( 0x7d );
            byte ^= 0x20;
        }
        packed.push_back( byte );
    }
    for( uint32_t i = 0; i < 2; i++ )
    {
        uint8_t byte = crc & 0xff;
        crc >>= 8;
        if ( byte == 0x7d || byte == 0x7e ) {
            packed.push_back( 0x7d );
            byte ^= 0x20;
        }
        packed.push_back( byte );
    }
    packed.push_back( 0x7e );
    return true;
}

bool GDL90::unpack( const RawBytes& packed, RawBytes& unpacked )
{
    //-----------------------------------------------------------
    // Get rid of starting and ending 0x7e bytes.
    // Get rid of escape sequences.
    //-----------------------------------------------------------
    unpacked.clear();
    size_t size = packed.size();
    if ( size < 4 || packed[0] != 0x7e || packed[size-1] != 0x7e ) return error();
    for( size_t i = 1; i < size-1; i++ )
    {
        uint8_t byte = packed[i];
        if ( byte == 0x7d ) {
            i++;
            if ( i >= (size-1) ) return error();
            byte = packed[i] ^ 0x20;
        }
        unpacked.push_back( byte );
    }

    //-----------------------------------------------------------
    // Compute CRC on the part of the message that doesn't contain the CRC.
    //-----------------------------------------------------------
    size = unpacked.size();
    if ( size < 2 ) return error();
    uint16_t crc = (unpacked[size-1] << 8) | unpacked[size-2];
    uint16_t crc_expected = crc_compute( unpacked, size-2 );
    if ( crc != crc_expected ) return error();

    //-----------------------------------------------------------
    // Get rid of the CRC at the end.
    //-----------------------------------------------------------
    unpacked.resize( size-2 );
    return true;
}

bool GDL90::id_decode( MESSAGE_ID& id, const RawBytes& unpacked )
{
    if ( unpacked.size() == 0 ) return error();
    id = MESSAGE_ID( unpacked[0] );
    return id == MESSAGE_ID::HEARTBEAT            ||
           id == MESSAGE_ID::INITIALIZATION       ||
           id == MESSAGE_ID::UPLINK_DATA          ||
           id == MESSAGE_ID::OWNERSHIP_REPORT     ||
           id == MESSAGE_ID::OWNERSHIP_GEOMETRIC_ALTITUDE ||
           id == MESSAGE_ID::TRAFFIC_REPORT       ||
           id == MESSAGE_ID::BASIC_UAT_REPORT     ||
           id == MESSAGE_ID::LONG_UAT_REPORT      ||
           id == MESSAGE_ID::FOREFLIGHT;
}

bool GDL90::foreflight_subid_decode( MESSAGE_FOREFLIGHT_SUBID& subid, const RawBytes& unpacked )
{
    if ( unpacked.size() < 2 ) return error();
    MESSAGE_ID id = MESSAGE_ID( unpacked[0] );
    if ( id != MESSAGE_ID::FOREFLIGHT ) return error();
    subid = MESSAGE_FOREFLIGHT_SUBID( unpacked[1] );
    return subid == MESSAGE_FOREFLIGHT_SUBID::ID ||
           subid == MESSAGE_FOREFLIGHT_SUBID::AHRS;
}

bool GDL90::heartbeat_encode( RawBytes& unpacked, uint32_t  status, uint32_t  timestamp, uint32_t  msg_count_uplink, uint32_t  msg_count_basic_and_long )
{
    unpacked.clear();
    if ( (status & HEARTBEAT_STATUS_DISALLOWED_MASK) != 0 ) return error();
    if ( timestamp > 0x1ffff ) return error();
    if ( msg_count_uplink > 31 ) return error();
    if ( msg_count_basic_and_long > 1023 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::HEARTBEAT) );
    status |= ((timestamp >> 16) & 1) << 15;
    unpacked.push_back( (status >> 8) & 0xff );
    unpacked.push_back( (status >> 0) & 0xff );
    unpacked.push_back( (timestamp >> 8) & 0xff );
    unpacked.push_back( (timestamp >> 0) & 0xff );
    uint16_t msg_counts = (msg_count_uplink << 11) | msg_count_basic_and_long;
    unpacked.push_back( (msg_counts >> 8) & 0xff );
    unpacked.push_back( (msg_counts >> 0) & 0xff );
    return true;
}

bool GDL90::heartbeat_decode( const RawBytes& unpacked, uint32_t& status, uint32_t& timestamp, uint32_t& msg_count_uplink, uint32_t& msg_count_basic_and_long )
{
    if ( unpacked.size() != 7 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::HEARTBEAT) ) return error();
    status = unpacked[i++] << 8;
    status |= unpacked[i++] << 0;
    timestamp = (status >> 15) << 16;
    status &= 0x7fff;
    if ( (status & HEARTBEAT_STATUS_DISALLOWED_MASK) != 0 ) return error();
    timestamp |= unpacked[i++] << 8;
    timestamp |= unpacked[i++] << 0;
    uint16_t msg_counts = unpacked[i++] << 8;
    msg_counts |= unpacked[i++] << 0;
    msg_count_uplink = msg_counts >> 11;
    msg_count_basic_and_long = msg_counts & 0x3ff;
    return true;
}

bool GDL90::initialization_encode( RawBytes& unpacked, uint32_t  config )
{
    unpacked.clear();
    if ( (config & INIT_CONFIG_DISALLOWED_MASK) != 0 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::INITIALIZATION) );
    unpacked.push_back( (config >> 8) & 0xff );
    unpacked.push_back( (config >> 0) & 0xff );
    return true;
}

bool GDL90::initialization_decode( const RawBytes& unpacked, uint32_t& config )
{
    if ( unpacked.size() != 3 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::INITIALIZATION) ) return error();
    config = unpacked[i++] << 8;
    config |= unpacked[i++] << 0;
    if ( (config & INIT_CONFIG_DISALLOWED_MASK) != 0 ) return error();
    return true;
}

bool GDL90::time_of_reception_frac_encode( uint32_t& frac_encoded, double  frac ) 
{
    if ( std::isnan( frac ) ) {
        frac_encoded = TIME_OF_RECEPTION_FRAC_ENCODED_INVALID;
    } else { 
        if ( frac < 0.0 || frac >= 1.0 ) return error();
        frac_encoded = frac * 1000000000.0/80.0;
    }
    return true;
}

bool GDL90::time_of_reception_frac_decode( uint32_t  frac_encoded, double& frac )
{
    if ( frac_encoded > 0xffffff ) return error();
    if ( frac_encoded == TIME_OF_RECEPTION_FRAC_ENCODED_INVALID ) {
        frac = std::nanf("0");
    } else {
        frac = double(frac_encoded) * 80.0/1000000000.0;
        if ( frac >= 1.0 ) return error();
    }
    return true;
}

bool GDL90::uplink_data_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload )
{
    unpacked.clear();
    if ( time_of_reception_frac >= (1 << 24) ) return error();
    if ( payload.size() != 432 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::UPLINK_DATA) );
    unpacked.push_back( (time_of_reception_frac >> 16) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 8) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 0) & 0xff );
    for( uint32_t i = 0; i < payload.size(); i++ )
    {
        unpacked.push_back( payload[i] );
    }
    return true;
}

bool GDL90::uplink_data_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload )
{
    if ( unpacked.size() != 436 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::UPLINK_DATA) ) return error();
    time_of_reception_frac  = unpacked[i++] << 16;
    time_of_reception_frac |= unpacked[i++] << 8;
    time_of_reception_frac |= unpacked[i++] << 0;
    payload.clear();
    for( ; i < unpacked.size(); i++ )
    {
        payload.push_back( unpacked[i] );
    }
    return true;
}

bool GDL90::basic_uat_report_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload )
{
    unpacked.clear();
    if ( time_of_reception_frac >= (1 << 24) ) return error();
    if ( payload.size() != 18 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::BASIC_UAT_REPORT) );
    unpacked.push_back( (time_of_reception_frac >> 16) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 8) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 0) & 0xff );
    for( uint32_t i = 0; i < payload.size(); i++ )
    {
        unpacked.push_back( payload[i] );
    }
    return true;
}

bool GDL90::basic_uat_report_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload )
{
    if ( unpacked.size() != 22 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::BASIC_UAT_REPORT) ) return error();
    time_of_reception_frac  = unpacked[i++] << 16;
    time_of_reception_frac |= unpacked[i++] << 8;
    time_of_reception_frac |= unpacked[i++] << 0;
    payload.clear();
    for( ; i < unpacked.size(); i++ )
    {
        payload.push_back( unpacked[i] );
    }
    return true;
}

bool GDL90::long_uat_report_encode(       RawBytes& unpacked, uint32_t  time_of_reception_frac, const RawBytes& payload )
{
    unpacked.clear();
    if ( time_of_reception_frac >= (1 << 24) ) return error();
    if ( payload.size() != 34 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::LONG_UAT_REPORT) );
    unpacked.push_back( (time_of_reception_frac >> 16) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 8) & 0xff );
    unpacked.push_back( (time_of_reception_frac >> 0) & 0xff );
    for( uint32_t i = 0; i < payload.size(); i++ )
    {
        unpacked.push_back( payload[i] );
    }
    return true;
}

bool GDL90::long_uat_report_decode( const RawBytes& unpacked, uint32_t& time_of_reception_frac,       RawBytes& payload )
{
    if ( unpacked.size() != 38 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::LONG_UAT_REPORT) ) return error();
    time_of_reception_frac  = unpacked[i++] << 16;
    time_of_reception_frac |= unpacked[i++] << 8;
    time_of_reception_frac |= unpacked[i++] << 0;
    payload.clear();
    for( ; i < unpacked.size(); i++ )
    {
        payload.push_back( unpacked[i] );
    }
    return true;
}

bool GDL90::latlon_encode( uint32_t& latlon_encoded, double  latlon )
{
    if ( latlon < -180.0 || latlon >= 180.0 ) return error();
    latlon_encoded = latlon*double( 1 << 23 )/180.0;
    latlon_encoded &= 0xffffff;
    if ( latlon >= 0.0 && latlon_encoded >= 0x800000 ) return error();
    if ( latlon <  0.0 && latlon_encoded <  0x800000 ) return error();
    return true;
}

bool GDL90::latlon_decode( uint32_t latlon_encoded, double& latlon )
{
    int32_t latlon_encoded_s = latlon_encoded | ((latlon_encoded >= 0x800000) ? 0xff000000 : 0x00000000);
    latlon = double(latlon_encoded_s) * 180.0/double(1 << 23);
    return true;
}

bool GDL90::altitude_encode( uint32_t& altitude_encoded, double altitude )
{
    if ( std::isnan( altitude ) ) {
        altitude_encoded = ALTITUDE_ENCODED_INVALID;
    } else {
        if ( altitude < -1000.0 || altitude > 101350.0 ) return error();
        altitude_encoded = (altitude + 1000.0) / 25.0;
        if ( altitude_encoded == ALTITUDE_ENCODED_INVALID ) return error();
    }
    return true;
}

bool GDL90::altitude_decode( uint32_t  altitude_encoded, double& altitude )
{
    if ( altitude_encoded > 0xfff ) return error();
    if ( altitude_encoded == ALTITUDE_ENCODED_INVALID ) {
        altitude = std::nanf("1");
    } else {
        altitude = altitude_encoded*25.0 - 1000.0;
    }
    return true;
}

bool GDL90::horizontal_velocity_encode( uint32_t& velocity_encoded, double  velocity )
{
    if ( std::isnan( velocity ) ) {
        velocity_encoded = HORIZONTAL_VELOCITY_ENCODED_INVALID;
    } else {
        if ( velocity < 0.0 ) return error();
        if ( velocity >= 4094.0 ) velocity = 4094.0;
        velocity_encoded = velocity;
        if ( velocity_encoded == HORIZONTAL_VELOCITY_ENCODED_INVALID ) return error();
    }
    return true;
}

bool GDL90::horizontal_velocity_decode( uint32_t velocity_encoded, double& velocity )
{
    if ( velocity_encoded == VERTICAL_VELOCITY_ENCODED_INVALID ) {
        velocity = std::nanf("3");
    } else {
        velocity = velocity_encoded;
    }
    return true;
}

bool GDL90::vertical_velocity_encode( uint32_t& velocity_encoded, double  velocity )
{
    if ( std::isnan( velocity ) ) {
        velocity_encoded = VERTICAL_VELOCITY_ENCODED_INVALID;
    } else {
        if ( velocity <= -32640.0 ) velocity = -32640.0;
        if ( velocity >=  32640.0 ) velocity =  32640.0;
        velocity_encoded = velocity / 64.0;
        velocity_encoded &= 0xfff;
        if ( (velocity_encoded >= 0x1ff && velocity_encoded <= 0x7ff) || (velocity_encoded >= 0x800 && velocity_encoded <= 0xe01) ) return error();
    }
    return true;
}

bool GDL90::vertical_velocity_decode( uint32_t velocity_encoded, double& velocity )
{
    if ( velocity_encoded == VERTICAL_VELOCITY_ENCODED_INVALID ) {
        velocity = std::nanf("4");
    } else {
        if ( (velocity_encoded >= 0x1ff && velocity_encoded <= 0x7ff) || (velocity_encoded >= 0x801 && velocity_encoded <= 0xe01) ) return error();
        int32_t velocity_encoded_s = velocity_encoded | ((velocity_encoded >= 0x801) ? 0xfffff000 : 0x00000000);
        velocity = double(velocity_encoded_s) * 64.0;
    }
    return true;
}

bool GDL90::track_hdg_encode( uint32_t& track_hdg_encoded, double  track_hdg )
{
    if ( track_hdg < 0.0 || track_hdg > 360.0 ) return error();
    track_hdg_encoded = track_hdg * 256.0/360.0;
    if ( track_hdg_encoded == 0x100 ) track_hdg_encoded = 0x00;
    return true;
}

bool GDL90::track_hdg_decode( uint32_t  track_hdg_encoded, double& track_hdg )
{
    track_hdg = double(track_hdg_encoded) * 360.0 / 256.0;
    return true;
}

bool GDL90::is_valid_call_sign( std::string call_sign )
{
    if ( call_sign.length() > 8 ) return error();
    bool have_space = false;
    for( uint32_t i = 0; i < call_sign.length(); i++ ) 
    {
        char ch = call_sign[i];
        if ( have_space ) {
            if ( ch != ' ' ) return error();
        } else if ( ch == ' ' ) {
            have_space = true;
        } else {
            if ( (ch < '0' || ch > '9') && (ch < 'A' || ch > 'Z') ) return error();
        }
    }
    return true;
}

bool GDL90::ownership_or_traffic_report_encode( RawBytes& unpacked, bool is_ownership, ALERT_STATUS alert_status, ADDR_TYPE addr_type, uint32_t participant_address, 
                                                        uint32_t latitude, uint32_t longitude, uint32_t altitude, uint32_t misc, 
                                                        NIC nic, NACP nacp, uint32_t horiz_velocity, uint32_t vert_velocity, uint32_t track_hdg, 
                                                        EMITTER emitter, std::string call_sign, EMERGENCY_PRIO emergency_prio_code )
{
    unpacked.clear();
    if ( uint32_t(alert_status) > uint32_t(ALERT_STATUS::__LAST) ) return error();
    if ( uint32_t(addr_type) > uint32_t(ADDR_TYPE::__LAST) ) return error();
    if ( participant_address > 0xffffff ) return error();
    if ( latitude > 0xffffff ) return error();
    if ( longitude > 0xffffff ) return error();
    if ( altitude > 0xfff ) return error();
    if ( misc > 0xf ) return error();
    if ( uint32_t(nic) > uint32_t(NIC::__LAST) ) return error();
    if ( uint32_t(nacp) > uint32_t(NACP::__LAST) ) return error();
    if ( horiz_velocity > 0xfff ) return error();
    if ( vert_velocity > 0xfff ) return error();
    if ( track_hdg > 0xff ) return error();
    if ( uint32_t(emitter) > uint32_t(EMITTER::__LAST) ) return error();
    if ( !is_valid_call_sign( call_sign ) ) return error();
    if ( uint32_t(emergency_prio_code) > uint32_t(EMERGENCY_PRIO::__LAST) ) return error();
    unpacked.push_back( uint8_t( is_ownership ? MESSAGE_ID::OWNERSHIP_REPORT : MESSAGE_ID::TRAFFIC_REPORT ) );
    unpacked.push_back( (uint8_t(alert_status) << 4) | uint8_t(addr_type) );
    unpacked.push_back( (participant_address >> 16) & 0xff );
    unpacked.push_back( (participant_address >> 8) & 0xff );
    unpacked.push_back( (participant_address >> 0) & 0xff );
    unpacked.push_back( (latitude >> 16) & 0xff );
    unpacked.push_back( (latitude >> 8) & 0xff );
    unpacked.push_back( (latitude >> 0) & 0xff );
    unpacked.push_back( (longitude >> 16) & 0xff );
    unpacked.push_back( (longitude >> 8) & 0xff );
    unpacked.push_back( (longitude >> 0) & 0xff );
    unpacked.push_back( (altitude >> 4) & 0xff );
    unpacked.push_back( (((altitude >> 0) & 0xf) << 4) | misc );
    unpacked.push_back( (uint8_t(nic) << 4) | uint8_t(nacp) );
    unpacked.push_back( (horiz_velocity >> 4) & 0xff );
    unpacked.push_back( (((horiz_velocity >> 0) & 0xf) << 4) | ((vert_velocity >> 8) & 0xf) );
    unpacked.push_back( (vert_velocity >> 0) & 0xff );
    unpacked.push_back( track_hdg );
    unpacked.push_back( uint8_t(emitter) );
    for( uint32_t i = 0; i < 8; i++ )
    {
        char ch = (i < call_sign.length()) ? call_sign[i] : ' ';
        unpacked.push_back( ch );
    }
    unpacked.push_back( uint8_t(emergency_prio_code) << 4 );
    return true;
}

bool GDL90::ownership_or_traffic_report_decode( const RawBytes& unpacked, bool is_ownership, ALERT_STATUS& alert_status, ADDR_TYPE& addr_type, uint32_t& participant_address, 
                                                        uint32_t& latitude, uint32_t& longitude, uint32_t& altitude, uint32_t& misc, 
                                                        NIC& nic, NACP& nacp, uint32_t& horiz_velocity, uint32_t& vert_velocity, uint32_t& track_hdg, 
                                                        EMITTER& emitter, std::string& call_sign, EMERGENCY_PRIO& emergency_prio_code )
{
    if ( unpacked.size() != 28 ) return error();
    uint32_t i = 0;
    MESSAGE_ID id = MESSAGE_ID( unpacked[i++] );
    if ( (is_ownership && id != MESSAGE_ID::OWNERSHIP_REPORT) || (!is_ownership && id != MESSAGE_ID::TRAFFIC_REPORT) ) return error();
    is_ownership = id == MESSAGE_ID::OWNERSHIP_REPORT;
    uint8_t byte = unpacked[i++];
    alert_status = ALERT_STATUS( (byte >> 4) & 0xf );
    addr_type = ADDR_TYPE( byte & 0xf );
    participant_address  = unpacked[i++] << 16;
    participant_address |= unpacked[i++] << 8;
    participant_address |= unpacked[i++] << 0;
    latitude  = unpacked[i++] << 16;
    latitude |= unpacked[i++] << 8;
    latitude |= unpacked[i++] << 0;
    longitude  = unpacked[i++] << 16;
    longitude |= unpacked[i++] << 8;
    longitude |= unpacked[i++] << 0;
    altitude = unpacked[i++] << 4;
    byte = unpacked[i++];
    altitude |= (byte >> 4) & 0xf;
    misc = byte & 0xf;
    byte = unpacked[i++];
    nic = NIC( (byte >> 4) & 0xf );
    nacp = NACP( byte & 0xf );
    horiz_velocity = unpacked[i++] << 4;
    byte = unpacked[i++];
    horiz_velocity |= (byte >> 4) & 0xf;
    vert_velocity = ((byte >> 0) & 0xf) << 8;
    vert_velocity |= unpacked[i++];
    track_hdg = unpacked[i++];
    emitter = EMITTER( unpacked[i++] );
    call_sign = "";
    for( uint32_t c = 0; c < 8; c++ )
    {
        char ch = unpacked[i++];
        call_sign += std::string( 1, ch );
    }
    byte = unpacked[i++];
    emergency_prio_code = EMERGENCY_PRIO( (byte >> 4) & 0xf );
    return true;
}

bool GDL90::height_encode( uint32_t& height_encoded, double  height )
{
    if ( std::isnan( height ) ) {
        height_encoded = HEIGHT_ENCODED_INVALID;
    } else {
        if ( height < -32767.0 || height > 32767.0 ) return error();
        height_encoded = int32_t( height );
        height_encoded &= 0xffff;
    }
    return true;
}

bool GDL90::height_decode( uint32_t height_encoded, double& height )
{
    if ( height_encoded == HEIGHT_ENCODED_INVALID ) {
        height = std::nanf("4");
    } else {
        int32_t height_encoded_s = height_encoded | ((height_encoded >= 0x8000) ? 0xffff0000 : 0x00000000);
        height = height_encoded_s;
    }
    return true;
}

bool GDL90::height_above_terrain_encode(       RawBytes& unpacked, uint32_t  height )
{
    unpacked.clear();
    if ( height >= (1 << 16) ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::HEIGHT_ABOVE_TERRAIN) );
    unpacked.push_back( (height >> 8) & 0xff );
    unpacked.push_back( (height >> 0) & 0xff );
    return true;
}

bool GDL90::height_above_terrain_decode( const RawBytes& unpacked, uint32_t& height )
{
    if ( unpacked.size() != 3 ) return error();
    size_t i = 0; 
    if ( unpacked[i++] != uint8_t(MESSAGE_ID::HEIGHT_ABOVE_TERRAIN) ) return error();
    height = unpacked[i++] << 8;
    height |= unpacked[i++];
    return true;
}

bool GDL90::geo_altitude_encode( uint32_t& geo_altitude_encoded, double  geo_altitude )
{
    if ( geo_altitude < (-5.0*32768.0) || geo_altitude > (5.0*32767.0) ) return error(); 
    geo_altitude_encoded = geo_altitude / 5.0;
    geo_altitude_encoded &= 0xffff;
    return true;
}

bool GDL90::geo_altitude_decode( uint32_t  geo_altitude_encoded, double& geo_altitude )
{
    int32_t geo_altitude_encoded_s = geo_altitude_encoded | ((geo_altitude_encoded >= 0x8000) ? 0xffff0000 : 0x00000000);
    geo_altitude = double(geo_altitude_encoded_s) * 5.0;
    return true;
}

bool GDL90::vertical_figure_of_merit_encode( uint32_t& vertical_figure_of_merit_encoded, double  vertical_figure_of_merit )
{
    if ( std::isnan( vertical_figure_of_merit ) ) {
        vertical_figure_of_merit_encoded = VERTICAL_FIGURE_OF_MERIT_NOT_AVAIL;
    } else {
        if ( vertical_figure_of_merit < 0.0 ) return error();
        if ( vertical_figure_of_merit >= 32766.0 ) {
            vertical_figure_of_merit_encoded = VERTICAL_FIGURE_OF_MERIT_GE_32766;
        } else {
            vertical_figure_of_merit_encoded = vertical_figure_of_merit;
        }           
    }
    return true;
}

bool GDL90::vertical_figure_of_merit_decode( uint32_t  vertical_figure_of_merit_encoded, double& vertical_figure_of_merit )
{
    if ( vertical_figure_of_merit_encoded > 0x7fff ) return error();
    if ( vertical_figure_of_merit_encoded == VERTICAL_FIGURE_OF_MERIT_NOT_AVAIL ) {
        vertical_figure_of_merit = std::nanf("20");
    } else if ( vertical_figure_of_merit_encoded == VERTICAL_FIGURE_OF_MERIT_GE_32766 ) {
        vertical_figure_of_merit = 32766;
    } else {
        vertical_figure_of_merit = vertical_figure_of_merit_encoded;
    }
    return true;
}

bool GDL90::ownership_geometric_altitude_encode(       RawBytes& unpacked, uint32_t  geo_altitude, bool  vertical_warning, uint32_t  vertical_figure_of_merit )
{
    unpacked.clear();
    if ( geo_altitude > 0xffff ) return error();
    if ( vertical_figure_of_merit > 0x7fff ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::OWNERSHIP_GEOMETRIC_ALTITUDE) );
    unpacked.push_back( (geo_altitude >> 8) & 0xff );
    unpacked.push_back( (geo_altitude >> 0) & 0xff );
    vertical_figure_of_merit |= uint32_t(vertical_warning) << 15;
    unpacked.push_back( (vertical_figure_of_merit >> 8) & 0xff );
    unpacked.push_back( (vertical_figure_of_merit >> 0) & 0xff );
    return true;
}

bool GDL90::ownership_geometric_altitude_decode( const RawBytes& unpacked, uint32_t& geo_altitude, bool& vertical_warning, uint32_t& vertical_figure_of_merit )
{
    if ( unpacked.size() != 5 ) return error();
    uint32_t i = 0;
    MESSAGE_ID id = MESSAGE_ID( unpacked[i++] );
    if ( id != MESSAGE_ID::OWNERSHIP_GEOMETRIC_ALTITUDE ) return error();
    geo_altitude = unpacked[i++] << 8;
    geo_altitude |= unpacked[i++];
    vertical_figure_of_merit  = unpacked[i++] << 8;
    vertical_figure_of_merit |= unpacked[i++] << 0;
    vertical_warning = (vertical_figure_of_merit >> 15) & 0x1;
    vertical_figure_of_merit &= 0x7fff;
    return true;
}

bool GDL90::foreflight_id_encode(       RawBytes& unpacked, uint64_t  device_serial_number, std::string  device_name, std::string  device_long_name, uint32_t  capabilities_mask )
{
    unpacked.clear();
    if ( device_name.length() > 8 ) return error();
    if ( device_long_name.length() > 16 ) return error();
    if ( (capabilities_mask & FOREFLIGHT_CAPABILITIES_DISALLOWED_MASK) != 0 ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::FOREFLIGHT) );
    unpacked.push_back( uint8_t(MESSAGE_FOREFLIGHT_SUBID::ID) );
    unpacked.push_back( 1 );
    for( uint32_t b = 0; b < 8; b++ )
    {
        unpacked.push_back( (device_serial_number >> ((7-b)*8)) & 0xff );  // big endian
    }
    for( uint32_t c = 0; c < 8; c++ )
    {
        char ch = (c < device_name.length()) ? device_name[c] : ' '; 
        unpacked.push_back( ch );
    }
    for( uint32_t c = 0; c < 16; c++ )
    {
        char ch = (c < device_long_name.length()) ? device_long_name[c] : ' '; 
        unpacked.push_back( ch );
    }
    for( uint32_t b = 0; b < 4; b++ )
    {
        unpacked.push_back( (capabilities_mask >> ((3-b)*8)) & 0xff );  // big endian
    }
    return true;
}

bool GDL90::foreflight_id_decode( const RawBytes& unpacked, uint64_t& device_serial_number, std::string& device_name, std::string& device_long_name, uint32_t& capabilities_mask )
{
    if ( unpacked.size() != 39 ) return error();
    uint32_t i = 0;
    MESSAGE_ID id = MESSAGE_ID( unpacked[i++] );
    if ( id != MESSAGE_ID::FOREFLIGHT ) return error();
    MESSAGE_FOREFLIGHT_SUBID subid = MESSAGE_FOREFLIGHT_SUBID( unpacked[i++] );
    if ( subid != MESSAGE_FOREFLIGHT_SUBID::ID ) return error();
    if ( unpacked[i++] != 1 ) return error();
    device_serial_number = 0;
    for( uint32_t b = 0; b < 8; b++ )
    {
        device_serial_number |= uint64_t(unpacked[i++]) << ((7-b)*8);
    }
    device_name = "";
    for( uint32_t c = 0; c < 8; c++ )
    {
        char ch = unpacked[i++];
        device_name += std::string( 1, ch );
    }
    device_long_name = "";
    for( uint32_t c = 0; c < 16; c++ )
    {
        char ch = unpacked[i++];
        device_long_name += std::string( 1, ch );
    }
    capabilities_mask = 0;
    for( uint32_t b = 0; b < 4; b++ )
    {
        capabilities_mask |= uint32_t(unpacked[i++]) << ((3-b)*8);
    }
    if ( (capabilities_mask & FOREFLIGHT_CAPABILITIES_DISALLOWED_MASK) != 0 ) return error();
    return true;
}

bool GDL90::foreflight_roll_pitch_encode( uint32_t& roll_pitch_encoded, double  roll_pitch )
{
    if ( std::isnan( roll_pitch ) ) {
        roll_pitch_encoded = FOREFLIGHT_ROLL_PITCH_INVALID;
    } else {
        if ( roll_pitch < -180.0 || roll_pitch > 180.0 ) return error();
        roll_pitch_encoded = roll_pitch * 10.0;
        roll_pitch_encoded &= 0xffff;
    }
    return true;
}

bool GDL90::foreflight_roll_pitch_decode( uint32_t  roll_pitch_encoded, double& roll_pitch )
{
    if ( roll_pitch_encoded == FOREFLIGHT_ROLL_PITCH_INVALID ) {
        roll_pitch = std::nanf("21");
    } else {
        if ( roll_pitch_encoded > 0xffff ) return error();
        int32_t roll_pitch_encoded_s = roll_pitch_encoded | ((roll_pitch_encoded >= 0x8000) ? 0xffff0000 : 0x00000000);
        roll_pitch = double(roll_pitch_encoded_s) / 10.0;
        if ( roll_pitch < -180.0 || roll_pitch > 180.0 ) return error();
    }
    return true;
}

bool GDL90::foreflight_heading_encode( uint32_t& heading_encoded, double  heading, bool is_magnetic )
{
    if ( std::isnan( heading ) ) {
        heading_encoded = FOREFLIGHT_HEADING_INVALID;
    } else {
        if ( heading < -360.0 || heading > 360.0 ) return error();
        heading_encoded = heading * 10.0;
        heading_encoded &= 0x7fff;
        heading_encoded |= is_magnetic << 15;
    }
    return true;
}

bool GDL90::foreflight_heading_decode( uint32_t  heading_encoded, double& heading, bool& is_magnetic )
{
    if ( heading_encoded == FOREFLIGHT_ROLL_PITCH_INVALID ) {
        heading = std::nanf("22");
        is_magnetic = false;
    } else {
        if ( heading_encoded > 0xffff ) return error();
        is_magnetic = (heading_encoded >> 15) & 1;
        heading_encoded &= 0x7fff;
        int32_t heading_encoded_s = heading_encoded | ((heading_encoded >= 0x4000) ? 0xffff8000 : 0x00000000);
        heading = double(heading_encoded_s) / 10.0;
        if ( heading < -360.0 || heading > 360.0 ) return error();
    }
    return true;
}

bool GDL90::foreflight_ahrs_encode(       RawBytes& unpacked, uint32_t  roll, uint32_t  pitch, uint32_t  heading, uint32_t  ias, uint32_t  tas )
{
    unpacked.clear();
    int32_t roll_s  = roll  | ((roll  >= 0x8000) ? 0xffff0000 : 0x00000000);
    int32_t pitch_s = pitch | ((pitch >= 0x8000) ? 0xffff0000 : 0x00000000);
    int32_t heading_s = heading & 0x7fff;
    if ( heading_s > 0x4000) heading_s |= 0xffff8000;
    if ( roll != FOREFLIGHT_ROLL_PITCH_INVALID && (roll_s < -1800 || roll_s > 1800) ) return error();
    if ( pitch != FOREFLIGHT_ROLL_PITCH_INVALID && (pitch_s < -1800 || pitch_s > 1800) ) return error();
    if ( heading != FOREFLIGHT_HEADING_INVALID && (heading_s < -3600 || heading_s > 3600) ) return error();
    unpacked.push_back( uint8_t(MESSAGE_ID::FOREFLIGHT) );
    unpacked.push_back( uint8_t(MESSAGE_FOREFLIGHT_SUBID::AHRS) );
    unpacked.push_back( (roll >> 8) & 0xff );
    unpacked.push_back( (roll >> 0) & 0xff );
    unpacked.push_back( (pitch >> 8) & 0xff );
    unpacked.push_back( (pitch >> 0) & 0xff );
    unpacked.push_back( (heading >> 8) & 0xff );
    unpacked.push_back( (heading >> 0) & 0xff );
    unpacked.push_back( (ias >> 8) & 0xff );
    unpacked.push_back( (ias >> 0) & 0xff );
    unpacked.push_back( (tas >> 8) & 0xff );
    unpacked.push_back( (tas >> 0) & 0xff );
    return true;
}

bool GDL90::foreflight_ahrs_decode( const RawBytes& unpacked, uint32_t& roll, uint32_t& pitch, uint32_t& heading, uint32_t& ias, uint32_t& tas )
{
    if ( unpacked.size() != 12 ) return error();
    uint32_t i = 0;
    MESSAGE_ID id = MESSAGE_ID( unpacked[i++] );
    if ( id != MESSAGE_ID::FOREFLIGHT ) return error();
    MESSAGE_FOREFLIGHT_SUBID subid = MESSAGE_FOREFLIGHT_SUBID( unpacked[i++] );
    if ( subid != MESSAGE_FOREFLIGHT_SUBID::AHRS ) return error();
    roll  = unpacked[i++] << 8;
    roll |= unpacked[i++] << 0;
    pitch  = unpacked[i++] << 8;
    pitch |= unpacked[i++] << 0;
    heading  = unpacked[i++] << 8;
    heading |= unpacked[i++] << 0;
    ias  = unpacked[i++] << 8;
    ias |= unpacked[i++] << 0;
    tas  = unpacked[i++] << 8;
    tas |= unpacked[i++] << 0;
    return true;
}

#endif
