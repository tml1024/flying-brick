// -*- comment-column: 50; fill-column: 110; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*-

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#include "MSFS/MSFS.h"
#include "MSFS/MSFS_Render.h"
#include "MSFS/MSFS_WindowsTypes.h"
#include "SimConnect.h"
#pragma GCC diagnostic pop

#include "minIni.h"

#include "ThisAircraft.h"

static HANDLE hSimConnect = 0;

static double static_cg_height;

static constexpr bool verbose = true;

// As soon as an API call fails or we get an exception we are in an unknown state and it is not worth
// to continue attempting to do anything.
static bool failed = false;

static bool simPaused = false;                    // Whether the "Paused" event with value 1 has been received
static bool gotFirstState = false;
static bool simFrozen = false;                    // Whether we are controlling the aircraft or not
static bool landing = false;
static bool takingOff = false;
static bool ignitionSwitch = false;               // Whether the ignition switch was last seen on or off

// Use different numeric ranges for the enums to recognize the values if they show up in unexpected places

enum DataDefinition : SIMCONNECT_DATA_DEFINITION_ID {
    DataDefinitionMutableState = 1000,
    DataDefinitionAllState,
};

enum Event : SIMCONNECT_CLIENT_EVENT_ID {
    EventPause = 2000,
    EventFreezeAltSet,
    EventFreezeAttSet,
    EventFreezePosSet,
    EventParkingBrakeToggle
};

enum Request : DWORD {
    RequestAllState = 3000,
};

enum Group : SIMCONNECT_NOTIFICATION_GROUP_ID {
};

struct ReadonlyState {
    double rudder;
    double aileron;
    double elevator;
    double throttle;
    double agl;
    double velWindX, velWindY, velWindZ;
    int64_t onGround;
    int64_t ignitionSwitch;
    double parkingBrake;
    int64_t altFreeze, attFreeze, posFreeze;
    double pressure;
};

struct MutableState {
    double heading;
    double bank, pitch;
    double lat, lon, msl;

    double velBodyX, velBodyY, velBodyZ;
    double velWorldX, velWorldY, velWorldZ;

    // Instrument indications (only?)
    double kias, ktas;
    double vs;
};

struct AllState {
    ReadonlyState readonly;
    MutableState state;
};

// Helper functions, don't warn if not used
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

static constexpr double deg2rad(double deg) {
    return deg / 180 * M_PI;
}

static constexpr double rad2deg(double rad) {
    return rad / M_PI * 180;
}

static constexpr double fps2fpm(double fps) {
    return fps * 60;
}

static constexpr double fpm2fps(double fpm) {
    return fpm / 60;
}

static constexpr double fps2kn(double fps) {
    return fps * 0.592484;
}

static constexpr double kn2fps(double kn) {
    return kn * 1.68781042021544;
}

static constexpr double ft2m(double ft) {
    return ft * (12 * 0.0254);
}

static constexpr double m2ft(double m) {
    return m / (12 * 0.0254);
}

static std::string exception_type(int exception) {
    switch (exception) {
    case SIMCONNECT_EXCEPTION_NONE:
        return "NONE";
    case SIMCONNECT_EXCEPTION_ERROR:
        return "ERROR";
    case SIMCONNECT_EXCEPTION_SIZE_MISMATCH:
        return "SIZE_MISMATCH";
    case SIMCONNECT_EXCEPTION_UNRECOGNIZED_ID:
        return "UNRECOGNIZED_ID";
    case SIMCONNECT_EXCEPTION_UNOPENED:
        return "UNOPENED";
    case SIMCONNECT_EXCEPTION_VERSION_MISMATCH:
        return "VERSION_MISMATCH";
    case SIMCONNECT_EXCEPTION_TOO_MANY_GROUPS:
        return "TOO_MANY_GROUPS";
    case SIMCONNECT_EXCEPTION_NAME_UNRECOGNIZED:
        return "NAME_UNRECOGNIZED";
    case SIMCONNECT_EXCEPTION_TOO_MANY_EVENT_NAMES:
        return "TOO_MANY_EVENT_NAMES";
    case SIMCONNECT_EXCEPTION_EVENT_ID_DUPLICATE:
        return "EVENT_ID_DUPLICATE";
    case SIMCONNECT_EXCEPTION_TOO_MANY_MAPS:
        return "TOO_MANY_MAPS";
    case SIMCONNECT_EXCEPTION_TOO_MANY_OBJECTS:
        return "TOO_MANY_OBJECTS";
    case SIMCONNECT_EXCEPTION_TOO_MANY_REQUESTS:
        return "TOO_MANY_REQUESTS";
    case SIMCONNECT_EXCEPTION_WEATHER_INVALID_PORT:
        return "WEATHER_INVALID_PORT";
    case SIMCONNECT_EXCEPTION_WEATHER_INVALID_METAR:
        return "WEATHER_INVALID_METAR";
    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_GET_OBSERVATION:
        return "WEATHER_UNABLE_TO_GET_OBSERVATION";
    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_CREATE_STATION:
        return "WEATHER_UNABLE_TO_CREATE_STATION";
    case SIMCONNECT_EXCEPTION_WEATHER_UNABLE_TO_REMOVE_STATION:
        return "WEATHER_UNABLE_TO_REMOVE_STATION";
    case SIMCONNECT_EXCEPTION_INVALID_DATA_TYPE:
        return "INVALID_DATA_TYPE";
    case SIMCONNECT_EXCEPTION_INVALID_DATA_SIZE:
        return "INVALID_DATA_SIZE";
    case SIMCONNECT_EXCEPTION_DATA_ERROR:
        return "DATA_ERROR";
    case SIMCONNECT_EXCEPTION_INVALID_ARRAY:
        return "INVALID_ARRAY";
    case SIMCONNECT_EXCEPTION_CREATE_OBJECT_FAILED:
        return "CREATE_OBJECT_FAILED";
    case SIMCONNECT_EXCEPTION_LOAD_FLIGHTPLAN_FAILED:
        return "LOAD_FLIGHTPLAN_FAILED";
    case SIMCONNECT_EXCEPTION_OPERATION_INVALID_FOR_OBJECT_TYPE:
        return "OPERATION_INVALID_FOR_OBJECT_TYPE";
    case SIMCONNECT_EXCEPTION_ILLEGAL_OPERATION:
        return "ILLEGAL_OPERATION";
    case SIMCONNECT_EXCEPTION_ALREADY_SUBSCRIBED:
        return "ALREADY_SUBSCRIBED";
    case SIMCONNECT_EXCEPTION_INVALID_ENUM:
        return "INVALID_ENUM";
    case SIMCONNECT_EXCEPTION_DEFINITION_ERROR:
        return "DEFINITION_ERROR";
    case SIMCONNECT_EXCEPTION_DUPLICATE_ID:
        return "DUPLICATE_ID";
    case SIMCONNECT_EXCEPTION_DATUM_ID:
        return "DATUM_ID";
    case SIMCONNECT_EXCEPTION_OUT_OF_BOUNDS:
        return "OUT_OF_BOUNDS";
    case SIMCONNECT_EXCEPTION_ALREADY_CREATED:
        return "ALREADY_CREATED";
    case SIMCONNECT_EXCEPTION_OBJECT_OUTSIDE_REALITY_BUBBLE:
        return "OBJECT_OUTSIDE_REALITY_BUBBLE";
    case SIMCONNECT_EXCEPTION_OBJECT_CONTAINER:
        return "OBJECT_CONTAINER";
    case SIMCONNECT_EXCEPTION_OBJECT_AI:
        return "OBJECT_AI";
    case SIMCONNECT_EXCEPTION_OBJECT_ATC:
        return "OBJECT_ATC";
    case SIMCONNECT_EXCEPTION_OBJECT_SCHEDULE:
        return "OBJECT_SCHEDULE";
    default:
        assert(false);
    }
}

static std::string simobject_type(int type) {
    switch (type) {
    case SIMCONNECT_SIMOBJECT_TYPE_USER:
        return "USER";
    case SIMCONNECT_SIMOBJECT_TYPE_ALL:
        return "ALL";
    case SIMCONNECT_SIMOBJECT_TYPE_AIRCRAFT:
        return "AIRCRAFT";
    case SIMCONNECT_SIMOBJECT_TYPE_HELICOPTER:
        return "HELICOPTER";
    case SIMCONNECT_SIMOBJECT_TYPE_BOAT:
        return "BOAT";
    case SIMCONNECT_SIMOBJECT_TYPE_GROUND:
        return "GROUND";
    default:
        assert(false);
    }
}

static std::string data_request_flags(int flags) {
    if (flags == SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT)
        return "DEFAULT";
    std::string result;
    if (flags & SIMCONNECT_DATA_REQUEST_FLAG_CHANGED)
        result += std::string(result != "" ? "+" : "") + "CHANGED";
    if (flags & SIMCONNECT_DATA_REQUEST_FLAG_TAGGED)
        result += std::string(result != "" ? "+" : "") + "TAGGED";
    return result;
}

static std::string panel_service(int type) {
    switch (type) {
    case PANEL_SERVICE_PRE_QUERY:
        return "PRE_QUERY";
    case PANEL_SERVICE_POST_QUERY:
        return "POST_QUERY";
    case PANEL_SERVICE_PRE_INSTALL:
        return "PRE_INSTALL";
    case PANEL_SERVICE_POST_INSTALL:
        return "POST_INSTALL";
    case PANEL_SERVICE_PRE_INITIALIZE:
        return "PRE_INITIALIZE";
    case PANEL_SERVICE_POST_INITIALIZE:
        return "POST_INITIALIZE";
    case PANEL_SERVICE_PRE_UPDATE:
        return "PRE_UPDATE";
    case PANEL_SERVICE_POST_UPDATE:
        return "POST_UPDATE";
    case PANEL_SERVICE_PRE_GENERATE:
        return "PRE_GENERATE";
    case PANEL_SERVICE_POST_GENERATE:
        return "POST_GENERATE";
    case PANEL_SERVICE_PRE_DRAW:
        return "PRE_DRAW";
    case PANEL_SERVICE_POST_DRAW:
        return "POST_DRAW";
    case PANEL_SERVICE_PRE_KILL:
        return "PRE_KILL";
    case PANEL_SERVICE_POST_KILL:
        return "POST_KILL";
    case PANEL_SERVICE_CONNECT_TO_WINDOW:
        return "CONNECT_TO_WINDOW";
    case PANEL_SERVICE_DISCONNECT:
        return "DISCONNECT";
    case PANEL_SERVICE_PANEL_OPEN:
        return "PANEL_OPEN";
    case PANEL_SERVICE_PANEL_CLOSE:
        return "PANEL_CLOSE";
    default:
        assert(false);
    }
}

#pragma GCC diagnostic pop

static void dumpReadonlyState(const ReadonlyState &state) {
    std::cout << " rud:" << std::fixed << std::setw(5) << std::setprecision(2) << state.rudder
              << " ail:" << std::fixed << std::setw(5) << std::setprecision(2) << state.aileron
              << " ele:" << std::fixed << std::setw(5) << std::setprecision(2) << state.elevator
              << " thr:" << std::fixed << std::setw(5) << std::setprecision(2) << state.throttle
              << " /"
              << " agl:" << std::fixed << std::setw(6) << std::setprecision(1) << state.agl
              << " gnd: " << (state.onGround ? "Y" : "N")
              << " ign: " << (state.ignitionSwitch ? "Y" : "N")
              << " brk:" << std::fixed << std::setw(5) << std::setprecision(2) << state.parkingBrake
              << " frz: " << (state.altFreeze ? "Y" : "N") << (state.attFreeze ? "Y" : "N") << (state.posFreeze ? "Y" : "N")
              << " pres:" << std::fixed << std::setw(8) << std::setprecision(6) << state.pressure;
}

static void dumpMutableState(const MutableState &state) {
    std::cout << " hdg:" << std::setw(3) << int(rad2deg(state.heading))
              << " bnk:" << std::setw(4) << int(rad2deg(state.bank))
              << " pit:" << std::setw(4) << int(rad2deg(state.pitch))
              << " pos: " << std::fixed << std::setprecision(4) << std::abs(rad2deg(state.lat)) << (state.lat > 0 ? "N" : "S")
              << " " << std::fixed << std::setprecision(4) << std::abs(rad2deg(state.lon)) << (state.lon > 0 ? "E" : "W")
              << " msl:" << std::fixed << std::setw(6) << std::setprecision(1) << state.msl
              << " /"
              << " bod:" << std::fixed << std::setw(5) << std::setprecision(2) << state.velBodyX
              << "," << std::fixed << std::setw(5) << std::setprecision(2) << state.velBodyY
              << "," << std::fixed << std::setw(5) << std::setprecision(2) << state.velBodyZ
              << " wld:" << std::fixed << std::setw(5) << std::setprecision(2) << state.velWorldX
              << "," << std::fixed << std::setw(5) << std::setprecision(2) << state.velWorldY
              << "," << std::fixed << std::setw(5) << std::setprecision(2) << state.velWorldZ
              << " /"
              << " ias:" << std::fixed << std::setw(3) << std::setprecision(0) << state.kias
              << " tas:" << std::fixed << std::setw(3) << std::setprecision(0) << state.ktas
              << " vs:" << std::fixed << std::setw(6) << std::setprecision(1) << state.vs;
}

class AllStateHistory {
private:
    static constexpr auto HistoryLength = 10;

    // How many array elements are in use. The ones at index current is always the latest.
    int number;
    int current;
    int previous;

    // How many times our callback has been called
    int callbacks_;

    // How many times we have stored new values into this struct
    int rounds_;

    // The history of state received from the sim
    std::array<AllState, HistoryLength> input;

    // Timestamps of the inputs
    std::array<std::chrono::time_point<std::chrono::steady_clock>, HistoryLength> time;

    // The history of state set into the sim
    std::array<MutableState, HistoryLength> output_;

public:
    AllStateHistory()
        : number(0),
          current(0),
          previous(0),
          callbacks_(0),
          rounds_(0)
    {
    }

    int bumpCallbacks() {
        return ++callbacks_;
    }

    void bump(const AllState &receivedInput) {
        rounds_++;

        if (number < HistoryLength)
            number++;

        previous = current;
        current = (current + 1) % HistoryLength;
        input[current] = receivedInput;
        output_[current] = output_[previous];
        time[current] = std::chrono::steady_clock::now();
    }

    int callbacks() const {
        return callbacks_;
    }

    int rounds() const {
        return rounds_;
    }

    const ReadonlyState& readonly() const {
        return input[current].readonly;
    }

    const MutableState& state() const {
        return input[current].state;
    }

    MutableState& output() {
        return output_[current];
    }

    int milliSecondsSinceLast() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(time[current] - time[previous]).count();
    }

    void setMotionless() {
        // Set the desired initial state: motionless

        std::cout << THISAIRCRAFT << ": setMotionless()" << std::flush;

        output() = input[current].state;

        // Keep heading as is

        output().bank = output().pitch = 0;

        // Keep lat, lon, msl as is

        output().velBodyX = output().velBodyY = output().velBodyZ = 0;
        output().velWorldX = output().velWorldY = output().velWorldZ = 0;

        output().kias = output().ktas = 0;

        // Keep alt as is

        output().vs = 0;

        if (verbose) {
            std::cout << THISAIRCRAFT ": " << std::setw(5) << callbacks() << " Set motionless state:";
            dumpMutableState(output());
            std::cout << std::flush;
        }
    }

    bool aboveGround() {
        // Use hysteresis to avoid toggling freeze back and forth. When we are controlling the aircraft
        // (simFrozen true), we decide that we are landing when the AGL below above static_cg_height + 1.
        // When we are not controlling the aircraft (letting it land "softly" by itself, we require AGL to be one
        // foot higher to decide we are not landing any more.

        assert(!(landing && takingOff));

        if (simFrozen && !landing && !takingOff)
            return readonly().agl > static_cg_height + 1;
        else if (takingOff)
            return true;
        else
            return (!readonly().onGround && readonly().agl > static_cg_height + 2);
    }
};

static std::map<DWORD, std::string> calls;

static HRESULT recordCall(int lineNumber,
                          std::string call,
                          HRESULT value) {
    if (!SUCCEEDED(value)) {
        // Output to std::cerr is unbuffered, and appears in the Console window each part on a separate line.
        // Not ideal. So collect output to std::cerr into one string and write it in one go.
        std::stringstream output;
        output << THISAIRCRAFT ": The call '" << call << "' failed at line " << lineNumber;
        std::cerr << output.str() << std::flush;
        failed = true;
        return value;
    }

    DWORD id;
    SimConnect_GetLastSentPacketID(hSimConnect, &id);
    calls[id] = call;

    static uint64_t counter = 0;
    if ((++counter % 100) == 0 && id > 100)
    {
        auto newestToRemove = calls.upper_bound(id - 100);
        if (newestToRemove != calls.end())
            calls.erase(calls.begin(), newestToRemove);
    }

    return value;
}

#define RECORD(expr) \
    recordCall(__LINE__, #expr, expr);

constexpr auto HUNDREDTH = 0.01;

constexpr auto EARTH_RADIUS_FT = m2ft(6371000);

// Full throttle means 1000 fpm up, zero throttle means 1000 fpm down. Keep a large dead zone around 50%
// throttle.
static double throttle2vs(double throttle) {
    if (throttle < 0.45)
        return fpm2fps((throttle - 0.45) / 0.45 * 1000);
    else if (throttle > 0.55)
        return fpm2fps((throttle - 0.55) / 0.45 * 1000);
    else
        return 0;
}

static bool doDisplay(const AllStateHistory &state) {
    // For now, display when we are close to ground and for five sequential callbacks every 500 callbacks.
    return state.readonly().agl < 10 || (state.rounds() % 500) < 5;
}

static void setDirectControl(AllStateHistory &state) {
    if (verbose && doDisplay(state)) {
        std::cout << THISAIRCRAFT ": " << std::setw(5) << state.callbacks() << " Set state:";
        dumpMutableState(state.output());
        std::cout << std::flush;
    }
    assert(sizeof(MutableState) == sizeof(state.output()));
    RECORD(SimConnect_SetDataOnSimObject(hSimConnect, DataDefinitionMutableState,
                                         SIMCONNECT_OBJECT_ID_USER, 0,
                                         0, sizeof(MutableState), (void*)&state.output()));
}

static void setVerticalSpeed(double throttle, double timeSinceLast, MutableState &control) {
    const double vs = throttle2vs(throttle);
    if (vs != 0) {
        control.velBodyY = vs;
        control.velWorldY = control.velBodyY;

        const double diffY = control.velWorldY * timeSinceLast / 1000;
        control.msl += diffY;
        control.vs = fps2fpm(vs);
    } else {
        control.velBodyY = control.velWorldY = control.vs = 0;
    }
}

static void freezeSimulation(const ReadonlyState &state) {
    if (!state.altFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Freezing ALT" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAltSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
    if (!state.attFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Freezing ATT" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAttSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
    if (!state.posFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Freezing POS" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezePosSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
    simFrozen = true;
}

static void unfreezeSimulation(const ReadonlyState &state) {
    if (state.altFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Unfreezing ALT" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAltSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
    if (state.attFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Unfreezing ATT" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAttSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
#if 0 // We never want to let the simulator move the aircraft as it seems to let the wind affect it wildly
    if (state.posFreeze) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Unfreezing POS" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezePosSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }
#endif
    simFrozen = false;
}

static void handleState(const AllState &input) {
            
    // If paused, do nothing
    if (simPaused)
        return;

    // Check if we are in a "zombie" state when the sim is in the main menu, at "Null Island" (0N 0E).
    // In that case, do nothing.1
    if (std::abs(input.state.lat) < 0.0001 && std::abs(input.state.lon) < 0.0001)
        return;

    // Another "zombie" state: Deep down under ground or up in the stratosphere.
    if (input.readonly.agl < -100 || input.readonly.agl > 100000
        || input.state.msl < -100 || input.state.msl > 100000)
        return;

    // If ignition switch off, do nothing. When turning it off, let the simulator handle the aircraft
    // falling down, typically. When turning it on, take control.

    if (ignitionSwitch && !input.readonly.ignitionSwitch) {
        ignitionSwitch = false;
        unfreezeSimulation(input.readonly);
        gotFirstState = false;
        return;
    }

    static AllStateHistory state;

    // Ignore first couple of state callbacks
    if (state.bumpCallbacks() < 5)
        return;

    state.bump(input);

    MutableState& control = state.output();

    if (!ignitionSwitch && state.readonly().ignitionSwitch) {
        ignitionSwitch = true;
        freezeSimulation(state.readonly());
        std::cout << THISAIRCRAFT << ": line " << __LINE__ << std::flush;
        state.setMotionless();
    } else if (!state.readonly().ignitionSwitch) {
        return;
    }

    if (verbose && doDisplay(state)) {
        std::cout << THISAIRCRAFT ": " << std::setw(5) << state.callbacks() << " Got RO state:";
        dumpReadonlyState(state.readonly());
        std::cout << std::flush;
        std::cout << THISAIRCRAFT ": " << std::setw(5) << state.callbacks() << " Got state:";
        dumpMutableState(state.state());
        std::cout << std::flush;
    }

    // We want the parking brake to be always on when on ground
    if (state.readonly().onGround && state.readonly().parkingBrake < 0.5) {
        if (verbose)
            std::cout << THISAIRCRAFT ": Setting parking brake" << std::flush;
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventParkingBrakeToggle, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    }

    auto timeSinceLast = state.milliSecondsSinceLast();

    // Obviously we can turn and move only in the air
    if (state.aboveGround()) {

        if (state.readonly().agl > static_cg_height + 2)
            takingOff = false;

        // Make sure the simulator itself is not trying to move the aircraft.
        freezeSimulation(state.readonly());

        if (!gotFirstState) {
            std::cout << THISAIRCRAFT << ": line " << __LINE__ << std::flush;
            state.setMotionless();
            ignitionSwitch = state.readonly().ignitionSwitch;
            gotFirstState = true;
        } else {
            // Arbitrary choice: Full rudder pedal deflection means 45 degrees per second yaw rate.
            auto diff = state.readonly().rudder * timeSinceLast / 1000 * deg2rad(45);
            control.heading += diff;
            while (control.heading >= 2 * M_PI)
                control.heading -= 2 * M_PI;
            while (control.heading < 0)
                control.heading += 2 * M_PI;

            // Another arbitrary choice: Full elevator control deflection means 100 knots GS
            // forward or 50 knots backward. Full aileron control deflection means 50 knots left or right.

            // Stick pushed forward: negative elevator input, set speed forward.
            // Stick pulled back: positive elevator input, set speed backward.
            if (state.readonly().elevator < -HUNDREDTH)
                control.velBodyZ = -state.readonly().elevator * kn2fps(100);
            else if (state.readonly().elevator > HUNDREDTH)
                control.velBodyZ = -state.readonly().elevator * kn2fps(50);
            else
                control.velBodyZ = 0;

            // Stick tilted sideways: aileron input, set speed sideways
            if (std::abs(state.readonly().aileron) > HUNDREDTH)
                control.velBodyX = state.readonly().aileron * kn2fps(50);
            else
                control.velBodyX = 0;

            const double bodyRelativeAbsoluteVelocity = std::sqrt(control.velBodyZ * control.velBodyZ
                                                                  + control.velBodyX * control.velBodyX);

            const double bodyRelativeTrack = M_PI/2 - std::atan2(control.velBodyZ, control.velBodyX);

            const double worldRelativeTrack = control.heading + bodyRelativeTrack;

            control.velWorldZ = std::cos(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;
            control.velWorldX = std::sin(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;

            // This is just a toy, so use a spherical Earth approximation and ignore the poles
            // and the antimeridian.
            control.lat += control.velWorldZ * timeSinceLast / 1000 / EARTH_RADIUS_FT;
            control.lon += control.velWorldX * timeSinceLast / 1000 * std::cos(control.lat) / EARTH_RADIUS_FT;

            // Assume this aircraft is used only at low altitudes and ignore wind
            control.kias = fps2kn(bodyRelativeAbsoluteVelocity);
            control.ktas = control.kias;

            // Vertical speed and position handled in setVerticalSpeed().
            setVerticalSpeed(state.readonly().throttle, timeSinceLast, control);
        }
        setDirectControl(state);
    } else {

        assert(!state.aboveGround());

        // Vertical velocity however can be changed while on the ground. We can lift off.

        if (throttle2vs(state.readonly().throttle) > 0) {
            landing = false;
            takingOff = true;
            setVerticalSpeed(state.readonly().throttle, timeSinceLast, control);
            freezeSimulation(state.readonly());
            setDirectControl(state);
        } else if (gotFirstState) {
            landing = true;
            std::cout << THISAIRCRAFT << ": line " << __LINE__ << std::flush;
            state.setMotionless();
            setDirectControl(state);

            // Let the simulator itself handle it on ground
            unfreezeSimulation(state.readonly());

            gotFirstState = false;
        }
    }
}

static void dispatchProc(SIMCONNECT_RECV *pData, DWORD cbData, void *pContext) {
    if (failed)
        return;

    switch(pData->dwID) {
    case SIMCONNECT_RECV_ID_EVENT: {
        SIMCONNECT_RECV_EVENT *event = (SIMCONNECT_RECV_EVENT*)pData;

        switch(event->uEventID) {
        case EventPause:
            simPaused = event->dwData;
            if (verbose)
                std::cout << THISAIRCRAFT ": PAUSE " << (simPaused ? "ON" : "OFF") << std::flush;
            break;
        default:
            if (verbose)
                std::cout << THISAIRCRAFT ": EVENT " << event->uEventID << " " << event->dwData << std::flush;
            break;
        }
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_FILENAME: {
        SIMCONNECT_RECV_EVENT_FILENAME *filename = (SIMCONNECT_RECV_EVENT_FILENAME*)pData;
        if (verbose)
            std::cout << THISAIRCRAFT ": EVENT_FILENAME "
                      << filename->szFileName;
        switch (filename->uEventID) {
        default:
            if (verbose)
                std::cout << " " << filename->uEventID;
            break;
        }
        if (verbose)
            std::cout << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        SIMCONNECT_RECV_SIMOBJECT_DATA *data = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
        switch (data->dwRequestID) {
        case RequestAllState:
            handleState(*(AllState*)&data->dwData);
            break;
        default:
            assert(false);
        }
        break;
    }
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        SIMCONNECT_RECV_EXCEPTION *exception = (SIMCONNECT_RECV_EXCEPTION*)pData;
        std::stringstream output;
        output << THISAIRCRAFT ": EXCEPTION "
               << exception_type(exception->dwException) << " "
               << (calls.count(exception->dwSendID) ? ("from " + calls[exception->dwSendID]) : "from unknown API call") << " "
               << exception->dwIndex;
        std::cerr << output.str() << std::flush;
        failed = true;
        break;
    }
    }
}

static bool flight_model_callback(const char *Section, const char *Key, const char *Value, void *UserData) {
    if (strcasecmp(Section, "CONTACT_POINTS") == 0
        && strcasecmp(Key, "static_cg_height") == 0) {

        static_cg_height = strtod(Value, NULL);
        if (verbose)
            std::cout << THISAIRCRAFT ": Static CG height from flight_model.cfg: " << static_cg_height << "ft" << std::flush;

        // That is all we want.
        return false;
    }

    // Continue browsing.
    return true;
}

static void initialize() {
    if (hSimConnect != 0)
        return;

    if (!SUCCEEDED(SimConnect_Open(&hSimConnect, THISAIRCRAFT, nullptr, 0, 0, 0))) {
        std::cerr << THISAIRCRAFT ": SimConnect_Open failed" << std::flush;
        return;
    }

    // Read our flight_model.cfg to avoid having to duplicate some information as magic numbers in this file.
    ini_browse(flight_model_callback, NULL, ".\\SimObjects\\Airplanes\\" THISAIRCRAFT "\\flight_model.cfg");

    // Let's re-set this to false after each SimConnect_Open()
    failed = false;

    // Most likely it is pointless to check the return values from these SimConnect calls. It seems that
    // errors in parameters are reported asynchronously anyway as SIMCONNECT_RECV_ID_EXCEPTION.

    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventPause, "Pause"));

    RECORD(SimConnect_MapClientEventToSimEvent(hSimConnect, EventFreezeAltSet, "FREEZE_ALTITUDE_SET"));
    RECORD(SimConnect_MapClientEventToSimEvent(hSimConnect, EventFreezeAttSet, "FREEZE_ATTITUDE_SET"));
    RECORD(SimConnect_MapClientEventToSimEvent(hSimConnect, EventFreezePosSet, "FREEZE_LATITUDE_LONGITUDE_SET"));
    RECORD(SimConnect_MapClientEventToSimEvent(hSimConnect, EventParkingBrakeToggle, "PARKING_BRAKES"));

    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "RUDDER PEDAL POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "AILERON POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "ELEVATOR POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "GENERAL ENG THROTTLE LEVER POSITION:1", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "PLANE ALT ABOVE GROUND", "feet",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));

    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "RELATIVE WIND VELOCITY BODY X", "feet/second",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "RELATIVE WIND VELOCITY BODY Y", "feet/second",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "RELATIVE WIND VELOCITY BODY Z", "feet/second",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));

    // Use only 64-bit types so that the sizes of the structs (without any packing pragmas) match what
    // SimConnect wants.
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "SIM ON GROUND", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "MASTER IGNITION SWITCH", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "BRAKE PARKING POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          0));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "IS ALTITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "IS ATTITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "IS LATITUDE LONGITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "AMBIENT PRESSURE", "inHg",
                                          SIMCONNECT_DATATYPE_FLOAT64));


    for (auto definition: {DataDefinitionAllState, DataDefinitionMutableState}) {
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE HEADING DEGREES TRUE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE BANK DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE PITCH DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LATITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LONGITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              0));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED INDICATED", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED TRUE", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VERTICAL SPEED", "feet/minute",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
    }

    gotFirstState = false;
    ignitionSwitch = false;

    RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                             RequestAllState, DataDefinitionAllState,
                                             SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                             SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                             0));

    RECORD(SimConnect_CallDispatch(hSimConnect, dispatchProc, NULL));
}

static void deinitialize() {
    if (hSimConnect == 0)
        return;

    // Effectively unsubscribe to this data by (re-)requesting it with a very high interval
    RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                             RequestAllState, DataDefinitionAllState,
                                             SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                             SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                             DWORD_MAX));

    if (!SUCCEEDED(SimConnect_Close(hSimConnect))) {
        std::cerr << THISAIRCRAFT ": SimConnect_Close failed" << std::flush;
        return;
    }

    hSimConnect = 0;
}

extern "C" MSFS_CALLBACK bool FlightModel_gauge_callback(FsContext ctx, int service_id, void* pData) {
    switch (service_id) {
    case PANEL_SERVICE_PRE_INSTALL:
        initialize();
        break;

    case PANEL_SERVICE_PRE_KILL:
        deinitialize();
        break;
    }

    return true;
}
