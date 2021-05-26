// -*- comment-column: 50; fill-column: 110; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*-

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
    int64_t altFreeze, attFreeze, posFreeze;
};

struct MutableState {
    double heading;
    double bank, pitch;
    double lat, lon, msl;

    double velBodyX, velBodyY, velBodyZ;
    double velWorldX, velWorldY, velWorldZ;

    // Instrument indications (only?)
    double kias, ktas;
    double alt;
    double vs;
};

struct AllState {
    ReadonlyState readonly;
    MutableState state;
};

// As soon as an API call fails or we get an exception we are in an unknown state and it is not worth
// to continue attempting to do anything.
static bool failed = false;

static bool simPaused = true;
static bool gotFirstState = false;
static bool ignitionSwitch = false;

static int stateRequestDispatchCounter;

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

static void dumpReadonlyState(const ReadonlyState &state) {
    std::cout << " rud:" << std::fixed << std::setw(5) << std::setprecision(2) << state.rudder
              << " ail:" << std::fixed << std::setw(5) << std::setprecision(2) << state.aileron
              << " ele:" << std::fixed << std::setw(5) << std::setprecision(2) << state.elevator
              << " thr:" << std::fixed << std::setw(5) << std::setprecision(2) << state.throttle
              << " /"
              << " agl:" << std::fixed << std::setw(6) << std::setprecision(1) << state.agl
              << " gnd: " << (state.onGround ? "Y" : "N")
              << " ign: " << (state.ignitionSwitch ? "Y" : "N")
              << " frz: " << (state.altFreeze ? "Y" : "N") << (state.attFreeze ? "Y" : "N") << (state.posFreeze ? "Y" : "N");
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
              << " alt:"  << std::fixed << std::setw(6) << std::setprecision(1) << state.alt
              << " vs:" << std::fixed << std::setw(6) << std::setprecision(1) << state.vs;
}

#pragma GCC diagnostic pop


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

static void setMotionlessState(const MutableState &state, MutableState &control) {
    // Set the desired initial state: motionless

    control = state;

    // Keep heading as is

    control.bank = control.pitch = 0;

    // Keep lat, lon, msl as is

    control.velBodyX = control.velBodyY = control.velBodyZ = 0;
    control.velWorldX = control.velWorldY = control.velWorldZ = 0;

    control.kias = control.ktas = 0;

    // Keep alt as is

    control.vs = 0;
}

static bool doDisplay(const ReadonlyState &state) {
    return state.agl < 10 || (stateRequestDispatchCounter % 50) == 0;
}

static void setDirectControl(const ReadonlyState &ronly, const MutableState &control) {
    if (doDisplay(ronly)) {
        std::cout << THISAIRCRAFT ": " << std::setw(5) << stateRequestDispatchCounter << " Set state:";
        dumpMutableState(control);
        std::cout << std::flush;
    }
    RECORD(SimConnect_SetDataOnSimObject(hSimConnect, DataDefinitionMutableState,
                                         SIMCONNECT_OBJECT_ID_USER, 0,
                                         0, sizeof(MutableState), (void*)&control));
}

static bool setVerticalSpeed(double throttle, double timeSinceLast, MutableState &control) {
    const double vs = throttle2vs(throttle);
    if (vs != 0) {
        control.velBodyY = vs;
        control.velWorldY = control.velBodyY;

        const double diffY = control.velWorldY * timeSinceLast / 1000;
        control.alt += diffY;
        control.msl += diffY;
        control.vs = fps2fpm(vs);

        return true;
    }
    return false;
}

static bool aboveGround(const ReadonlyState &state) {
    return (!state.onGround && state.agl > static_cg_height + 1);
}

static void freezeSimulation(const ReadonlyState &state) {
    if (!state.altFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAltSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    if (!state.attFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAttSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    if (!state.posFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezePosSet, TRUE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
}

static void unfreezeSimulation(const ReadonlyState &state) {
    if (state.altFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAltSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    if (state.attFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezeAttSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
    if (state.posFreeze)
        RECORD(SimConnect_TransmitClientEvent(hSimConnect, SIMCONNECT_OBJECT_ID_USER, EventFreezePosSet, FALSE,
                                              SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE, SIMCONNECT_EVENT_FLAG_GROUPID_IS_PRIORITY));
}

static void handleState(const AllState &state) {
            
    // If paused, do nothing
    if (simPaused)
        return;

    static MutableState directControl;

    // Check if we are in a "zombie" state when the sim is in the main menu, at "Null Island" (0N 0E).
    // In that case, do nothing.
    if (std::abs(state.state.lat) < 0.0001 && std::abs(state.state.lon) < 0.0001)
        return;

    // Another "zombie" state: Deep down under ground or up in the stratosphere.
    if (state.readonly.agl < -100 || state.readonly.agl > 100000 || state.state.msl < -100 || state.state.msl > 100000)
        return;

    // If ignition switch off, do nothing. When turning it off, let the simulator handle the aircraft
    // falling down, typically. When turning it on, take control.

    if (ignitionSwitch && !state.readonly.ignitionSwitch) {
        ignitionSwitch = false;
        unfreezeSimulation(state.readonly);
        gotFirstState = false;
        return;
    }

    if (!ignitionSwitch && state.readonly.ignitionSwitch) {
        ignitionSwitch = true;
        freezeSimulation(state.readonly);
        setMotionlessState(state.state, directControl);
    } else if (!state.readonly.ignitionSwitch) {
        return;
    }

    ++stateRequestDispatchCounter;

    if (doDisplay(state.readonly)) {
        std::cout << THISAIRCRAFT ": " << std::setw(5) << stateRequestDispatchCounter << " Got RO state:";
        dumpReadonlyState(state.readonly);
        std::cout << std::flush;
        std::cout << THISAIRCRAFT ": " << std::setw(5) << stateRequestDispatchCounter << " Got state:";
        dumpMutableState(state.state);
        std::cout << std::flush;
    }

    std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
    static std::chrono::time_point<std::chrono::steady_clock> lastTime = now;
    auto timeSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();

    // Obviously we can turn and move only in the air
    if (aboveGround(state.readonly)) {

        // Make sure the simulator itself is not trying to move the aircraft.
        freezeSimulation(state.readonly);

        if (!gotFirstState) {
            setMotionlessState(state.state, directControl);
            ignitionSwitch = state.readonly.ignitionSwitch;
            gotFirstState = true;
        } else {
            // Arbitrary choice: Full rudder pedal deflection means 45 degrees per second yaw rate.
            auto diff = state.readonly.rudder * timeSinceLast / 1000 * deg2rad(45);
            directControl.heading += diff;
            while (directControl.heading >= 2 * M_PI)
                directControl.heading -= 2 * M_PI;
            while (directControl.heading < 0)
                directControl.heading += 2 * M_PI;

            // Another arbitrary choice: Full elevator control deflection means 100 knots GS
            // forward or 50 knots backward. Full aileron control deflection means 50 knots left or right.

            // Stick pushed forward: negative elevator input, set speed forward.
            // Stick pulled back: positive elevator input, set speed backward.
            if (state.readonly.elevator < -HUNDREDTH)
                directControl.velBodyZ = -state.readonly.elevator * kn2fps(100);
            else if (state.readonly.elevator > HUNDREDTH)
                directControl.velBodyZ = -state.readonly.elevator * kn2fps(50);
            else
                directControl.velBodyZ = 0;

            // Stick tilted sideways: aileron input, set speed sideways
            if (std::abs(state.readonly.aileron) > HUNDREDTH)
                directControl.velBodyX = state.readonly.aileron * kn2fps(50);
            else
                directControl.velBodyX = 0;

            const double bodyRelativeAbsoluteVelocity = std::sqrt(directControl.velBodyZ * directControl.velBodyZ
                                                                  + directControl.velBodyX * directControl.velBodyX);
            const double bodyRelativeTrack = M_PI/2 - std::atan2(directControl.velBodyZ, directControl.velBodyX);
            const double worldRelativeTrack = directControl.heading + bodyRelativeTrack;

            directControl.velWorldZ = std::cos(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;
            directControl.velWorldX = std::sin(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;

            // This is just a toy, so use a spherical Earth approximation and ignore the poles
            // and the antimeridian.
            directControl.lat += directControl.velWorldZ * timeSinceLast / 1000 / EARTH_RADIUS_FT;
            directControl.lon += directControl.velWorldX * timeSinceLast / 1000 * std::cos(directControl.lat) / EARTH_RADIUS_FT;

            // Assume this aircraft is used only at low altitudes and ignore wind
            directControl.kias = fps2kn(bodyRelativeAbsoluteVelocity);
            directControl.ktas = directControl.kias;

            // Vertical speed and position handled in setVerticalSpeed().
            setVerticalSpeed(state.readonly.throttle, timeSinceLast, directControl);
        }
        setDirectControl(state.readonly, directControl);
        lastTime = now;
    } else {

        assert(!aboveGround(state.readonly));

        // Vertical velocity however can be changed while on the ground. We can lift off.

        if (throttle2vs(state.readonly.throttle) > 0) {
            setVerticalSpeed(state.readonly.throttle, timeSinceLast, directControl);
            freezeSimulation(state.readonly);
            setDirectControl(state.readonly, directControl);
            lastTime = now;
        } else if (gotFirstState) {
            setMotionlessState(state.state, directControl);
            setDirectControl(state.readonly, directControl);

            // Let the simulator itself handle it on ground
            unfreezeSimulation(state.readonly);

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
            std::cout << THISAIRCRAFT ": PAUSE " << (simPaused ? "ON" : "OFF") << std::flush;
            break;
        default:
            std::cout << THISAIRCRAFT ": EVENT " << event->uEventID << " " << event->dwData << std::flush;
            break;
        }
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_FILENAME: {
        SIMCONNECT_RECV_EVENT_FILENAME *filename = (SIMCONNECT_RECV_EVENT_FILENAME*)pData;
        std::cout << THISAIRCRAFT ": EVENT_FILENAME "
                  << filename->szFileName;
        switch (filename->uEventID) {
        default:
            std::cout << " " << filename->uEventID;
            break;
        }
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
    std::cout << THISAIRCRAFT ": Connected" << std::flush;

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
                                          "IS ALTITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "IS ATTITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "IS LATITUDE LONGITUDE FREEZE ON", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));


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
                                              "INDICATED ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VERTICAL SPEED", "feet/minute",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              10));
    }

    stateRequestDispatchCounter = 0;
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
