// -*- comment-column: 50; fill-column: 110; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*-

#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "MSFS/MSFS.h"
#include "MSFS/MSFS_Render.h"
#include "MSFS/MSFS_WindowsTypes.h"
#include "SimConnect.h"

#include "FlyingBrick.h"

static HANDLE hSimConnect = 0;

// Use different numeric ranges for the enums to recognize the values if they show up in unexpected places

enum DataDefinition : SIMCONNECT_DATA_DEFINITION_ID {
    DataDefinitionAircraftState = 1000,
    DataDefinitionAllState,
};

enum Event : DWORD {
    EventPause = 2000,
};

enum Request : DWORD {
    RequestAllState = 3000,
};

struct ReadonlyState {
    double rudder;
    double aileron;
    double elevator;
    double throttle, mixture;
    int64_t onGround;
};

struct AircraftState {
    // Actual values
    double velBodyX, velBodyY, velBodyZ;
    double velWorldX, velWorldY, velWorldZ;
    double heading;
    double bank, pitch;
    double lat, lon, msl;
    // Indications
    double kias, ktas;
    double alt;
    double vs;
};

struct AllState {
    ReadonlyState readonly;
    AircraftState state;
};

// As soon as an API call fails or we get an exception we are in an unknown state and it is not worth
// to continue attempting to do anything.
static bool failed = false;

static bool simPaused = true;
static bool gotFirstState = false;

static AircraftState desiredState;

static std::map<DWORD, std::string> calls;

static HRESULT record_call(int lineNumber,
                           std::string call,
                           HRESULT value) {
    if (!SUCCEEDED(value)) {
        std::cerr << "==== FlyingBrick: The call '" << call << "' failed at line " << lineNumber << std::endl;
        failed = true;
        return value;
    }

    DWORD id;
    SimConnect_GetLastSentPacketID(hSimConnect, &id);
    calls[id] = call;

    static uint64_t counter = 0;
    if ((++counter % 100) == 0 && id > 100)
    {
        auto oldSize = calls.size();
        auto newestToRemove = calls.upper_bound(id - 100);
        if (newestToRemove != calls.end())
            calls.erase(calls.begin(), newestToRemove);
    }

    return value;
}

#define RECORD(expr) \
    record_call(__LINE__, #expr, expr);

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

constexpr auto HALF = 0.5;
constexpr auto HUNDREDTH = 0.01;

constexpr auto EARTH_RADIUS_FT = m2ft(6371000);

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
        return std::to_string(exception);
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
        return std::to_string(type);
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
        return "? (" + std::to_string(type) + ")";
    }
}

static void FlyingBrickDispatchProc(SIMCONNECT_RECV *pData, DWORD cbData, void *pContext) {
    if (failed)
        return;

    HRESULT hr;

    switch(pData->dwID) {
    case SIMCONNECT_RECV_ID_EVENT: {
        SIMCONNECT_RECV_EVENT *event = (SIMCONNECT_RECV_EVENT*)pData;

        switch(event->uEventID) {
        case EventPause:
            simPaused = event->dwData;
            if (simPaused)
                gotFirstState = false;
            break;
        default:
            std::cout << "==== FlyingBrick: EVENT " << event->uEventID << " " << event->dwData << std::endl;
            break;
        }
        break;
    }
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        SIMCONNECT_RECV_SIMOBJECT_DATA *data = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
        switch (data->dwRequestID) {
        case RequestAllState: {
            AllState *state = (AllState*)&data->dwData;
            
            static uint64_t counter = 0;
            if ((++counter % 10) == 0)
                std::cout << "==== FlyingBrick: SIMOBJECT_DATA RequestAllState "
                          << " rudder:" << std::fixed << std::setw(5) << std::setprecision(2) << state->readonly.rudder
                          << " aileron:" << std::fixed << std::setw(5) << std::setprecision(2) << state->readonly.aileron
                          << " elevator:" << std::fixed << std::setw(5) << std::setprecision(2) << state->readonly.elevator
                          << " throttle:" << std::fixed << std::setw(5) << std::setprecision(2) << state->readonly.throttle
                          << " mixture:" << std::fixed << std::setw(5) << std::setprecision(2) << state->readonly.mixture
                          << " velBody:(" << std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyX) << ","
                          <<                 std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyY) << ","
                          <<                 std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyZ) << ")"
                          << " velWorld:(" << std::setw(4) << std::setprecision(2) << int(state->state.velWorldX) << ","
                          <<               std::setw(4) << std::setprecision(2) << int(state->state.velWorldY) << ","
                          <<               std::setw(4) << std::setprecision(2) << int(state->state.velWorldZ) << ")"
                          << " heading:" << std::setw(3) << int(rad2deg(state->state.heading))
                          << " bank:" << std::setw(4) << int(rad2deg(state->state.bank))
                          << " pitch:" << std::setw(4) << int(rad2deg(state->state.pitch))
                          << " " << std::fixed << std::setprecision(4) << std::abs(rad2deg(state->state.lat)) << (state->state.lat > 0 ? "N" : "S")
                          << " " << std::fixed << std::setprecision(4) << std::abs(rad2deg(state->state.lon)) << (state->state.lon > 0 ? "E" : "W")
                          << std::endl;

            std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            static std::chrono::time_point<std::chrono::steady_clock> lastTime = now;
            
            if (!simPaused) {
                if (!gotFirstState) {
                    // Set the desired initial state: motionless
                    desiredState = state->state;
                    desiredState.velBodyX = desiredState.velBodyY = desiredState.velBodyZ = 0;
                    desiredState.velWorldX = desiredState.velWorldY = desiredState.velWorldZ = 0;
                    desiredState.heading = desiredState.bank = desiredState.pitch = 0;
                    desiredState.kias = desiredState.ktas = 0;
                    desiredState.vs = 0;

                    gotFirstState = true;
                } else {
                    auto timeSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();

                    // Obviously we can turn and move only in the air
                    if (!state->readonly.onGround) {
                        // Arbitrary choice in this toy: Full rudder pedal deflection means 45 degrees per
                        // second yaw rate.
                        if (std::fabs(state->readonly.rudder) > HUNDREDTH) {
                            auto diff = state->readonly.rudder * timeSinceLast / 1000 * deg2rad(45);
                            desiredState.heading += diff;
                        }

                        // Another arbitrary choice: Full elevator control deflection means 50 knots GS
                        // forward or 10 knots backward. Full aileron control deflection means 10 knots left or right.
                        if (std::abs(state->readonly.elevator) > HUNDREDTH || std::abs(state->readonly.aileron) > HUNDREDTH) {

                            // Stick pushed forward: negative elevator input, set speed forward.
                            // Stick pulled back: positive elevator input, set speed backward.
                            if (state->readonly.elevator < -HUNDREDTH)
                                desiredState.velBodyZ = -state->readonly.elevator * kn2fps(50);
                            else if (state->readonly.elevator > HUNDREDTH)
                                desiredState.velBodyZ = -state->readonly.elevator * kn2fps(10);
                            else
                                desiredState.velBodyZ = 0;

                            // Stick tilted sideways: aileron input, set speed sideways
                            if (std::abs(state->readonly.elevator) > HUNDREDTH)
                                desiredState.velBodyX = state->readonly.aileron * kn2fps(10);
                            else
                                desiredState.velBodyX = 0;

                            const double bodyRelativeAbsoluteVelocity = std::sqrt(desiredState.velBodyZ * desiredState.velBodyZ
                                                                                  + desiredState.velBodyX * desiredState.velBodyX);
                            const double bodyRelativeTrack = M_PI/2 - std::atan2(desiredState.velBodyZ, desiredState.velBodyX);
                            const double worldRelativeTrack = desiredState.heading + bodyRelativeTrack;

                            desiredState.velWorldZ = std::cos(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;
                            desiredState.velWorldX = std::sin(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;

                            // Also modify the geographical position. This is just a toy, use spherical Earth approximation and ignore the poles and
                            // the 180E longitude.
                            desiredState.lat += desiredState.velWorldZ * timeSinceLast / 1000 / EARTH_RADIUS_FT;
                            desiredState.lon += desiredState.velWorldX * timeSinceLast / 1000 * std::cos(desiredState.lat) / EARTH_RADIUS_FT;

                            desiredState.kias = fps2kn(bodyRelativeAbsoluteVelocity);
                            // Assume this aircraft is used only at low altitudes and ignore wind
                            desiredState.ktas = desiredState.kias;
                        }
                    }

                    // Vertical velocity however can be changed while on the ground. We can lift off. Full
                    // throttle (and zero mixture) means 500 fpm up, full mixture (and zero throttle) means 500 fpm down
                    if ((!state->readonly.onGround && std::abs(state->readonly.throttle - state->readonly.mixture) > HUNDREDTH)
                        || (state->readonly.onGround && state->readonly.throttle - state->readonly.mixture > HUNDREDTH)) {
                        desiredState.velBodyY = (state->readonly.throttle - state->readonly.mixture) * fpm2fps(500);
                        desiredState.velWorldY = desiredState.velBodyY;
                        desiredState.vs = fps2fpm(desiredState.velBodyY);

                        // And modify the altitude
                        desiredState.alt += desiredState.velWorldY * timeSinceLast / 1000;
                    }
                    lastTime = now;
                }
                RECORD(SimConnect_SetDataOnSimObject(hSimConnect, DataDefinitionAircraftState,
                                                     SIMCONNECT_OBJECT_ID_USER, 0,
                                                     0, sizeof(AircraftState), &desiredState));
            }
            break;
        }
        default:
            std::cerr << " ? (" << data->dwRequestID << ")";
        }
        break;
    }
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        SIMCONNECT_RECV_EXCEPTION *exception = (SIMCONNECT_RECV_EXCEPTION*)pData;
        std::cerr << "==== FlyingBrick: EXCEPTION "
                  << exception_type(exception->dwException) << " "
                  << (calls.count(exception->dwSendID) ? ("from " + calls[exception->dwSendID]) : "from unknown API call") << " "
                  << exception->dwIndex
                  << std::endl;
        failed = true;
        break;
    }
    }
}

static void init() {
    HRESULT hr;

    if (hSimConnect != 0)
        return;

    if (!SUCCEEDED(SimConnect_Open(&hSimConnect, "FlyingBrick", nullptr, 0, 0, 0))) {
        std::cerr << "==== FlyingBrick: SimConnect_Open failed" << std::endl;
        return;
    }
    std::cout << "==== FlyingBrick: Connected" << std::endl;

    // Let's re-set this to false after each SimConnect_Open()
    failed = false;

    // Most likely it is pointless to check the return values from these SimConnect calls. It seems that
    // errors in parameters are reported asynchronously anyway as SIMCONNECT_RECV_ID_EXCEPTION.

    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventPause, "Pause"));

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
                                          "GENERAL ENG MIXTURE LEVER POSITION:1", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64,
                                          HUNDREDTH));

    // Use only 64-bit types so that the sizes of the structs (without any packing pragmas) match what
    // SimConnect wants.
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAllState,
                                          "SIM ON GROUND", "boolean",
                                          SIMCONNECT_DATATYPE_INT64));


    for (auto definition: {DataDefinitionAllState, DataDefinitionAircraftState}) {
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE HEADING DEGREES TRUE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE BANK DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE PITCH DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LATITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LONGITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HUNDREDTH));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED INDICATED", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED TRUE", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "INDICATED ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VERTICAL SPEED", "feet/minute",
                                              SIMCONNECT_DATATYPE_FLOAT64,
                                              HALF));
    }

    RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                             RequestAllState, DataDefinitionAllState,
                                             SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                             SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                             0));

    gotFirstState = false;

    RECORD(SimConnect_CallDispatch(hSimConnect, FlyingBrickDispatchProc, NULL));
}

static void deinit() {
    if (hSimConnect == 0)
        return;

    // Effectively unsubscribe to this data by (re-)requesting it with a very high interval
    RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                             RequestAllState, DataDefinitionAllState,
                                             SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                             SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                             DWORD_MAX));

    if (!SUCCEEDED(SimConnect_Close(hSimConnect))) {
        std::cerr << "==== FlyingBrick: SimConnect_Close failed" << std::endl;
        return;
    }

    hSimConnect = 0;
}

extern "C" MSFS_CALLBACK bool FlightModel_gauge_callback(FsContext ctx, int service_id, void* pData) {
    switch (service_id) {
    case PANEL_SERVICE_PRE_INSTALL:
        std::cout << "==== FlyingBrick: " << panel_service(service_id) << std::endl;
        init();
        break;

    case PANEL_SERVICE_PRE_KILL:
        std::cout << "==== FlyingBrick: " << panel_service(service_id) << std::endl;
        deinit();
        break;
    }

    return true;
}
