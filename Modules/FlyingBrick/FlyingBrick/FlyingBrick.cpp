// -*- comment-column: 50; fill-column: 110; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*-

#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "MSFS/MSFS.h"
#include "MSFS/MSFS_WindowsTypes.h"
#include "SimConnect.h"

#include "FlyingBrick.h"

static HANDLE hSimConnect = 0;

static bool quit = false;

// Use different numeric ranges for the enums to recognize the values if they turn on in unexpected places

enum DataDefinition : SIMCONNECT_DATA_DEFINITION_ID {
    DataDefinitionAircraftState = 1000,
    DataDefinitionControlsAndAircraftState,
};

enum Event : DWORD {
    EventSixHz = 2000,
    EventFlightLoaded,
    EventFrame,
    EventPause,
    EventSimStart,
    EventSimStop,
};

enum Request : DWORD {
    RequestControlsAndAircraftState = 3000,
};

constexpr auto EPSILON = 0.01;

struct Controls {
    double rudder;
    double aileron;
    double elevator;
    double leftBrake, rightBrake;
};

struct AircraftState {
    // Actual values
    double velBodyX, velBodyY, velBodyZ;
    double velWorldX, velWorldY, velWorldZ;
    double heading;
    double bank, pitch;
    double lat, lon, msl, agl, aglMinusCG;
    double gs;
    // Misc
    int64_t onGround;
    // Indications
    double kias, ktas;
    double alt;
    double vs;
};

struct ControlsAndAircraftState {
    Controls controls;
    AircraftState state;
};


static bool simRunning = false;
static bool simPaused = true;
static bool gotFirstState = false;

static AircraftState desiredState;

static std::map<DWORD, std::string> calls;

static HRESULT record_call(int lineNumber,
                           std::string call,
                           HRESULT value) {
    if (!SUCCEEDED(value)) {
        std::cerr << "==== FlyingBrick: The call '" << call << "' failed at line " << lineNumber << std::endl;
        return value;
    }
    DWORD id;
    static DWORD lastId = 0;

    SimConnect_GetLastSentPacketID(hSimConnect, &id);

    if (lastId > 0)
        assert(id > lastId);
    lastId = id;

    calls[id] = call;

    static uint64_t counter = 0;

    if ((++counter % 100) == 0 && id > 100)
    {
        auto oldSize = calls.size();
        auto newestToRemove = calls.upper_bound(id - 100);
        if (newestToRemove != calls.end())
            calls.erase(calls.begin(), newestToRemove);

        std::cout << "==== FlyingBrick: Pruned call history: " << oldSize << " => " << calls.size() << std::endl;
    }

    return value;
}

#define RECORD(expr) \
    record_call(__LINE__, #expr, expr);

static double rad2deg(double rad) {
    return rad / M_PI * 180;
}

static double deg2rad(double deg) {
    return deg / 180 * M_PI;
}

static double fps2fpm(double fps) {
    return fps * 60;
}

static double fpm2fps(double fpm) {
    return fpm / 60;
}

static double fps2kn(double fps) {
    return fps * 0.592484;
}

static double kn2fps(double kn) {
    return kn * 1.68781042021544;
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

static void MSFS_CALLBACK FlyingBrickDispatchProc(SIMCONNECT_RECV *pData, DWORD cbData, void *pContext) {
    HRESULT hr;

    switch(pData->dwID) {
    case SIMCONNECT_RECV_ID_OPEN: {
        SIMCONNECT_RECV_OPEN *open = (SIMCONNECT_RECV_OPEN*) pData;
        std::cout << "==== FlyingBrick: OPEN" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT: {
        SIMCONNECT_RECV_EVENT *event = (SIMCONNECT_RECV_EVENT*)pData;

        switch(event->uEventID) {
        case EventSixHz:
            static uint64_t sixHzCounter = 0;
            sixHzCounter++;
            if ((sixHzCounter % 100) != 0)
                break;
            std::cout << "==== FlyingBrick: EVENT SixHz (" << sixHzCounter << ")" << std::endl;
            break;

        case EventFlightLoaded:
            std::cout << "==== FlyingBrick: EVENT FlightLoaded" << std::endl;
            break;

        case EventFrame:
            static uint64_t frameCounter = 0;
            frameCounter++;
            if ((frameCounter % 1000) != 0)
                break;
            std::cout << "==== FlyingBrick: EVENT Frame (" << frameCounter << ")" << std::endl;
            break;

        case EventPause:
            std::cout << "==== FlyingBrick: EVENT Pause " << (event->dwData ? "ON" : "OFF") << std::endl;
            simPaused = event->dwData;
            if (simPaused)
                gotFirstState = false;
            break;

        case EventSimStart:
            std::cout << "==== FlyingBrick: EVENT SimStart" << std::endl;
            RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                                     RequestControlsAndAircraftState, DataDefinitionControlsAndAircraftState,
                                                     SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                                     SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                                     0));
            simRunning = true;
            break;

        case EventSimStop:
            std::cout << "==== FlyingBrick: EVENT SimStop" << std::endl;
            // Effectively unsubscribe to this data by (re-)requesting it with a very high interval
            RECORD(SimConnect_RequestDataOnSimObject(hSimConnect,
                                                     RequestControlsAndAircraftState, DataDefinitionControlsAndAircraftState,
                                                     SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME,
                                                     SIMCONNECT_DATA_REQUEST_FLAG_CHANGED, 0,
                                                     DWORD_MAX));
            simRunning = false;
            gotFirstState = false;
            break;

        default:
            std::cout << "==== FlyingBrick: EVENT " << event->uEventID << " " << event->dwData << std::endl;
            break;
        }
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_FILENAME: {
        SIMCONNECT_RECV_EVENT_FILENAME *filename = (SIMCONNECT_RECV_EVENT_FILENAME*)pData;
        std::cout << "==== FlyingBrick: EVENT_FILENAME "
                  << filename->szFileName;
        switch (filename->uEventID) {
        case EventFlightLoaded:
            std::cout << " FlightLoaded";
            break;
        default:
            std::cout << " " << filename->uEventID;
            break;
        }
        std::cout << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_OBJECT_ADDREMOVE: {
        SIMCONNECT_RECV_EVENT_OBJECT_ADDREMOVE *object = (SIMCONNECT_RECV_EVENT_OBJECT_ADDREMOVE*)pData;
        std::cout << "==== FlyingBrick: EVENT_OBJECT_ADDREMOVE "
                  << object->uEventID << " " << simobject_type(object->eObjType) << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_FRAME: {
        static uint64_t frameCounter = 0;
        frameCounter++;
        if ((frameCounter % 1000) != 0)
            break;
        SIMCONNECT_RECV_EVENT_FRAME *frame = (SIMCONNECT_RECV_EVENT_FRAME*)pData;
        std::cout << "==== FlyingBrick: EVENT_FRAME ("
                  << frameCounter << ") "
                  << frame->uEventID << " "
                  << frame->fFrameRate << " "
                  << frame->fSimSpeed << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        SIMCONNECT_RECV_SIMOBJECT_DATA *data = (SIMCONNECT_RECV_SIMOBJECT_DATA*)pData;
        switch (data->dwRequestID) {
        case RequestControlsAndAircraftState: {
            ControlsAndAircraftState *state = (ControlsAndAircraftState*)&data->dwData;
            
            static uint64_t counter = 0;
            counter++;
            if ((counter % 10) == 0)
                std::cout << "==== FlyingBrick: SIMOBJECT_DATA RequestControlsAndAircraftState "
                          << " rudder:" << std::fixed << std::setw(5) << std::setprecision(2) << state->controls.rudder
                          << " aileron:" << std::fixed << std::setw(5) << std::setprecision(2) << state->controls.aileron
                          << " elevator:" << std::fixed << std::setw(5) << std::setprecision(2) << state->controls.elevator
                          << " brakes:(" << std::fixed << std::setw(4) << std::setprecision(0) << int(100*state->controls.leftBrake) << ","
                          <<                std::fixed << std::setw(4) << std::setprecision(0) << int(100*state->controls.rightBrake) << ")"
                          << " velBody:(" << std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyX) << ","
                          <<                 std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyY) << ","
                          <<                 std::fixed << std::setw(4) << std::setprecision(2) << int(state->state.velBodyZ) << ")"
                          << " velWorld:(" << std::setw(4) << std::setprecision(2) << int(state->state.velWorldX) << ","
                          <<               std::setw(4) << std::setprecision(2) << int(state->state.velWorldY) << ","
                          <<               std::setw(4) << std::setprecision(2) << int(state->state.velWorldZ) << ")"
                          << " heading:" << std::setw(3) << int(rad2deg(state->state.heading))
                          << " bank:" << std::setw(4) << int(rad2deg(state->state.bank))
                          << " pitch:"  << std::setw(4) << int(rad2deg(state->state.pitch))
                          << std::endl;

            std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
            static std::chrono::time_point<std::chrono::steady_clock> lastTime = now;
            
            if (simRunning && !simPaused) {
                if (!gotFirstState) {
                    // Set the desired initial state: motionless
                    desiredState = state->state;
                    desiredState.velBodyX = desiredState.velBodyY = desiredState.velBodyZ = 0;
                    desiredState.velWorldX = desiredState.velWorldY = desiredState.velWorldZ = 0;
                    desiredState.heading = desiredState.bank = desiredState.pitch = 0;
                    desiredState.gs = desiredState.kias = desiredState.ktas = 0;
                    desiredState.vs = 0;

                    gotFirstState = true;
                } else {
                    auto timeSinceLast = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime);

                    // Obviously we can turn and move only in the air
                    if (!state->state.onGround) {
                        // Arbitrary choice in this toy: Full rudder pedal deflection means 45 degrees per
                        // second yaw rate.
                        if (std::fabs(state->controls.rudder) > EPSILON) {
                            auto diff = state->controls.rudder * timeSinceLast.count() / 1000 * deg2rad(45);
                            std::cout << "==== FlyingBrick: heading difference: " << diff << " rad" << std::endl;
                            desiredState.heading += diff;
                        }

                        // Another arbitrary choice: Full elevator deflection means 50 knots GS. Ditto for full aileron deflection.
                        if (std::abs(state->controls.elevator) > EPSILON || std::abs(state->controls.aileron) > EPSILON) {
                            desiredState.velBodyZ = state->controls.elevator * timeSinceLast.count() / 1000 * kn2fps(50);
                            desiredState.velBodyX = state->controls.aileron * timeSinceLast.count() / 1000 * kn2fps(50);

                            const double bodyRelativeAbsoluteVelocity = std::sqrt(desiredState.velBodyZ * desiredState.velBodyZ
                                                                                  + desiredState.velBodyX * desiredState.velBodyX);
                            const double bodyRelativeTrack = M_PI/2 - std::atan2(desiredState.velBodyZ, desiredState.velBodyX);
                            const double worldRelativeTrack = desiredState.heading + bodyRelativeTrack;
                            desiredState.velWorldZ = std::cos(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;
                            desiredState.velWorldX = std::sin(worldRelativeTrack) * bodyRelativeAbsoluteVelocity;

                            std::cout << "==== FlyingBrick: velBody(Z,X):(" << desiredState.velBodyZ << "," << desiredState.velBodyX << ")"
                                      << " bodyRelativeAbsoluteVelocity:" << bodyRelativeAbsoluteVelocity
                                      << " bodyRelativeTrack:" << bodyRelativeTrack
                                      << " worldRelativeTrack:" << worldRelativeTrack
                                      << " velWorld(Z,X):(" << desiredState.velWorldZ << "," << desiredState.velWorldX << ")" << std::endl;

                            desiredState.kias = fps2kn(bodyRelativeAbsoluteVelocity);
                            // Assume this aircraft is used only at low altitudes and ignore wind
                            desiredState.ktas = desiredState.gs = desiredState.kias;
                        }
                    }

                    // Vertical velocity however can be changed while on the ground. We can lift off. Full
                    // right or left brake means plus or minus 500 fpm.
                    if ((!state->state.onGround && std::abs(state->controls.rightBrake - state->controls.leftBrake) > EPSILON)
                        || (state->state.onGround && state->controls.rightBrake - state->controls.leftBrake > EPSILON)) {
                        desiredState.velBodyY = (state->controls.rightBrake - state->controls.leftBrake) * timeSinceLast.count() / 1000 * fpm2fps(500);
                        desiredState.velWorldY = desiredState.velBodyY;
                        desiredState.vs = fps2fpm(desiredState.velBodyY);
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
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA_BYTYPE: {
        SIMCONNECT_RECV_SIMOBJECT_DATA_BYTYPE *data = (SIMCONNECT_RECV_SIMOBJECT_DATA_BYTYPE*)pData;
        std::cout << "==== FlyingBrick: SIMOBJECT_DATA_BYTYPE "
                  << data->dwRequestID << " "
                  << data->dwObjectID << " "
                  << data->dwDefineID << " "
                  << data_request_flags(data->dwFlags) << " "
                  << data->dwoutof << " " << data->dwDefineCount
                  << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_QUIT: {
        std::cout << "==== FlyingBrick: QUIT" << std::endl;
        quit = true;
        break;
    }
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        SIMCONNECT_RECV_EXCEPTION *exception = (SIMCONNECT_RECV_EXCEPTION*)pData;
        std::cout << "==== FlyingBrick: EXCEPTION "
                  << exception_type(exception->dwException) << " "
                  << (calls.count(exception->dwSendID) ? calls[exception->dwSendID] : "at unknown line") << " "
                  << exception->dwIndex
                  << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_WEATHER_OBSERVATION: {
        SIMCONNECT_RECV_WEATHER_OBSERVATION *observation = (SIMCONNECT_RECV_WEATHER_OBSERVATION*)pData;
        std::cout << "==== FlyingBrick: WEATHER_OBSERVATION "
                  << observation->dwRequestID << " "
                  << observation->szMetar
                  << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_CLOUD_STATE: {
        SIMCONNECT_RECV_CLOUD_STATE *cloud_state = (SIMCONNECT_RECV_CLOUD_STATE*)pData;
        std::cout << "==== FlyingBrick: CLOUD_STATE" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_ASSIGNED_OBJECT_ID: {
        SIMCONNECT_RECV_ASSIGNED_OBJECT_ID *object_id = (SIMCONNECT_RECV_ASSIGNED_OBJECT_ID*)pData;
        std::cout << "==== FlyingBrick: ASSIGNED_OBJECT_ID "
                  << object_id->dwRequestID << " "
                  << object_id->dwObjectID
                  << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_RESERVED_KEY: {
        SIMCONNECT_RECV_RESERVED_KEY *key = (SIMCONNECT_RECV_RESERVED_KEY*)pData;
        std::cout << "==== FlyingBrick: RESERVED_KEY" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_CUSTOM_ACTION: {
        SIMCONNECT_RECV_CUSTOM_ACTION *action = (SIMCONNECT_RECV_CUSTOM_ACTION*)pData;
        std::cout << "==== FlyingBrick: CUSTOM_ACTION" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_SYSTEM_STATE: {
        SIMCONNECT_RECV_SYSTEM_STATE *state = (SIMCONNECT_RECV_SYSTEM_STATE*)pData;
        std::cout << "==== FlyingBrick: SYSTEM_STATE" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_CLIENT_DATA: {
        SIMCONNECT_RECV_CLIENT_DATA *data = (SIMCONNECT_RECV_CLIENT_DATA*)pData;
        std::cout << "==== FlyingBrick: CLIENT_DATA" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_WEATHER_MODE: {
        SIMCONNECT_RECV_EVENT_WEATHER_MODE *mode = (SIMCONNECT_RECV_EVENT_WEATHER_MODE*)pData;
        std::cout << "==== FlyingBrick: EVENT_WEATHER_MODE" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_AIRPORT_LIST: {
        SIMCONNECT_RECV_AIRPORT_LIST *airport_list = (SIMCONNECT_RECV_AIRPORT_LIST*)pData;
        std::cout << "==== FlyingBrick: AIRPORT_LIST" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_VOR_LIST: {
        SIMCONNECT_RECV_VOR_LIST *vor_list = (SIMCONNECT_RECV_VOR_LIST*)pData;
        std::cout << "==== FlyingBrick: VOR_LIST" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_NDB_LIST: {
        SIMCONNECT_RECV_NDB_LIST *ndb_list = (SIMCONNECT_RECV_NDB_LIST*)pData;
        std::cout << "==== FlyingBrick: NDB_LIST" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_WAYPOINT_LIST: {
        SIMCONNECT_RECV_WAYPOINT_LIST *waypoint_list = (SIMCONNECT_RECV_WAYPOINT_LIST*)pData;
        std::cout << "==== FlyingBrick: WAYPOINT_LIST" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_MULTIPLAYER_SERVER_STARTED: {
        SIMCONNECT_RECV_EVENT_MULTIPLAYER_SERVER_STARTED *server_started = (SIMCONNECT_RECV_EVENT_MULTIPLAYER_SERVER_STARTED*)pData;
        std::cout << "==== FlyingBrick: EVENT_MULTIPLAYER_SERVER_STARTED" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_MULTIPLAYER_CLIENT_STARTED: {
        SIMCONNECT_RECV_EVENT_MULTIPLAYER_CLIENT_STARTED *client_started = (SIMCONNECT_RECV_EVENT_MULTIPLAYER_CLIENT_STARTED*)pData;
        std::cout << "==== FlyingBrick: EVENT_MULTIPLAYER_CLIENT_STARTED" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_MULTIPLAYER_SESSION_ENDED: {
        SIMCONNECT_RECV_EVENT_MULTIPLAYER_SESSION_ENDED *session_ended = (SIMCONNECT_RECV_EVENT_MULTIPLAYER_SESSION_ENDED*)pData;
        std::cout << "==== FlyingBrick: EVENT_MULTIPLAYER_SESSION_ENDED" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_RACE_END: {
        SIMCONNECT_RECV_EVENT_RACE_END *race_end = (SIMCONNECT_RECV_EVENT_RACE_END*)pData;
        std::cout << "==== FlyingBrick: EVENT_RACE_END" << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_EVENT_RACE_LAP: {
        SIMCONNECT_RECV_EVENT_RACE_LAP *race_lap = (SIMCONNECT_RECV_EVENT_RACE_LAP*)pData;
        std::cout << "==== FlyingBrick: EVENT_RACE_LAP" << std::endl;
        break;
    }
    default:
        std::cerr << "==== FlyingBrick: Got unknown SIMCONNECT_RECV " << pData->dwID << std::endl;
        break;
    }
}

extern "C" MSFS_CALLBACK void module_init(void) {
    HRESULT hr;

    if (hSimConnect == 0) {
        if (!SUCCEEDED(SimConnect_Open(&hSimConnect, "FlyingBrick", nullptr, 0, 0, 0))) {
            std::cerr << "==== FlyingBrick: SimConnect_Open failed" << std::endl;
            return;
        }
        std::cout << "==== FlyingBrick: Connected" << std::endl;
    }

    // Most likely it is pointless to check the return values from these SimConnect calls. It seems that
    // errors in parameters are reported asynchronously anyway as SIMCONNECT_RECV_ID_EXCEPTION.

    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSixHz, "6Hz"));
    
    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventFlightLoaded, "FlightLoaded"));

    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventPause, "Pause"));
    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSimStart, "SimStart"));
    RECORD(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSimStop, "SimStop"));

    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionControlsAndAircraftState,
                                          "RUDDER PEDAL POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionControlsAndAircraftState,
                                          "AILERON POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionControlsAndAircraftState,
                                          "ELEVATOR POSITION", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionControlsAndAircraftState,
                                          "BRAKE LEFT POSITION EX1", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64));
    RECORD(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionControlsAndAircraftState,
                                          "BRAKE RIGHT POSITION EX1", "position",
                                          SIMCONNECT_DATATYPE_FLOAT64));

    for (auto definition: {DataDefinitionAircraftState, DataDefinitionControlsAndAircraftState}) {
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY BODY Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD X", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Y", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VELOCITY WORLD Z", "feet/second",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE HEADING DEGREES TRUE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE BANK DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE PITCH DEGREES", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LATITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE LONGITUDE", "radians",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE ALT ABOVE GROUND", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "PLANE ALT ABOVE GROUND MINUS CG", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "GROUND VELOCITY", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64));

        // Use only 64-bit types so that the sizes of the structs (without any packing pragmas) match what
        // SimConnect wants.
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "SIM ON GROUND", "boolean",
                                              SIMCONNECT_DATATYPE_INT64));

        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED INDICATED", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "AIRSPEED TRUE", "knots",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "INDICATED ALTITUDE", "feet",
                                              SIMCONNECT_DATATYPE_FLOAT64));
        RECORD(SimConnect_AddToDataDefinition(hSimConnect, definition,
                                              "VERTICAL SPEED", "feet/minute",
                                              SIMCONNECT_DATATYPE_FLOAT64));
    }
    RECORD(SimConnect_CallDispatch(hSimConnect, FlyingBrickDispatchProc, NULL));
}

extern "C" MSFS_CALLBACK void module_deinit(void) {
    if (!hSimConnect)
        return;

    if (!SUCCEEDED(SimConnect_Close(hSimConnect))) {
        std::cerr << "==== FlyingBrick: SimConnect_Close failed" << std::endl;
        return;
    }
}

