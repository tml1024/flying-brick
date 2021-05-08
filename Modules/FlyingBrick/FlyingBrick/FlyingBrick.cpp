// -*- comment-column: 50; fill-column: 110; c-basic-offset: 4; tab-width: 4; indent-tabs-mode: nil -*-

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
    DataDefinition0 = 1000,
    DataDefinitionVelocityAndAttitude,
    DataDefinitionAircraftState,
};

enum Event : DWORD {
    EventSixHz = 2000,
    EventFlightLoaded,
    EventFrame,
    EventPause,
    EventSimStart,
    EventSimStop,
    EventThrottle,
};

enum Group : DWORD {
    Group0 = 3000,
};

enum Request : DWORD {
    Request0 = 4000,
    RequestAircraftState,
};

struct Controls {
    double rudder;
    double aileron;
    double elevator;
    double leftBrake, rightBrake;
};

struct VelocityAndAttitude {
    double velX, velY, velZ;
    double heading;
    double bank, pitch;
};

struct AircraftState {
    Controls controls;
    VelocityAndAttitude velAndAtt;
};

static std::map<DWORD, std::string> calls;

static HRESULT record_call(const std::string &call,
                           HRESULT (*lambda)()) {
    HRESULT result;
    result = lambda();
    DWORD id;
    SimConnect_GetLastSentPacketID(hSimConnect, &id);
    calls[id] = call;

    return result;
}

#define ATTEMPT(call) \
    record_call(#call, []() { return call; })

static double rad2deg(double radians) {
    return radians / M_PI * 360;
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
            if ((++sixHzCounter % 100) != 0)
                break;
            std::cout << "==== FlyingBrick: EVENT SixHz (" << sixHzCounter << ")" << std::endl;
            break;

        case EventFlightLoaded:
            std::cout << "==== FlyingBrick: EVENT FlightLoaded" << std::endl;
            break;

        case EventFrame:
            static uint64_t frameCounter = 0;
            if ((++frameCounter % 1000) != 0)
                break;
            std::cout << "==== FlyingBrick: EVENT Frame (" << frameCounter << ")" << std::endl;
            break;

        case EventPause:
            std::cout << "==== FlyingBrick: EVENT Pause " << (event->dwData ? "ON" : "OFF") << std::endl;
            break;

        case EventSimStart:
            std::cout << "==== FlyingBrick: EVENT SimStart" << std::endl;
            if (!SUCCEEDED(ATTEMPT(SimConnect_RequestDataOnSimObject(hSimConnect,
                                                                     Request0, DataDefinition0,
                                                                     SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_ONCE))))
                std::cerr << "==== FlyingBrick: SimConnect_RequestDataOnSimObject(0) failed" << std::endl;
            if (!SUCCEEDED(ATTEMPT(SimConnect_RequestDataOnSimObject(hSimConnect,
                                                                     RequestAircraftState, DataDefinitionAircraftState,
                                                                     SIMCONNECT_OBJECT_ID_USER, SIMCONNECT_PERIOD_SIM_FRAME))))
                std::cerr << "==== FlyingBrick: SimConnect_RequestDataOnSimObject(1) failed" << std::endl;
            break;

        case EventSimStop:
            std::cout << "==== FlyingBrick: EVENT SimStop" << std::endl;
            break;

        case EventThrottle:
            std::cout << "==== FlyingBrick: EVENT Throttle " << event->dwData << std::endl;
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
        if ((++frameCounter % 1000) != 0)
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
        std::cout << "==== FlyingBrick: SIMOBJECT_DATA "
                  << data->dwRequestID << " "
                  << data->dwObjectID << " "
                  << data->dwDefineID << " "
                  << data_request_flags(data->dwFlags) << " "
                  << data->dwoutof << " " << data->dwDefineCount;
        switch (data->dwRequestID) {
        case Request0: {
            SIMCONNECT_DATA_INITPOSITION *pos = (SIMCONNECT_DATA_INITPOSITION*)&data->dwData;
            std::cout << " "
                      << abs(pos->Longitude) << (pos->Longitude < 0 ? "W" : "E") << " "
                      << abs(pos->Latitude) << (pos->Latitude < 0 ? "N" : "S") << " "
                      << pos->Altitude << " ft MSL";
            break;
        }
        case RequestAircraftState: {
            AircraftState *state = (AircraftState*)&data->dwData;
            std::cout << " rudder:" << std::setw(4) << int(100*state->controls.rudder)
                      << " aileron:" << std::setw(4) << int(100*state->controls.aileron)
                      << " elevator:" << std::setw(4) << int(100*state->controls.elevator)
                      // Show the effective value when using left and right brakes as a combined axis
                      << " brake:" << std::setw(4) << int(100*(state->controls.rightBrake - state->controls.leftBrake))
                      << " velocity:(" << std::setw(4) << int(state->velAndAtt.velX) << ","
                      << std::setw(4) << int(state->velAndAtt.velY) << ","
                      << std::setw(4) << int(state->velAndAtt.velZ) << ")"
                      << " heading:" << std::setw(3) << int(rad2deg(state->velAndAtt.heading))
                      << " bank:" << std::setw(4) << int(rad2deg(state->velAndAtt.bank))
                      << " pitch:" << std::setw(4) << int(rad2deg(state->velAndAtt.pitch));
            break;
        }
        default:
            std::cerr << " ? (" << data->dwRequestID << ")";
        }
        std::cout << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA_BYTYPE: {
        SIMCONNECT_RECV_SIMOBJECT_DATA_BYTYPE *data = (SIMCONNECT_RECV_SIMOBJECT_DATA_BYTYPE*)pData;
        std::cout << "==== FlyingBrick: SIMOBJECT_DATA_BYTYPE "
                  << data->dwRequestID << " "
                  << data->dwObjectID << " "
                  << data->dwDefineID << " "
                  << data_request_flags(data->dwFlags) << " "
                  << data->dwoutof << " " << data->dwDefineCount << std::endl;
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
                  << (calls.count(exception->dwSendID) ? calls[exception->dwSendID] : "?") << " "
                  << exception->dwIndex << std::endl;
        break;
    }
    case SIMCONNECT_RECV_ID_WEATHER_OBSERVATION: {
        SIMCONNECT_RECV_WEATHER_OBSERVATION *observation = (SIMCONNECT_RECV_WEATHER_OBSERVATION*)pData;
        std::cout << "==== FlyingBrick: WEATHER_OBSERVATION "
                  << observation->dwRequestID << " "
                  << observation->szMetar << std::endl;
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
                  << object_id->dwObjectID << std::endl;
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

    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSixHz, "6Hz")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(6Hz) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventFlightLoaded, "FlightLoaded")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(FlightLoaded) failed" << std::endl;
        return;
    }
#if 0
    // Yes this works, but we don't really need per-frame callbacks
    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventFrame, "Frame")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(Frame) failed" << std::endl;
        return;
    }
#endif

    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventPause, "Pause")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(Pause) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSimStart, "SimStart")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(SimStart) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_SubscribeToSystemEvent(hSimConnect, EventSimStop, "SimStop")))) {
        std::cerr << "==== FlyingBrick: SimConnect_SubscribeToSystemEvent(SimStop) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinition0,
                                                          "Initial Position", NULL,
                                                          SIMCONNECT_DATATYPE_INITPOSITION)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(Initial Position) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "RUDDER PEDAL POSITION", "position",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(rudder) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "AILERON POSITION", "position",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(aileron) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "ELEVATOR POSITION", "position",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(aileron) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "BRAKE LEFT POSITION EX1", "position",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(left brake) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "BRAKE RIGHT POSITION EX1", "position",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(right brake) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "VELOCITY WORLD X", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity x) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "VELOCITY WORLD X", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity x) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "VELOCITY WORLD Y", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity y) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "VELOCITY WORLD Y", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity y) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "VELOCITY WORLD Z", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity z) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "VELOCITY WORLD Z", "feet per second",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(velocity z) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "PLANE HEADING DEGREES TRUE", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(heading) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "PLANE HEADING DEGREES TRUE", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(heading) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "PLANE BANK DEGREES", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(bank) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "PLANE BANK DEGREES", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(bank) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionVelocityAndAttitude,
                                                          "PLANE PITCH DEGREES", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(pitch) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddToDataDefinition(hSimConnect, DataDefinitionAircraftState,
                                                          "PLANE PITCH DEGREES", "radians",
                                                          SIMCONNECT_DATATYPE_FLOAT64)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddToDataDefinition(pitch) failed" << std::endl;
        return;
    }

    // This doesn't seem to actually work?
    if (!SUCCEEDED(ATTEMPT(SimConnect_MapClientEventToSimEvent(hSimConnect, EventThrottle, "THROTTLE_SET")))) {
        std::cerr << "==== FlyingBrick: SimConnect_MapClientEventToSimEvent(throttle) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_AddClientEventToNotificationGroup(hSimConnect, Group0, EventThrottle)))) {
        std::cerr << "==== FlyingBrick: SimConnect_AddClientEventToNotificationGroup(group0, throttle) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_SetNotificationGroupPriority(hSimConnect, Group0, SIMCONNECT_GROUP_PRIORITY_HIGHEST)))) {
        std::cerr << "==== FlyingBrick: SimConnect_SetNotificationGroupPriority(group0) failed" << std::endl;
        return;
    }

    if (!SUCCEEDED(ATTEMPT(SimConnect_CallDispatch(hSimConnect, FlyingBrickDispatchProc, NULL)))) {
        std::cerr << "==== FlyingBrick: SimConnect_CallDispatch failed" << std::endl;
        return;
    }
}

extern "C" MSFS_CALLBACK void module_deinit(void) {
    if (!hSimConnect)
        return;

    if (!SUCCEEDED(SimConnect_Close(hSimConnect))) {
        std::cerr << "==== FlyingBrick: SimConnect_Close failed" << std::endl;
        return;
    }
}
