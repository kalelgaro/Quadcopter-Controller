#include "QuadcopterController.h"

QuadcopterController::QuadcopterController(AHRS::AHRSBaseSystem<float, unsigned int> &AHRSSystem, TelemetryController &telemetryController) :
    m_ahrsSystem( AHRSSystem )
    , m_droneState( new QuadcopterState() )
    , m_telemetry( telemetryController )
{

}

QuadcopterController::~QuadcopterController()
{

}

