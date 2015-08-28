#ifndef QUADCOPTERCONTROLLER_H
#define QUADCOPTERCONTROLLER_H

/*--- Objects needed to maintain all elements working. ---*/
#include "TelemetryController.h"

#include "AHRSBaseSystem.h"

struct QuadcopterState {


};

class QuadcopterController
{
public:
    QuadcopterController(AHRS::AHRSBaseSystem<float, unsigned int> &AHRSSystem, TelemetryController &telemetryController);
    ~QuadcopterController();

    /**
     * @brief update
     * Do the main functions of the quadcoptero control. This function must be
     * called periodically, with a known and stable period, and will be responsable
     * to maintain the control-loop.
     * This functions handles all the control so the it must be a Hard Real
     * time method. This class will use the TelemetryController to
     * do all the taks related with this.
     */
    void controlUpdate( void );

    /**
     * @brief telemetryUpdate
     * Do all the communication data trade-off with the telemetry link.
     * This method will be responsable of updating the Telemetry Link state and
     * maintain the data flow within the telemetry base. The data, basicallty,
     * consists of commands received by the base and the state data sent,
     * containing all kind of informations about the current status of the drone.
     * This functions must be called as fast as possible and will internally control
     * it's period. (Soft- real-time).
     */
    inline void telemetryUpdate( void );

    /**
     * @brief setAHRSValues
     * Receive the newest values obtained by the AHRS System. These values
     * must constains all the data needed by the control loop, like the attitude
     * of the drone (Stabilty control) and it's position (provided byte and GPS,
     * for example). This function, basically, will set the measured state
     * for the controllers.
     */
    void setStateValues(const QuadcopterState &newState);

private:
    /*-- Variables related with the state of the drone. --*/
    AHRS::AHRSBaseSystem<float, unsigned int> &m_ahrsSystem;
    QuadcopterState *m_droneState; /** This variable will store the last received state. */

    /*-- Variables and objected related of the modules of the drone. --*/
    TelemetryController &m_telemetry; /** This variable represents the RF Link with the telemetry base.
                                        It will be responsable for send the data (State) and received
                                        the commands of the base. This objects abstracts the link with
                                        the base. */


};

void QuadcopterController::telemetryUpdate( void )
{
    m_telemetry.update();
}

#endif // QUADCOPTERCONTROLLER_H
