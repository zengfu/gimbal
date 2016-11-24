#include "stm32f1xx_hal.h"
#include "newcontrol.h"

float pidCmd[3];

float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};

void computeMotorCommands(float dt)
{
  pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
                                  sensors.margAttitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
                                  dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);
}

float updatePID(float command,/*desired*/
								float state,/*test*/ 
								float deltaT,
								uint8_t iHold, 
								struct PIDdata *PIDparameters)
{
    float error;
    float dTerm;
    float dTermFiltered;
    float dAverage;

	
    error = command - state;

    if (PIDparameters->type == ANGULAR)
        error = standardRadianFormat(error);

    ///////////////////////////////////

    if (iHold == false)
    {
    	PIDparameters->iTerm += error * deltaT;
    	PIDparameters->iTerm = constrain(PIDparameters->iTerm, -PIDparameters->windupGuard, PIDparameters->windupGuard);//?y¡¤??T¡¤¨´
    }

    ///////////////////////////////////
	
    if (PIDparameters->dErrorCalc == D_ERROR)  // Calculate D term from error
    {
			//¨ª¡§1yerror?????¡é¡¤???¡ê?1?¨º?¡ê¡§de/dt¡ê?¡ê????Dde?¨ª¨º?(error - PIDparameters->lastDcalcValue) 
		dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
        PIDparameters->lastDcalcValue = error;//¡À¡ê¡ä?¦Ì¡À?¡ã??2?
	}
	else                                       // Calculate D term from state
	{
		//¡¤??¨°¡ê?¨º1¨®????a¨ºy????¦Ì??¨²D¦Ì???¨¨¡Áa??3¨¦¦Ì?¡Á¨®???¨¨¦Ì??¦Ì??DD?¡é¡¤?????
		dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;

		if (PIDparameters->type == ANGULAR)//¨¨?1?¨¤¨¤D¨ª?a???¨¨
		    dTerm = standardRadianFormat(dTerm);//¡Áa??3¨¦¡À¨º¡Á????¨¨

		PIDparameters->lastDcalcValue = state;//¡À¡ê¡ä?¦Ì¡À?¡ã¡Á¡ä¨¬?
	}

    ///////////////////////////////////
		//???¡é¡¤?????DD¨°??¡Á¦Ì¨ª¨ª¡§??2¡§ deltaT / (rc + deltaT) ?¨¢1??¨ª¨º???2¡§?¦Ì¨ºya ¡ê? rc=1.0f/(2.0f*PI*F)
    dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);

	//??¨¤¨²¨º¡¤¨¨y¡ä??¡é¡¤?????DD?¨®???¨´
    dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

    PIDparameters->lastLastDterm = PIDparameters->lastDterm;//¨¦?¡ä??¡é¡¤???¡À¡ê¡ä?¦Ì?¨¦?¨¦?¡ä??¡é¡¤???¡ä?¡ä¡é¡À?¨¢?
    PIDparameters->lastDterm = dTermFiltered;//¦Ì¡À?¡ã?¡é¡¤???¡À¡ê¡ä?¦Ì?¨¦?¡ä??¡é¡¤???¡ä?¡ä¡é¡À?¨¢?

    ///////////////////////////////////
//¡¤¦Ì??PID?????¨¢1?
    if (PIDparameters->type == ANGULAR)//¨¨?1?¨¤¨¤D¨ª?a???¨¨
        return(PIDparameters->P * error     /*  Kp*e  */           +
	           PIDparameters->I * PIDparameters->iTerm + /*   Ki*?¨°edt   */
	           PIDparameters->D * dAverage);/*   Kd*¡ê¡§de/dt¡ê?  */
    else
        return(PIDparameters->P * PIDparameters->B * command /* Kp *(B * point) */  +
               PIDparameters->I * PIDparameters->iTerm /*   Ki*?¨°edt   */      +
               PIDparameters->D * dAverage   /*   Kd*¡ê¡§de/dt¡ê?  */ -
               PIDparameters->P * state);//??????¨¢?

    ///////////////////////////////////
}
