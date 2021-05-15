#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

#include <main.h>

//#include <stdlib.h>
#include "sensors/proximity.h"
#include "proximity_processor.h"

//***defines
#define SENSORTRIGGERHIGHVALUE 200		// value at which the sensor is considered as  triggered
#define SENSORTRIGGERLOWVALUE 100		// value under which the sensor is considered not triggered anymore
#define OBSTACLECLOSE 700				// value over which the obstacle is considered, very close
#define SENSORCORRECTION 0			//value used if different behaviour is desired from the front side and the 2 back side sensor
#define IRFILTER 0.5				// value for the simple lowpass filter y = IRFILTER*x+(1-IRFILTER)*y
#define SENSORMAXVALUE 3000			// value over which the data from the sensor is considered nonsense
#define BEHINDAREA 180				//value in degress when both backside senors are triggered



#define IR1 0
#define IR2 1
#define IR4 3
#define IR5 4
#define IR7 6
#define IR8 7


static THD_WORKING_AREA(waProximityProcessor, 256);
static THD_FUNCTION(ProximityProcessor, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_topic_t *proximityTopic;
     proximityTopic=messagebus_find_topic(&bus, "/proximity");
     proximity_msg_t Proximity_Values;

     int sensorValues[PROXIMITY_NB_CHANNELS]={0};					// array used to save filtered values of the sensors
     presenceOfObstacle_t triggeredSensors[PROXIMITY_NB_CHANNELS]={0};	//array used to save the trigger status of the sensor
     while(true)
     {
    	 messagebus_topic_wait(proximityTopic, &proximityValues, sizeof(proximityValues));		//wait for new data


    	 // provide additional information
    	 if(triggeredSensors[IR1]>NOOBSTACLE || triggeredSensors[IR8]>NOOBSTACLE)
    	 {
    		 processingValues.aheadIsOK=false;
    	 }
    	 else
    	 {
    		 processingValues.aheadIsOK=true;
    	 }
    	 processingValues.directionOfObstacle=processingValues.directionOfObstacle/nbOfTriggeredSensors;
     }
}






//***************Public functions *****************************************
void proximity_processor_start(void)
{
	proximity_start();
	chThdCreateStatic(waProximityProcessing, sizeof(waProximityProcessing), NORMALPRIO, ProximityProcessing, NULL);

}
presenceOfObstacle_t presenceOfObstacle(void)
{

	return processingValues.obstaclePresence;
}

void ignoreObstacle(bool doIgnoreObstacle)
{
	processingValues.ignoringObstacle=doIgnoreObstacle;
}

float getObstacleDirection(void)
{

	return  (float)processingValues.directionOfObstacle;
}
bool aheadIsOk(void)
{

	return processingValues.aheadIsOK;
}
