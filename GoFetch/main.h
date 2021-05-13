#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define ZERO 					0

typedef int32_t		Angle;
typedef int			Speed;
typedef int8_t 		Direction; //utile?????
typedef uint8_t		Bool;

typedef struct Position {
	uint16_t xPosition;
	uint16_t yPosition;
} Position;

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
