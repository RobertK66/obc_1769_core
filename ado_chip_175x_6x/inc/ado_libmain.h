/*
 * ado_libmain.h
 *
 *  Created on: 12.05.2020
 *      Author: Robert
 */

#ifndef ADO_LIBMAIN_H_
#define ADO_LIBMAIN_H_

#define ADO_VERSION_MAJOR 	0
#define ADO_VERSION_MINOR 	9
#define ADO_VERSION_PATCH 	0
#define ADO_VERSION_RELEASE	"Dev"		// undefine this for a release ;-)

char* adoGetVersion(void);
const char* adoGetBuild(void);
const char* adoGetCompileConf(void);


#endif /* ADO_LIBMAIN_H_ */
