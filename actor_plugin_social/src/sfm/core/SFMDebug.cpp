/*
 * SFMDebug.cpp
 *
 *  Created on: Apr 7, 2019
 *      Author: rayvburn
 */

#include "sfm/core/SFMDebug.h"

void SfmDebugSetCurrentActorName(const std::string &name) {
	debug_current_actor_name = name;
}
void SfmDebugSetCurrentObjectName(const std::string &name) {
	debug_current_object_name = name;
}
void SfmSetPrintData(const bool &to_print) {
	print_data_dbg = to_print;
}


std::string SfmDebugGetCurrentActorName() {
	return(debug_current_actor_name);
}
std::string SfmDebugGetCurrentObjectName() {
	return(debug_current_object_name);
}
bool SfmGetPrintData() {
	return (print_data_dbg);
}

