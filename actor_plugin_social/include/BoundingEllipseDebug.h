/*
 * BoundingEllipseDebug.h
 *
 *  Created on: Apr 1, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_BOUNDINGELLIPSEDEBUG_H_
#define INCLUDE_BOUNDINGELLIPSEDEBUG_H_

static bool to_print_ellipse_info = false;

void debugEllipseSet(bool status) {
	to_print_ellipse_info = status;
}

bool debugEllipseGet() {
	return (to_print_ellipse_info);
}

#endif /* INCLUDE_BOUNDINGELLIPSEDEBUG_H_ */
