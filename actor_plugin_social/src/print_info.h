/*
 * print_info.h
 *
 *  Created on: Feb 15, 2019
 *      Author: jarek
 */

#ifndef SRC_PRINT_INFO_H_
#define SRC_PRINT_INFO_H_

bool print = false;

void Print_Set(bool state) {
	print = state;
}

bool Print_Get() {
	return print;
}

#endif /* SRC_PRINT_INFO_H_ */
