/*
 * ParamServer.h
 *
 *  Created on: Apr 17, 2019
 *      Author: rayvburn
 */

#ifndef INCLUDE_CORE_PARAMSERVER_H_
#define INCLUDE_CORE_PARAMSERVER_H_

namespace actor {
namespace core {

/* Helper class created to prevent the main one (actor::core::Actor)
 * pollution with a plenty of parameter variables */
class ParamServer {

public:

	ParamServer();

	virtual ~ParamServer();

};

} /* namespace core */
} /* namespace actor */

#endif /* INCLUDE_CORE_PARAMSERVER_H_ */
