/*
 * IParametersProvider.h
 *
 *  Created on: Dec 3, 2014
 *      Author: blackpc
 */

#ifndef IPARAMETERSPROVIDER_H_
#define IPARAMETERSPROVIDER_H_


#include <string>


using namespace std;


/*
 * Cost map parameters provider interface
 */
class IParametersProvider {

public:

	virtual ~IParametersProvider() { }

public:

	/**
	 * Gets map width in meters
	 * @return
	 */
	virtual double getMapWidth() const = 0;

	/**
	 * Gets map height in meters
	 * @return
	 */
	virtual double getMapHeight() const = 0;

	/**
	 * Gets map resolution in p/m
	 * @return
	 */
	virtual double getMapResolution() const = 0;

	/**
	 * Gets obstacle inflation radius in meters
	 * @return
	 */
	virtual double getInflationRadius() const = 0;

	/**
	 * Gets the frame id of the cost map
	 * @return
	 */
	virtual string getMapFrameId() const = 0;

};

#endif /* IPARAMETERSPROVIDER_H_ */
