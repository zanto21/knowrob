#ifndef __KNOWROB_TF_LOGGER__
#define __KNOWROB_TF_LOGGER__

#include <string>

// MONGO
#include <mongoc.h>
// ROS
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
// SWI Prolog
#define PL_SAFE_ARG_MACROS
#include <SWI-cpp.h>

#include <knowrob/ros/tf/memory.h>

/**
 * A TF listener that stores messages in MongoDB.
 */
class TFLogger
{
public:
	TFLogger(ros::NodeHandle &node,
			TFMemory &memory,
			const std::string &topic="tf");
	~TFLogger();

	void set_db_name(const std::string &db_name)
	{ db_name_ = db_name; }

	const std::string& get_db_name()
	{ return db_name_; }

	void set_time_threshold(double v)
	{ timeThreshold_ = v; }

	double get_time_threshold() const
	{ return timeThreshold_; }

	void set_vectorial_threshold(double v)
	{ vectorialThreshold_ = v; }

	double get_vectorial_threshold() const
	{ return vectorialThreshold_; }

	void set_angular_threshold(double v)
	{ angularThreshold_ = v; }

	double get_angular_threshold() const
	{ return angularThreshold_; }

	void store(const geometry_msgs::TransformStamped &ts);

protected:
	// The ordering of the variables is important.
	// Otherwise there will be a race condition.
	// memory_ needs the db_name_ to be initialized
	// and the subscribers need the memory_ to be initialized
	// and subscriber_ needs topic_ to be initialized
	// since they run on different threads, this doesn't happen
	// all the time and not even on all devices
	// and only if there is tf data published right when the logger is starting.
	double vectorialThreshold_;
	double angularThreshold_;
	double timeThreshold_;
	std::string db_name_;
	std::string topic_;
	TFMemory &memory_;
	ros::Subscriber subscriber_;
	ros::Subscriber subscriber_static_;

	char buf_[16];
	size_t keylen_;

	void store_document(bson_t *doc);

	bool ignoreTransform(const geometry_msgs::TransformStamped &ts);

	void callback(const tf::tfMessage::ConstPtr& msg);

	void appendTransform(bson_t *ts_doc,
			const geometry_msgs::TransformStamped &ts);
};

#endif //__KNOWROB_TF_LOGGER__
