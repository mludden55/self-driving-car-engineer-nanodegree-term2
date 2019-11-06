/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"


using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
     // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	default_random_engine gen;

     // Extracting standard deviations
	normal_distribution<double> N_x(x, std[0]);
	normal_distribution<double> N_y(y, std[1]);
	normal_distribution<double> N_theta(theta, std[2]);

	// initialize particles
	for (int i = 0; i < num_particles; i++) {
	     Particle particle;
		particle.id = i;
		particle.x = N_x(gen);
		particle.y = N_y(gen);
		particle.theta = N_theta(gen);
		particle.weight = 1.0;

		particles.push_back(particle);
		weights.push_back(1);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

     // extracting standard deviations
     double std_x = std_pos[0];
     double std_y = std_pos[1];
     double std_theta = std_pos[2];

     // Creating normal distributions
     normal_distribution<double> dist_x(0, std_x);
     normal_distribution<double> dist_y(0, std_y);
     normal_distribution<double> dist_theta(0, std_theta);

	// loop through and calculate new state
	for (int i = 0; i < num_particles; i++) {
	     double new_x;
		double new_y;
		double new_theta;

		if (yaw_rate == 0) {
			new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			new_theta = particles[i].theta;
		}
		else {
			new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			new_theta = particles[i].theta + yaw_rate * delta_t;
		}

		normal_distribution<double> N_x(new_x, std_pos[0]);
		normal_distribution<double> N_y(new_y, std_pos[1]);
		normal_distribution<double> N_theta(new_theta, std_pos[2]);

		particles[i].x = N_x(gen);
		particles[i].y = N_y(gen);
		particles[i].theta = N_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	int nObservations = observations.size();
     int nPredictions = predicted.size();

 	// loop through observations
	for (int i = 0; i < nObservations; i++) {

		// initialize min distance to maximum
		double minDistance = numeric_limits<double>::max();

		// initialize the found map
		int mapId = -1;

		for (int j = 0; j < nPredictions; j++ ) { // For each predition.

			double xDistance = observations[i].x - predicted[j].x;
			double yDistance = observations[i].y - predicted[j].y;
			double distance = xDistance * xDistance + yDistance * yDistance;

			// find the landmark closest to current observed landmark
			if ( distance < minDistance ) {
				minDistance = distance;
				mapId = predicted[j].id;
			}
		}

		// update the observation to nearest predicted landmark
		observations[i].id = mapId;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
	const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	double stdLandmarkRange = std_landmark[0];
	double stdLandmarkBearing = std_landmark[1];

	for (int i = 0; i < num_particles; i++) {

		double x = particles[i].x;
		double y = particles[i].y;
		double theta = particles[i].theta;

		// find landmarks in particle's range
		double sensor_range_2 = sensor_range * sensor_range;
		vector<LandmarkObs> inRangeLandmarks;
		for(int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			float landmarkX = map_landmarks.landmark_list[j].x_f;
			float landmarkY = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;
			double dX = x - landmarkX;
			double dY = y - landmarkY;
			if (dX*dX + dY*dY <= sensor_range_2) {
				inRangeLandmarks.push_back(LandmarkObs{ id, landmarkX, landmarkY });
			}
		}

		// transform observation coordinates
		vector<LandmarkObs> mappedObservations;
		for(int k = 0; k < observations.size(); k++) {
			double xx = cos(theta)*observations[k].x - sin(theta)*observations[k].y + x;
			double yy = sin(theta)*observations[k].x + cos(theta)*observations[k].y + y;
			mappedObservations.push_back(LandmarkObs{ observations[k].id, xx, yy });
		}

		// run dataAssociation for predictions and transformed observations
		dataAssociation(inRangeLandmarks, mappedObservations);

		// reinitilaze weight
		particles[i].weight = 1.0;

		for(int l = 0; l < mappedObservations.size(); l++) {
			double observationX = mappedObservations[l].x;
			double observationY = mappedObservations[l].y;

			int landmarkId = mappedObservations[l].id;

			double landmarkX, landmarkY;
			int m = 0;
			int nLandmarks = inRangeLandmarks.size();
			bool found = false;
			while(!found && m < nLandmarks) {
				if ( inRangeLandmarks[m].id == landmarkId) {
					found = true;
					landmarkX = inRangeLandmarks[m].x;
					landmarkY = inRangeLandmarks[m].y;
				}
				m++;
			}

			// Calculating weight.
			double dX = observationX - landmarkX;
			double dY = observationY - landmarkY;

			double weight = ( 1/(2*M_PI*stdLandmarkRange*stdLandmarkBearing)) * exp( -( dX*dX/(2*stdLandmarkRange*stdLandmarkRange) + (dY*dY/(2*stdLandmarkBearing*stdLandmarkBearing)) ) );
			particles[i].weight *= weight;

		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	vector<double> weights;
	double maxWeight = numeric_limits<double>::min();
	for(int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
		if ( particles[i].weight > maxWeight ) {
			maxWeight = particles[i].weight;
		}
	}

	// Creating distributions
	uniform_real_distribution<double> distDouble(0.0, maxWeight);
	uniform_int_distribution<int> distInt(0, num_particles - 1);

	// generating index.
	int index = distInt(gen);

	double beta = 0.0;

	// resample wheel
	vector<Particle> resampledParticles;
	for(int i = 0; i < num_particles; i++) {
		beta += distDouble(gen) * 2.0;
		while( beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resampledParticles.push_back(particles[index]);
	}

	particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	particle.associations= associations;
	particle.sense_x = sense_x;
	particle.sense_y = sense_y;

	return particle;
}

string ParticleFilter::getAssociations(Particle best){
	vector<int> v = best.associations;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseX(Particle best){
	vector<double> v = best.sense_x;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}

string ParticleFilter::getSenseY(Particle best){
	vector<double> v = best.sense_y;
	stringstream ss;
	copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
	string s = ss.str();
	s = s.substr(0, s.length()-1);  // get rid of the trailing space
	return s;
}
