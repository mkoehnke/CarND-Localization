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

// Random noise generator
static default_random_engine gen;

// Initial weight
const double DEFAULT_WEIGHT = 1.0;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    // Specify number of particles
    num_particles = 120;
    
    // Creates a normal (Gaussian) distribution for x,y,theta (sensor noise)
    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);
    
    // Initialize particles
    for (int i=0; i<num_particles;i++) {
        Particle particle;
        particle.id = i;
        particle.x = x + dist_x(gen);
        particle.y = y + dist_y(gen);
        particle.theta = theta + dist_theta(gen);
        particle.weight = DEFAULT_WEIGHT;
        particles.push_back(particle);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  
    bool going_straight = fabs(yaw_rate) < 0.001;

    // Normal Distributions (Sensor Noise)
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    for (int i=0; i<num_particles;i++) {
        double xf, yf, thetaf = 0.0;
        if (going_straight) {
            xf = velocity * delta_t * cos(particles[i].theta);
            yf = velocity * delta_t * sin(particles[i].theta);
        } else {
            xf = velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
            yf = velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
            thetaf = yaw_rate * delta_t;
        }
        
        particles[i].x += (xf + dist_x(gen));
        particles[i].y += (yf + dist_y(gen));
        particles[i].theta += dist_theta(gen);
        
        if (thetaf != 0.0) {
            particles[i].theta += thetaf;
        }
    }
 
 
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for (int i=0; i<observations.size(); i++ ) {
        LandmarkObs observation = observations[i];
        double minimal_distance = numeric_limits<double>::max(); // init with max double
        int matched_prediction_id = -1;
        
        for (int j=0; j<predicted.size(); j++) {
            LandmarkObs prediction = predicted[j];
            
            // Calculate distance between observation and prediction
            double distance = dist(observation.x, observation.y, prediction.x, prediction.y);
            
            // Find predicted landmark closest to the observed landmark
            if (distance < minimal_distance) {
                minimal_distance = distance;
                matched_prediction_id = prediction.id;
            }
        }
        
        // Set predicted neighbour id to observation id
        observations[i].id = matched_prediction_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
    
    
    for (int i=0; i<num_particles; i++) {
        
        double particle_x = particles[i].x;
        double particle_y = particles[i].y;
        double particle_theta = particles[i].theta;
        

        // Transforms observations into map coordinates
        
        vector<LandmarkObs> transformed_observations;
        
        for (int j=0; j<observations.size(); j++) {
            
            double observation_x = observations[j].x;
            double observation_y = observations[j].y;
            
            double transformed_x = particle_x + observation_x * cos(particle_theta) - observation_y * sin(particle_theta);
            double transformed_y = particle_y + observation_y * cos(particle_theta) + observation_x * sin(particle_theta);
            
            LandmarkObs transformed_observation;
            transformed_observation.id = observations[j].id;
            transformed_observation.x = transformed_x;
            transformed_observation.y = transformed_y;
            
            transformed_observations.push_back(transformed_observation);
        }
    
        
        // Removes landmarks that are not within sensor rangeb
        
        vector<LandmarkObs> map_landmarks_in_range;
        
        for (unsigned int j=0; j<map_landmarks.landmark_list.size(); j++) {

            float landmark_x = map_landmarks.landmark_list[j].x_f;
            float landmark_y = map_landmarks.landmark_list[j].y_f;

            double delta_x = landmark_x - particle_x;
            double delta_y = landmark_y - particle_y;
            double error = sqrt(delta_x * delta_x + delta_y * delta_y);
            
            if (error < sensor_range) {
                LandmarkObs map_landmark_in_range;
                map_landmark_in_range.id = map_landmarks.landmark_list[j].id_i;
                map_landmark_in_range.x = landmark_x;
                map_landmark_in_range.y = landmark_y;
                map_landmarks_in_range.push_back(map_landmark_in_range);
            }
        }
        
        
        // Matches landmarks (in sensor range) to observations
        
        dataAssociation(map_landmarks_in_range, transformed_observations);
        
        
        // Compare observations by vehicle to corresponding observations by particle + update weights
        double weight = DEFAULT_WEIGHT;
        
        for (int j=0; j<transformed_observations.size(); j++) {
            double observation_x = transformed_observations[j].x;
            double observation_y = transformed_observations[j].y;
            double prediction_x = 0.0;
            double prediction_y = 0.0;
            
            // Get prediction for current observation
            for (int x=0; x<map_landmarks_in_range.size(); x++) {
                if (map_landmarks_in_range[x].id == transformed_observations[j].id) {
                    prediction_x = map_landmarks_in_range[x].x;
                    prediction_y = map_landmarks_in_range[x].y;
                }
            }
            
            // Weight calculation
            double fraction = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1]));
            double expo = exp( -(pow(prediction_x - observation_x, 2) / (2 * pow(std_landmark[0], 2)) + (pow(prediction_y - observation_y, 2) / (2 * pow(std_landmark[1], 2)))));
            weight *= fraction * expo;
        }
        
        particles[i].weight = weight;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    vector<Particle> new_particles;
    
    // Collect all weights
    vector<double> all_weights;
    for (int i = 0; i < num_particles; i++) {
        all_weights.push_back(particles[i].weight);
    }
    
    // Generate starting index (resampling wheel)
    uniform_int_distribution<int> dist_index(0, num_particles-1);
    int index = dist_index(gen);
    
    // Get max weight
    double max_weight = *max_element(all_weights.begin(), all_weights.end());
    
    // Random distribution for weights
    uniform_real_distribution<double> dist_weight(0.0, max_weight);
    
    double beta = 0.0;
    
    // Resample wheel algorithm
    for (int i = 0; i < num_particles; i++) {
        beta += dist_weight(gen) * 2.0;
        while (beta > all_weights[index]) {
            beta -= all_weights[index];
            index = (index + 1) % num_particles;
        }
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
