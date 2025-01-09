#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <mutex>
#include <cmath>
#include <limits>
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>
#include <functional>

std::mutex mtx; // Mutex for thread safety

// Exception for GPS signal loss
class GPSSignalException : public std::exception {
public:
  const char* what() const noexcept override {
    return "GPS signal lost or invalid data!";
  }
};

// Subject class for Observer Pattern
class Subject {
  std::vector<std::function<void(const std::string&)>> observers;
public:
  void addObserver(const std::function<void(const std::string&)>& observer) {
    observers.push_back(observer);
  }

  void notifyObservers(const std::string& message) const {
    for (const auto& observer : observers) {
      observer(message);
    }
  }
};

// Strategy interface for route calculation
class RouteStrategy {
public:
  virtual void calculateRoute(const std::vector<std::pair<double, double>>& waypoints, const std::string& vehicleName) = 0;
  virtual ~RouteStrategy() = default;
};

// Dijkstra's strategy for route calculation
class DijkstraRouteStrategy : public RouteStrategy {
public:
  void calculateRoute(const std::vector<std::pair<double, double>>& waypoints, const std::string& vehicleName) override {
    std::cout << "Calculating shortest route using Dijkstra's algorithm... for " << vehicleName << "\n";
    size_t n = waypoints.size();

    std::vector<std::vector<double>> graph(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
    for (size_t i = 0; i < n - 1; ++i) {
      double dist = std::sqrt(std::pow(waypoints[i + 1].first - waypoints[i].first, 2) +
        std::pow(waypoints[i + 1].second - waypoints[i].second, 2));
      graph[i][i + 1] = dist;
      graph[i + 1][i] = dist;
    }

    std::vector<double> minDist(n, std::numeric_limits<double>::infinity());
    std::vector<int> previous(n, -1);
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    minDist[0] = 0.0;
    pq.push({ 0.0, 0 });

    while (!pq.empty()) {
      auto[dist, u] = pq.top();
      pq.pop();

      if (dist > minDist[u]) continue;

      for (size_t v = 0; v < n; ++v) {
        if (graph[u][v] < std::numeric_limits<double>::infinity()) {
          double newDist = minDist[u] + graph[u][v];
          if (newDist < minDist[v]) {
            minDist[v] = newDist;
            previous[v] = u;
            pq.push({ newDist, v });
          }
        }
      }
    }

    std::vector<std::pair<double, double>> optimizedPath;
    for (int v = n - 1; v != -1; v = previous[v]) {
      optimizedPath.push_back(waypoints[v]);
    }
    std::reverse(optimizedPath.begin(), optimizedPath.end());

    std::lock_guard<std::mutex> lock(mtx);
    std::cout << "Optimized Path for " << vehicleName << ":\n";
    for (const auto& point : optimizedPath) {
      std::cout << "(" << point.first << ", " << point.second << ")\n";
    }
  }
};

// Template class for managing navigation routes
template <typename T>
class NavigationRoutes {
  std::vector<T> routes;

public:
  void addRoute(const T& route) {
    routes.push_back(route);
  }

  const std::vector<T>& getRoutes() const {
    return routes;
  }

  void clearRoutes() {
    routes.clear();
  }
};

// Base Car Class with Strategy Pattern and Observer Pattern
class Car : public Subject {
protected:
  std::string vehicleType;
  std::string name;
  double latitude;
  double longitude;
  double speed;
  std::pair<double, double> destination;
  std::shared_ptr<RouteStrategy> routeStrategy;
  bool gpsOn;  // Flag to toggle GPS signal

public:
  Car(std::string type, std::string vehicleName, double lat, double lon, double spd, std::shared_ptr<RouteStrategy> strategy)
    : vehicleType(type), name(vehicleName), latitude(lat), longitude(lon), speed(spd), routeStrategy(strategy), gpsOn(true) {}

  virtual ~Car() = default;

  // Getter and Setter methods
  std::string getVehicleType() const { return vehicleType; }
  std::string getName() const { return name; }
  double getLatitude() const { return latitude; }
  double getLongitude() const { return longitude; }
  double getSpeed() const { return speed; }
  std::pair<double, double> getDestination() const { return destination; }
  bool isGPSOn() const { return gpsOn; }

  void setLatitude(double lat) { latitude = lat; }
  void setLongitude(double lon) { longitude = lon; }
  void setSpeed(double spd) { speed = spd; }
  void setDestination(double destLat, double destLon) { destination = { destLat, destLon }; }
  void toggleGPS(bool state) { gpsOn = state; }

  std::pair<double, double> getCurrentLocation() const {
    return { latitude, longitude };
  }

  virtual void calculateRoute(const std::vector<std::pair<double, double>>& waypoints) {
    routeStrategy->calculateRoute(waypoints, name);
  }

  void updateLocation(double newLat, double newLon) {
    latitude = newLat;
    longitude = newLon;
  }

  bool isNearDestination() const {
    double distance = std::sqrt(std::pow(latitude - destination.first, 2) + std::pow(longitude - destination.second, 2));
    return distance < 0.01;
  }

  bool isOffRoute(const std::vector<std::pair<double, double>>& waypoints) const {
    for (const auto& point : waypoints) {
      double distance = std::sqrt(std::pow(latitude - point.first, 2) + std::pow(longitude - point.second, 2));
      if (distance < 0.01) return false;
    }
    return true;
  }
};

// Derived ElectricCar class that uses DijkstraRouteStrategy

class ElectricCar : public Car {
public:
  ElectricCar(std::string vehicleName, double lat, double lon, double spd, std::shared_ptr<RouteStrategy> strategy)
    : Car("ElectricCar", vehicleName, lat, lon, spd, strategy) {}

  void calculateRoute(const std::vector<std::pair<double, double>>& waypoints) override {
    std::cout << "ElectricCar " << name << " is calculating route using Dijkstra's algorithm...\n";
    routeStrategy->calculateRoute(waypoints, name);
  }
};

// Derived SUV class that uses DijkstraRouteStrategy
class SUV : public Car {
public:
  SUV(std::string vehicleName, double lat, double lon, double spd, std::shared_ptr<RouteStrategy> strategy)
    : Car("SUV", vehicleName, lat, lon, spd, strategy) {}

  void calculateRoute(const std::vector<std::pair<double, double>>& waypoints) override {
    std::cout << "SUV " << name << " is calculating route using Dijkstra's algorithm...\n";
    routeStrategy->calculateRoute(waypoints, name);
  }
};

// Function to simulate vehicle tracking with manual control of GPS state
void trackVehicle(Car* vehicle, const NavigationRoutes<std::pair<double, double>>& navigationRoutes) {
  try {
    vehicle->calculateRoute(navigationRoutes.getRoutes());
    const auto& route = navigationRoutes.getRoutes();
    for (size_t i = 0; i < route.size(); ++i) {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Allow the user to toggle GPS state periodically
      if (i % 5 == 0) {
        std::lock_guard<std::mutex> lock(mtx);
        std::cout << "Enter 'off' to turn off GPS or 'on' to turn on GPS of " << vehicle->getName() << " (Current state: "
          << (vehicle->isGPSOn() ? "on" : "off") << "): ";
        std::string input;
        std::cin >> input;
        if (input == "off") {
          vehicle->toggleGPS(false);
          std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): GPS is now OFF\n";
        }
        else if (input == "on") {
          vehicle->toggleGPS(true);
          std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): GPS is now ON\n";
        }
      }

      // Simulate GPS error if GPS is turned off
      if (!vehicle->isGPSOn()) {
        throw GPSSignalException();
      }

      // Update vehicle's location
      vehicle->updateLocation(route[i].first, route[i].second);

      // Manually check if the vehicle is off-route every 3rd iteration
      if (i % 3 == 0) {
        if (vehicle->isOffRoute(route)) {
          std::lock_guard<std::mutex> lock(mtx);
          std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): is off-route!\n";
          vehicle->notifyObservers("Vehicle is off-route");
        }
        else {
          std::lock_guard<std::mutex> lock(mtx);
          std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): is on-route.\n";
        }
      }

      // Output current location
      std::lock_guard<std::mutex> lock(mtx);
      std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): tracked location: ("
        << vehicle->getCurrentLocation().first << ", " << vehicle->getCurrentLocation().second << ")\n";

      // Check if the vehicle has reached the destination
      if (vehicle->isNearDestination()) {
        std::cout << vehicle->getName() << " (" << vehicle->getVehicleType() << "): reached destination!\n";
        vehicle->notifyObservers("Reached destination");
        return;
      }
    }
  }
  catch (const GPSSignalException& e) {
    // Handle GPS signal loss exception
    std::lock_guard<std::mutex> lock(mtx);
    std::cerr << "Error: " << e.what() << " (" << vehicle->getName() << " " << vehicle->getVehicleType() << ")\n";
    vehicle->notifyObservers("GPS signal lost");
  }
}

int main() {
  std::shared_ptr<RouteStrategy> dijkstraStrategy = std::make_shared<DijkstraRouteStrategy>();

  ElectricCar tesla("Tesla", 12.9716, 77.5946, 60.0, dijkstraStrategy);
  SUV jeep("Jeep", 12.9716, 77.5946, 50.0, dijkstraStrategy);

  tesla.setDestination(12.9352, 77.6245);
  jeep.setDestination(12.9250, 77.5938);

  NavigationRoutes<std::pair<double, double>> navigationRoutes;
  navigationRoutes.addRoute({ 12.9716, 77.5946 });
  navigationRoutes.addRoute({ 12.9616, 77.6046 });
  navigationRoutes.addRoute({ 12.9516, 77.6146 });
  navigationRoutes.addRoute({ 12.9352, 77.6245 });

  tesla.addObserver([](const std::string& message) { std::cout << "Tesla: " << message << "\n"; });
  jeep.addObserver([](const std::string& message) { std::cout << "Jeep: " << message << "\n"; });

  std::thread t1(trackVehicle, &tesla, std::ref(navigationRoutes));
  std::thread t2(trackVehicle, &jeep, std::ref(navigationRoutes));

  t1.join();
  t2.join();

  return 0;
}
