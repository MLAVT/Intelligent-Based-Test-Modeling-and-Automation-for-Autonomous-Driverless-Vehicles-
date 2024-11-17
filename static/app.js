let map;
let directionsService;
let directionsRenderer;
let waypoints = [];
let markers = [];
let infoWindows = {};
let geojsonData = null;

async function initMap() {
  const sanJoseCenter = { lat: 37.3382, lng: -121.8863 };
  map = new google.maps.Map(document.getElementById("map"), {
    zoom: 13,
    center: sanJoseCenter
  });

  directionsService = new google.maps.DirectionsService();
  directionsRenderer = new google.maps.DirectionsRenderer();
  directionsRenderer.setMap(map);

  new google.maps.places.Autocomplete(document.getElementById("origin"));
  new google.maps.places.Autocomplete(document.getElementById("destination"));

  // Load GeoJSON data
  await loadGeoJsonData();

  // Right-click to add a custom checkpoint
  map.addListener("rightclick", (event) => {
    addCheckpoint(event.latLng);
  });

  document.getElementById("delete-trip").addEventListener("click", deleteTrip);
}

function loginUser(event) {
  event.preventDefault();
  const email = document.getElementById('email').value;
  const password = document.getElementById('password').value;
  console.log("Login attempt with:", email, password);
  // Implement your login logic here, typically an API request
}

function showRegistrationForm() {
  document.getElementById('registration-form').style.display = 'block';
}

function hideRegistrationForm() {
  document.getElementById('registration-form').style.display = 'none';
}

function registerUser(event) {
  event.preventDefault();
  const email = document.getElementById('reg-email').value;
  const password = document.getElementById('reg-password').value;
  console.log("Registration attempt with:", email, password);
  // Implement your registration logic here
}

async function loadGeoJsonData() {
  try {
    const response = await fetch('/static/Street_Intersections.geojson');
    geojsonData = await response.json();
    console.log("GeoJSON data loaded:", geojsonData); // Debugging line to check GeoJSON data structure
  } catch (error) {
    console.error("Failed to load GeoJSON data:", error);
  }
}

function calculateRoute() {
  const origin = document.getElementById("origin").value;
  const destination = document.getElementById("destination").value;

  if (!origin || !destination) {
    alert("Please enter both origin and destination addresses.");
    return;
  }

  directionsService.route(
    {
      origin: origin,
      destination: destination,
      travelMode: google.maps.TravelMode.DRIVING,
      waypoints: waypoints,
      optimizeWaypoints: true,
    },
    (result, status) => {
      if (status === google.maps.DirectionsStatus.OK) {
        directionsRenderer.setDirections(result);

        const route = result.routes[0].legs[0];
        document.getElementById("origin-address").innerText = `Origin address: ${route.start_address}`;
        document.getElementById("destination-address").innerText = `Destination address: ${route.end_address}`;
        document.getElementById("distance").innerText = `Distance: ${route.distance.text}`;

        clearMarkers();
        addCheckpointsAlongRoute(result.routes[0]);
      } else {
        console.error("Directions request failed due to:", status);
        alert("Could not display directions due to: " + status);
      }
    }
  );
}


function addCheckpoint(location) {
  const checkpointId = Date.now(); // Replace with DB ID if available

  const marker = new google.maps.Marker({
    position: location,
    map: map,
    title: "Custom Checkpoint",
  });

  // Attach the generated ID to the marker
  marker.checkpointId = checkpointId;

  // Redirect to the checkpoint details page on click
  marker.addListener('click', function () {
    window.location.href = `/checkpoints_details?id=${checkpointId}`;
  });

  markers.push(marker);
  waypoints.push({ location: location, stopover: true });

  calculateRoute();
}





function addCheckpointsAlongRoute(route) {
  const checkpointInterval = route.legs[0].distance.value / 5;
  let currentDistance = 0;
  let checkpointIndex = 0;

  route.legs[0].steps.forEach(step => {
    const stepDistance = step.distance.value;
    while (currentDistance + stepDistance >= checkpointInterval * checkpointIndex) {
      const progress = (checkpointInterval * checkpointIndex - currentDistance) / stepDistance;
      const checkpointLatLng = google.maps.geometry.spherical.interpolate(step.start_location, step.end_location, progress);
      const marker = new google.maps.Marker({
        position: checkpointLatLng,
        map: map,
        title: `Checkpoint ${checkpointIndex + 1}`,
      });

      marker.addListener('click', function() {
        displayCheckpointDetails(marker, checkpointLatLng);
      });

      markers.push(marker);
      checkpointIndex++;
    }

    currentDistance += stepDistance;
  });
}



function deleteTrip() {
  directionsRenderer.setDirections({ routes: [] });
  waypoints = [];
  clearMarkers();
  document.getElementById("origin-address").innerText = "Origin address: ";
  document.getElementById("destination-address").innerText = "Destination address: ";
  document.getElementById("distance").innerText = "Distance: ";
}

function clearMarkers() {
  markers.forEach(marker => marker.setMap(null));
  markers = [];
}

function findNearestIntersection(location) {
  if (!geojsonData) return { type: "Unknown", streets: "Unknown" };

  let nearestIntersection = null;
  let minDistance = Infinity;

  geojsonData.features.forEach(feature => {
    const coordinates = feature.geometry.coordinates;
    const intersectionLocation = new google.maps.LatLng(coordinates[1], coordinates[0]);
    const distance = google.maps.geometry.spherical.computeDistanceBetween(location, intersectionLocation);

    if (distance < minDistance) {
      minDistance = distance;
      nearestIntersection = feature;
    }
  });

  if (nearestIntersection) {
    const props = nearestIntersection.properties;
    const streets = `${props.ASTREETNAME} & ${props.BSTREETNAME}`;
    const intersectionType = props.INTERSECTIONTYPE || "Unknown";
    const longitude = props.LONGITUDE;
    const latitude = props.LATITUDE;
    
    console.log("Found nearest intersection:", { type: intersectionType, streets, longitude, latitude });
    return {
      type: intersectionType,
      streets: streets,
      longitude: longitude,
      latitude: latitude
    };
  }

  return { type: "Unknown", streets: "Unknown", longitude: 0, latitude: 0 };
}

function displayCheckpointDetails(marker, location) {
  const geocoder = new google.maps.Geocoder();
  geocoder.geocode({ location: location }, (results, status) => {
      if (status === google.maps.GeocoderStatus.OK && results[0]) {
          const address = results[0].formatted_address;
          const intersectionData = findNearestIntersection(location);

          // Update the street view image dynamically
          document.getElementById("checkpoint-location").innerText = `Location address: ${address}`;
          document.getElementById("intersection-type").innerText = `Intersection Type: ${intersectionData.type}`;
          document.getElementById("intersection-street").innerText = `Intersection Streets: ${intersectionData.streets}`;
          document.getElementById("longitude").innerText = `Longitude: ${intersectionData.longitude}`;
          document.getElementById("latitude").innerText = `Latitude: ${intersectionData.latitude}`;
          document.getElementById("status").innerText = "Status: Test Case Retrieved";
          document.getElementById("checkpoint-details").style.display = "block";
          
          // Dynamically set the Street View image source
          document.getElementById("street-view").src = `https://maps.googleapis.com/maps/api/streetview?size=500x300&location=${location.lat()},${location.lng()}&key=AIzaSyDjW2z9CLWbEPphBu_kMGNz8hIoy_Ezw9Q`;
          

      } else {
          console.error("Geocoder failed due to:", status);
      }
  });
}



document.addEventListener('DOMContentLoaded', function () {
  const viewTestCasesBtn = document.getElementById('viewTestCasesBtn');
  if (viewTestCasesBtn) {
    viewTestCasesBtn.addEventListener('click', viewTestCases);
  }
});

function closeCheckpointDetails() {
  document.getElementById("checkpoint-details").style.display = "none";
}

function deleteCheckpoint(lat, lng) {
  // Close the details view
  closeCheckpointDetails();

  // Filter out the marker to be deleted
  markers = markers.filter(marker => {
    const position = marker.getPosition();
    if (position.lat().toFixed(6) === lat.toFixed(6) && position.lng().toFixed(6) === lng.toFixed(6)) {
      marker.setMap(null);
      return false;
    }
    return true;
  });

  // Also remove the corresponding waypoint
  waypoints = waypoints.filter(wp => {
    const position = wp.location;
    return !(position.lat.toFixed(6) === lat.toFixed(6) && position.lng.toFixed(6) === lng.toFixed(6));
  });

  // Optionally: Update the route if necessary
  if (waypoints.length > 0) {
    calculateRoute();
  }
}

function viewTestCase(testCaseId) {
    console.log(testCaseId); // Just a test to ensure the value is passed correctly.
}

