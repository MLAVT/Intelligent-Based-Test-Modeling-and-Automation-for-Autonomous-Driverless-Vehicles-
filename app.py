from flask import Flask, request, jsonify, render_template, abort, redirect, url_for, send_from_directory
from flask_sqlalchemy import SQLAlchemy
from flask_migrate import Migrate
from werkzeug.utils import secure_filename
import os

# Create the Flask app
app = Flask(__name__)

# Set the configuration for SQLAlchemy (Database URI and other configurations)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///trips.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['UPLOAD_FOLDER'] = 'uploads'
app.config['STATIC_FOLDER'] = 'static'  # This line is for convenience in serving files from static folder

# Initialize SQLAlchemy with the app
db = SQLAlchemy(app)

# Initialize migration support with Flask-Migrate
migrate = Migrate(app, db)

# Import the models after initializing db
from models import Trip, Checkpoint, TestCase  # db is already initialized in app.py

@app.route('/')
def home():
    checkpoint = Checkpoint.query.first()  # Adjust this query based on your requirements
    return render_template('index.html', checkpoint=checkpoint)

@app.route('/static/<filename>')
def download_file(filename):
    return send_from_directory('static', filename, as_attachment=True)

@app.route('/checkpoints_details', methods=['GET'])
def checkpoints_details():
    checkpoint_id = request.args.get('id', type=int)
    checkpoint_data = Checkpoint.query.get(checkpoint_id)
    test_cases = TestCase.query.filter_by(checkpoint_id=checkpoint_id).limit(10).all()
    return render_template('checkpoints_details.html', checkpoint=checkpoint_data, testcases=test_cases)

@app.route('/upload/<string:file_type>', methods=['POST'])
def upload_file(file_type):
    if file_type not in ['context', 'input', 'output']:
        abort(400, description="Invalid file type")

    if 'file' not in request.files:
        abort(400, description="No file part")
    
    file = request.files['file']
    if file.filename == '':
        abort(400, description="No selected file")

    filename = secure_filename(file.filename)
    file.save(os.path.join(app.config['UPLOAD_FOLDER'], filename))
    
    return jsonify({"message": f"{file_type} file uploaded successfully", "filename": filename})

@app.route('/trips', methods=['POST'])
def create_trip():
    data = request.json
    trip = Trip(
        name=data.get('name'),
        origin=data.get('origin'),
        destination=data.get('destination'),
        distance=data.get('distance')
    )
    db.session.add(trip)
    db.session.commit()
    return jsonify({"message": "Trip created", "trip_id": trip.id}), 201

@app.route('/trips/<int:trip_id>/checkpoints', methods=['POST'])
def add_checkpoint_to_trip(trip_id):
    trip = Trip.query.get(trip_id)
    if not trip:
        return jsonify({"error": "Trip not found"}), 404

    data = request.json
    checkpoint = Checkpoint(
        latitude=data.get('latitude'),
        longitude=data.get('longitude'),
        location_address=data.get('location_address'),
        intersection_type=data.get('intersection_type'),
        intersection_street=data.get('intersection_street'),
        trip_id=trip_id
    )
    db.session.add(checkpoint)
    db.session.commit()
    return jsonify({"message": "Checkpoint added to trip", "checkpoint_id": checkpoint.id}), 201

@app.route('/trips/<int:trip_id>', methods=['GET'])
def get_trip(trip_id):
    trip = Trip.query.get(trip_id)
    if not trip:
        return jsonify({"error": "Trip not found"}), 404

    trip_data = {
        "id": trip.id,
        "name": trip.name,
        "origin": trip.origin,
        "destination": trip.destination,
        "distance": trip.distance,
        "created_at": trip.created_at,
        "checkpoints": [
            {
                "id": checkpoint.id,
                "latitude": checkpoint.latitude,
                "longitude": checkpoint.longitude,
                "location_address": checkpoint.location_address,
                "intersection_type": checkpoint.intersection_type,
                "intersection_street": checkpoint.intersection_street,
                "created_at": checkpoint.created_at
            }
            for checkpoint in trip.checkpoints
        ]
    }
    return jsonify(trip_data), 200

if __name__ == "__main__":
    app.run(debug=True)
