from datetime import datetime
from app import db  # Import db from app.py

class Trip(db.Model):
    __tablename__ = 'trips'
    
    id = db.Column(db.Integer, primary_key=True, autoincrement=True)
    name = db.Column(db.String(255), nullable=False)
    origin = db.Column(db.String(255), nullable=False)
    destination = db.Column(db.String(255), nullable=False)
    distance = db.Column(db.Float, nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)


class Checkpoint(db.Model):
    __tablename__ = 'checkpoints'

    id = db.Column(db.Integer, primary_key=True)
    latitude = db.Column(db.Float)
    longitude = db.Column(db.Float)
    location_address = db.Column(db.String(255))
    intersection_type = db.Column(db.String(100))
    intersection_street = db.Column(db.String(255))
    trip_id = db.Column(db.Integer, db.ForeignKey('trips.id'), nullable=False)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)

    object_detection_status = db.Column(db.String(50))
    ai_testing_tool_status = db.Column(db.String(50))
    generate_test_cases_status = db.Column(db.String(50))
    retrieve_test_cases_status = db.Column(db.String(50))
    context_tree_url = db.Column(db.String(255))
    input_tree_url = db.Column(db.String(255))
    output_tree_url = db.Column(db.String(255))

    test_cases = db.relationship('TestCase', backref='checkpoint', lazy=True)


class TestCase(db.Model):
    __tablename__ = 'test_cases'

    id = db.Column(db.Integer, primary_key=True)
    subkey = db.Column(db.String(50))
    script_url = db.Column(db.String(255))
    xml_url = db.Column(db.String(255))
    checkpoint_id = db.Column(db.Integer, db.ForeignKey('checkpoints.id'), nullable=False)
