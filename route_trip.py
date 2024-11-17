from flask import request, render_template, Blueprint, redirect, abort

from flask_login import current_user
import subprocess

from ..extensions import db

from ..models.checkpoint import Checkpoint, CheckpointStatus, IntersectionType
from ..models.trip import Trip

import time, os, json, requests, re
import numpy as np
from PIL import Image
from enum import Enum
from ultralytics import YOLO

USERNAME = "DATA298B"
PASSWORD = "demo"

AI_TESTING_TOOL_API_URL = 'http://3.14.249.198:8080'
AI_TESTING_TOOL_LOGIN_URI = 'users/authenticate'


NEW_FILE_PATH = "/usr/src/app/Trip/static/data"
ALLOWED_EXTENSIONS = [".jpg", ".png", ".jpeg"]

checkpoint_api = Blueprint( 'checkpoint_api', __name__ )

@checkpoint_api.before_request
def before_request():

    endpoints = ['checkpoint_api.get_checkpoint', 
                 'checkpoint_api.delete_checkpoint', 
                 'checkpoint_api.object_detection', 
                 'checkpoint_api.send_data',
                 'checkpoint_api.generate_test_case',
                 'checkpoint_api.retrieve_test_case']

    if request.endpoint and request.endpoint in endpoints:

        checkpoint = Checkpoint.query.get( request.path.split('/')[2] )

        if not checkpoint or checkpoint.trip.user != current_user:

            abort(404)

class DirectionalPerspective(Enum):
    ADD_X = "ADD_X"
    SUBTRACT_X = "SUBTRACT_X"
    ADD_Y = "ADD_Y"
    SUBTRACT_Y = "SUBTRACT_Y"


waypoint_location = {
    "Town10HD": {
        IntersectionType.tp3way.value: {
            "location": {"x":85.5, "y":70.5, "z":0.3}, 
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},             
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        IntersectionType.tp4way.value: {
            "location": {"x":-49, "y":-10, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },           
        },
        IntersectionType.tp5way.value: {
            "location": {"x":-60, "y":140, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "yield-controlled intersection": {
            "location": {"x":85.5, "y":70.5, "z":0.3}, 
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "stop-controlled intersection": {
            "location": {"x":-48, "y":-115, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "signal-controlled intersection": {
            "location": {"x":-82, "y":16, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "traffic circle": {
            "location": {"x":-60, "y":140, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        IntersectionType.tp2way.value: {
            "location": {"x":102, "y":70, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "one way road": {
            "location": {"x":-7, "y":24, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "parking lot": {
            "location": {"x":-58, "y":186, "z":0.3},
            "rotation": {"pitch":0.0, "roll":0.0, "yaw":90.0},
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            }
        }
    }
}

def generate_directional_reference_coordinate_x(key, x_distance_away, intersection_type, town):
    x_coord_of_obj = f"ego_vehicle_location.x + {x_distance_away}"
    if "Right" in key:
        x_coord_of_obj = f"ego_vehicle_location.x - {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"left\"] == DirectionalPerspective.ADD_X.value else ego_vehicle_location.y + {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"left\"] == DirectionalPerspective.ADD_Y.value else ego_vehicle_location.x + {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"left\"] == DirectionalPerspective.SUBTRACT_X.value else ego_vehicle_location.y - {x_distance_away}"
    elif "Left" in key:
        x_coord_of_obj = f"ego_vehicle_location.x + {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"right\"] == DirectionalPerspective.ADD_X.value else ego_vehicle_location.y + {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"right\"] == DirectionalPerspective.ADD_Y.value else ego_vehicle_location.x - {x_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"right\"] == DirectionalPerspective.SUBTRACT_X.value else ego_vehicle_location.y - {x_distance_away}"
    return x_coord_of_obj

def generate_directional_reference_coordinate_y(key, y_distance_away, intersection_type, town):
    y_coord_of_obj = f"ego_vehicle_location.y + {y_distance_away}"
    if key is not None and "Forward" in key:
        y_coord_of_obj = f"ego_vehicle_location.x + {y_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"front\"] == DirectionalPerspective.ADD_X.value else ego_vehicle_location.y + {y_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"front\"] == DirectionalPerspective.ADD_Y.value else ego_vehicle_location.x - {y_distance_away} if waypoint_location[\"{town}\"][\"{intersection_type}\"][\"perspective\"][\"front\"] == DirectionalPerspective.SUBTRACT_X.value else ego_vehicle_location.y - {y_distance_away}"
    return y_coord_of_obj

def carla_code_block_create_agent(x_coord, y_coord, z_coord, pitch, roll, yaw, agent_name='vehicle.nissan.patrol', set_simulate_physics=False):
    assert x_coord is not None
    assert y_coord is not None
    assert z_coord is not None
    assert yaw is not None
    assert agent_name is not None
    return f"transform = carla.Transform(carla.Location(x={x_coord}, y={y_coord}, z=round({z_coord})), carla.Rotation({pitch},{yaw},{roll})){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}actor = CarlaDataProvider.request_new_actor('{agent_name}', transform){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}if actor is not None:{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{CARLA_SCRIPT_CODE_SEPARATOR}actor.set_simulate_physics(enabled={set_simulate_physics}){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{CARLA_SCRIPT_CODE_SEPARATOR}self._actor_transforms[str({x_coord}) + '_' + str({y_coord}) + '_' + str({z_coord})] = transform{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{CARLA_SCRIPT_CODE_SEPARATOR}self._other_actors[str({x_coord}) + '_' + str({y_coord}) + '_' + str({z_coord})] = actor"

def carla_code_block_create_parallel_behavior():
    return f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}driving_to_next_intersection = py_trees.composites.Parallel(\"Driving towards Intersection\",policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)"

def carla_code_block_create_ego_vehicle_behavior():
    return f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}driving_to_next_intersection.add_child(WaypointFollower(self.ego_vehicles[0])){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}endcondition.add_child(StandStill(self.ego_vehicles[0], name=\"FinalSpeed\", duration=1)){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}endcondition.add_child(DriveDistance(self.ego_vehicles[0], 1))"

def carla_code_block_create_vehicle_behavior(x_coord, y_coord, z_coord):
    assert x_coord is not None
    assert y_coord is not None
    assert z_coord is not None
    z_coord = round(z_coord)
    return f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}driving_to_next_intersection.add_child(WaypointFollower(self._other_actors[str({x_coord}) + '_' + str({y_coord}) + '_' + str({z_coord})], self._first_actor_speed)){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}sequence.add_child(ActorTransformSetter(self._other_actors[str({x_coord}) + '_' + str({y_coord}) + '_' + str({z_coord})], self._actor_transforms[str({x_coord}) + '_' + str({y_coord}) + '_' + str({z_coord})]))"

def carla_code_block_set_weather(in_cloudiness=0.0, in_precipitation=0.0, in_precipitation_deposits=0.0, in_wind_intensity=0.0, in_sun_azimuth_angle=0.0, in_sun_altitude_angle=0.0, in_fog_density=0.0, in_fog_distance=0.0, in_fog_falloff=0.0, in_wetness=0.0, in_scattering_intensity=0.0, in_mie_scattering_scale=0.0, in_rayleigh_scattering_scale=0.0, in_dust_storm=0.0):
    return f"weather = client.WeatherParameters(in_cloudiness={in_cloudiness}, in_precipitation={in_precipitation}, in_precipitation_deposits={in_precipitation_deposits}, in_wind_intensity={in_wind_intensity}, in_sun_azimuth_angle={in_sun_azimuth_angle}, in_sun_altitude_angle={in_sun_altitude_angle}, in_fog_density={in_fog_density}, in_fog_distance={in_fog_distance}, in_fog_falloff={in_fog_falloff}, in_wetness={in_wetness}, in_scattering_intensity={in_scattering_intensity}, in_mie_scattering_scale={in_mie_scattering_scale}, in_rayleigh_scattering_scale={in_rayleigh_scattering_scale}, in_dust_storm={in_dust_storm}){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}world.set_weather(weather){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}print(world.get_weather())"

@checkpoint_api.route('<int:checkpoint_id>', methods=['GET'])
def get_checkpoint( checkpoint_id ):

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        CHECKPOINT_STATUS_DICT = {

            CheckpointStatus.CREATED : 1,
            CheckpointStatus.OBJECT_DETECTED : 2,
            CheckpointStatus.DATA_SENT : 3,
            CheckpointStatus.TEST_CASES_GENERATED : 4,
            CheckpointStatus.TEST_CASES_RETRIEVED : 5

        }
        
        context = load_tree( checkpoint_id, "context" )
        input = load_tree( checkpoint_id, "input" )
        output = load_tree( checkpoint_id, "output" )

        testcases = load_testcases( checkpoint_id )

        return render_template('view_checkpoint.html' , checkpoint = checkpoint, 
                                                        CheckpointStatus = CheckpointStatus, 
                                                        status = CHECKPOINT_STATUS_DICT.get( checkpoint.status ),
                                                        testcases = testcases,
                                                        context = context, input = input, output = output )
    
    else:

        abort(404)

@checkpoint_api.route('/add/trip/<int:trip_id>', methods=['POST'])
def start_process( trip_id ):

    checkpoint = Checkpoint()

    checkpoint.location_address = request.form.get("location")
    checkpoint.maneuver = request.form.get("maneuver")
    checkpoint.intersection_type = request.form.get("intersectionType")
    checkpoint.image_url = ''

    checkpoint.trip = Trip.query.get( trip_id )

    db.session.add( checkpoint )
    db.session.commit()

    if not os.path.exists(NEW_FILE_PATH):

        os.makedirs(NEW_FILE_PATH)

    if "image" in request.files:

        file = request.files["image"]

        image = Image.open( file )

        if image and allowed_file( file.filename ):

            full_name = os.path.splitext( file.filename )

            file_name = f"{checkpoint.id}{full_name[1]}"

            image_path = os.path.join( NEW_FILE_PATH, file_name )

            image.save( image_path )

            checkpoint.image_url = f'../static/data/{file_name}'

            db.session.commit()

    return render_template('loading.html' , checkpoint = Checkpoint.query.get( checkpoint.id )  )

@checkpoint_api.route('/delete/<int:checkpoint_id>', methods=['GET'])
def delete_checkpoint( checkpoint_id ):

    trip_id = 0

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        filenames = [ f"checkpoint{checkpoint.id}.json" ]

        trip_id = checkpoint.trip.id

        for type in ['png', 'jpg', 'jpeg']:

            filenames.append( f"{checkpoint.id}.{type}"  )
            
        for type in ['context', 'input', 'output', 'payload', 'testcases' ]:

            filenames.append( f"checkpoint{checkpoint.id}_{type}.json"  )

        db.session.delete( checkpoint )
        db.session.commit()

        for filename in filenames:

            file_path = os.path.join( NEW_FILE_PATH, filename)
                
            if os.path.exists( file_path ):

                os.remove( file_path )

    return redirect(f'/trip/{ trip_id }')

@checkpoint_api.route('/<int:checkpoint_id>/detect')
def object_detection( checkpoint_id ):

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        models = ["yolov8n.pt", "yolo_traffic_signs.pt", "yolo_lane_segmentation.pt"]

        name, extension = os.path.splitext( checkpoint.image_url )

        image_path = os.path.join( NEW_FILE_PATH, f'{checkpoint_id}{extension}' )

        image = Image.open( image_path )

        # results = model.predict( source = image, show = False )

        objects = []

        for model in models:

            results = YOLO( model ).predict( source = image, show = False )

            for box in results[0].boxes:

                class_id = int(box.cls.numpy()[0])

                new_name = check_object( results[0].names[class_id] )

                if not new_name: 
                    
                    continue

                xywhn = box.xywhn.numpy()[0]

                box_object = {
                    "class_id": class_id,
                    "name": new_name,
                    "relative_coordinates": {
                        "center_x": float(format(xywhn[0], ".6f")),
                        "center_y": float(format(xywhn[1], ".6f")),
                        "width": float(format(xywhn[2], ".6f")),
                        "height": float(format(xywhn[3], ".6f")),
                    },
                    "confidence": float(format(box.conf.numpy()[0], ".6f")),
                }

                objects.append(box_object)

        data = {"frame_id": 1, "filename": image_path, "objects": objects}
        
        json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}.json' )

        with open( json_path, 'w') as file:

            file.write( json.dumps( data ) )

        checkpoint.status = CheckpointStatus.OBJECT_DETECTED

        db.session.commit()

        time.sleep(1)

    return 'True'

@checkpoint_api.route('/<int:checkpoint_id>/send')
def send_data( checkpoint_id ):

    context = {}
    input = {}
    output = {}
    location = {}

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        f = open( os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}.json' ) )

        json_file = json.load( f )

        direct_lane = None
        alternative_lanes = []

        objects = json_file['objects']

        for obj in objects:

            if obj['name'] == 'direct':

                direct_lane = obj

            elif obj['name'] == 'alternative':
             
             alternative_lanes.append(obj)

        traffic_signs = get_traffic_sign()

        unique_objects = list( set( item['name'] for item in objects ) )

        input['Weather'] = { 'Clear', 'Cloudy', 'Snowy', 'Rainy', 'Foggy' }

        location['Car'] = { 'Front' : { 'Yes', 'No' } }

        if direct_lane and alternative_lanes:

            alternative_lanes_positions = ['left' if lane['relative_coordinates']['center_x'] < direct_lane['relative_coordinates']['center_x'] else 'right' for lane in alternative_lanes]

            if 'left' in alternative_lanes_positions:

                location['Car']['On Left Lane'] = { 'Yes', 'No' }
                                       
            elif 'right' in alternative_lanes_positions:

                location['Car']['On Right Lane'] = { 'Yes', 'No' }

        if 'traffic-light' in unique_objects:

            traffic_light = { 'Green', 'Yellow', 'Red' }

            input['Traffic Light'] = { 'Forward' : traffic_light }

            context['Traffic Light'] = { 'Forward' : { 'Applicable' } }
        
        input['Time'] = { 'Dusk', 'Daytime', 'Dawn', 'Night' }
        
        input['Vehicle'] = location

        if checkpoint.maneuver:
            context['My Car'] = {}

            context['My Car']['Manuever'] = { checkpoint.maneuver }

        context['Checkpoint Type'] = { checkpoint.intersection_type.value }

        for name in unique_objects:
        
            if name in traffic_signs:

                context['Traffic Sign'][ name.replace('-', ' ').title() ] = { 'Applicable' }

        output = {

            "Vehicle alert": ["Object detected", "Not applicable"],
            "Moving state": ["Driving", "Stopped"],

        }

        input_json = dict_to_json(input)
        context_json = dict_to_json(context)
        output_json = dict_to_json(output)

        loginToken = login()

        functionName = f'checkpoint{checkpoint.id}'

        payload = {

            "functionName" : functionName,
            "functionType" : "Image",
            
            "contextFileData" : json.dumps(
                { "name" : "context", "children" : context_json }, separators = ( ",", ":" ), ensure_ascii = False
            ),
            "contextFileName" : f"context_{functionName}.json_context",
            
            "inputFileData" : json.dumps(
                { "name": "input", "children": input_json }, separators = ( ",", ":" ), ensure_ascii = False
            ),
            "inputFileName" : f"input_{functionName}.json_input",
            
            "outputFileData" : json.dumps(
                { "name" : "output", "children" : output_json }, separators = ( ",", ":" ), ensure_ascii = False
            ),
            "outputFileName": f"output_{functionName}.json_output",

        }

        save_tree_json( checkpoint_id, "context", context_json )
        save_tree_json( checkpoint_id, "input", input_json )
        save_tree_json( checkpoint_id, "output", output_json )

        json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_payload.json' )

        with open( json_path, 'w' ) as file:

            file.write( json.dumps( payload, ensure_ascii = False ) )

        AI_TESTING_TOOL_FUNCTION = ( f'djbackend/user/{USERNAME}/projects/trip{checkpoint.trip.id}/functions' )

        res = requests.post(

            url = f'{AI_TESTING_TOOL_API_URL}/{AI_TESTING_TOOL_FUNCTION}',
            data = payload,
            headers = { 'Authorization' : f'Bearer {loginToken}' },
        
        )

        if res.status_code >= 200 and res.status_code < 300:

            checkpoint.status = CheckpointStatus.DATA_SENT

            db.session.commit()

            time.sleep(1)

    return res.text

@checkpoint_api.route('/<int:checkpoint_id>/generate_carla_script_xml/<int:testcase_id>', methods=['GET'])
def generate_carla_script_xml(checkpoint_id, testcase_id):

    # Opening JSON file
    f = open( os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}.json' ) )

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:
        testcases = load_testcases_for_carla_script( checkpoint_id )
        input_dict = testcases[str(testcase_id)]['inputDict']
        # TODO: uncomment and make sure Checkpoint model and waypoint_location above are in sync
        # Maybe create a new model with a foreign key of intersection type and coords being unique
        # intersection_type = waypoint_location[]
        town = "Town10HD"

        intersection_type = waypoint_location[town][checkpoint.intersection_type.value]
        with open('/usr/src/app/project/carla/FollowLeadingVehicle.xml', 'r') as file:
            # Reading the content of the file
            # using the read() function and storing
            # them in a new variable
            data = file.read()
            
            # Searching and replacing the text
            # using the replace() function
            data = data.replace("###town###", town)
            data = data.replace("###ego_vehicle_x###", str(intersection_type['location']['x']))
            data = data.replace("###ego_vehicle_y###", str(intersection_type['location']['y']))
            data = data.replace("###ego_vehicle_z###", str(intersection_type['location']['z']))
            data = data.replace("###ego_vehicle_yaw###", str(intersection_type['rotation']['yaw']))
            data = data.replace("###ego_vehicle_model###", "vehicle.lincoln.mkz_2017")

            for key, value in input_dict.items():
                if "Weather" in key:
                    if "Cloudy" in value:
                        data = data.replace("###cloudiness###", "60.0")
                        data = data.replace("###precipitation###", "0.0")
                        data = data.replace("###precipitation_deposits###", "0.0")
                        data = data.replace("###wind_intensity###", "10.0")
                        data = data.replace("###sun_azimuth_angle###", "-1.0")
                        data = data.replace("###sun_altitude_angle###", "45.0")
                        data = data.replace("###fog_density###", "3.0")
                        data = data.replace("###fog_distance###", "0.75")
                        data = data.replace("###fog_falloff###", "0.1")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "1.0")
                        data = data.replace("###mie_scattering_scale###", "0.03")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0331")
                        data = data.replace("###dust_storm###", "0.0")
                    elif "Snowy" in value:
                        data = data.replace("###cloudiness###", "50.0")
                        data = data.replace("###precipitation###", "0.0")
                        data = data.replace("###precipitation_deposits###", "0.0")
                        data = data.replace("###wind_intensity###", "0.0")
                        data = data.replace("###sun_azimuth_angle###", "0.0")
                        data = data.replace("###sun_altitude_angle###", "0.0")
                        data = data.replace("###fog_density###", "0.0")
                        data = data.replace("###fog_distance###", "0.0")
                        data = data.replace("###fog_falloff###", "0.0")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "0.0")
                        data = data.replace("###mie_scattering_scale###", "0.0")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0")
                        data = data.replace("###dust_storm###", "0.0")
                    elif "Rainy" in value:
                        data = data.replace("###cloudiness###", "60.0")
                        data = data.replace("###precipitation###", "60.0")
                        data = data.replace("###precipitation_deposits###", "60.0")
                        data = data.replace("###wind_intensity###", "60.0")
                        data = data.replace("###sun_azimuth_angle###", "-1.0")
                        data = data.replace("###sun_altitude_angle###", "45.0")
                        data = data.replace("###fog_density###", "3.0")
                        data = data.replace("###fog_distance###", "0.75")
                        data = data.replace("###fog_falloff###", "0.1")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "1.0")
                        data = data.replace("###mie_scattering_scale###", "0.03")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0331")
                        data = data.replace("###dust_storm###", "0.0")
                    elif "Foggy" in value:
                        data = data.replace("###cloudiness###", "50.0")
                        data = data.replace("###precipitation###", "0.0")
                        data = data.replace("###precipitation_deposits###", "0.0")
                        data = data.replace("###wind_intensity###", "0.0")
                        data = data.replace("###sun_azimuth_angle###", "0.0")
                        data = data.replace("###sun_altitude_angle###", "0.0")
                        data = data.replace("###fog_density###", "0.0")
                        data = data.replace("###fog_distance###", "0.0")
                        data = data.replace("###fog_falloff###", "0.0")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "0.0")
                        data = data.replace("###mie_scattering_scale###", "0.0")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0")
                        data = data.replace("###dust_storm###", "0.0")
                    elif "Clear" in value:
                        data = data.replace("###cloudiness###", "5.0")
                        data = data.replace("###precipitation###", "0.0")
                        data = data.replace("###precipitation_deposits###", "0.0")
                        data = data.replace("###wind_intensity###", "10.0")
                        data = data.replace("###sun_azimuth_angle###", "-1.0")
                        data = data.replace("###sun_altitude_angle###", "45.0")
                        data = data.replace("###fog_density###", "2.0")
                        data = data.replace("###fog_distance###", "0.75")
                        data = data.replace("###fog_falloff###", "0.1")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "1.0")
                        data = data.replace("###mie_scattering_scale###", "0.03")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0331")
                        data = data.replace("###dust_storm###", "0.0")
                elif "Time" in key:
                    if "Dusk" in value:
                        data = data.replace("###cloudiness###", "5.0")
                        data = data.replace("###precipitation###", "0.0")
                        data = data.replace("###precipitation_deposits###", "0.0")
                        data = data.replace("###wind_intensity###", "10.0")
                        data = data.replace("###sun_azimuth_angle###", "-1.0")
                        data = data.replace("###sun_altitude_angle###", "15.0")
                        data = data.replace("###fog_density###", "2.0")
                        data = data.replace("###fog_distance###", "0.75")
                        data = data.replace("###fog_falloff###", "0.1")
                        data = data.replace("###wetness###", "0.0")
                        data = data.replace("###scattering_intensity###", "1.0")
                        data = data.replace("###mie_scattering_scale###", "0.03")
                        data = data.replace("###rayleigh_scattering_scale###", "0.0331")
                        data = data.replace("###dust_storm###", "0.0")
                    continue
        
        generated_script_xml_file_name = f'{NEW_FILE_PATH}/checkpoint{checkpoint_id}_testcase{testcase_id}_carla_script.xml'

        # Opening our text file in write only
        # mode to write the replaced content
        with open(generated_script_xml_file_name, 'w+') as file:
            # Writing the replaced data in our
            # text file
            file.write(data)

        # subprocess.run(["black",generated_script_file_name])
        carla_script_as_html = load_carla_script(generated_script_xml_file_name)
        return render_template('script.html' , carla_script_as_html = carla_script_as_html  )

@checkpoint_api.route('/<int:checkpoint_id>/generate_carla_script/<int:testcase_id>', methods=['GET'])
def generate_carla_script(checkpoint_id, testcase_id):

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:
        testcases = load_testcases_for_carla_script( checkpoint_id )
        context_dict = testcases[str(testcase_id)]['contextDict']
        input_dict = testcases[str(testcase_id)]['inputDict']

        intersection_type = context_dict['Checkpoint Type']
        town = "Town10HD"
        new_carla_script_arr = []

        carla_tutorial_data = ""
        with open("/usr/src/app/project/carla/follow_leading_vehicle.py", "r") as file:
            carla_tutorial_data = file.read()
        # TODO get ego vehicles coordinates
        x_coord_of_obj = "ego_vehicle_location.x"
        y_coord_of_obj = "ego_vehicle_location.y"
        z_coord_of_obj = "ego_vehicle_location.z"
        yaw_of_obj = "ego_vehicle_rotation.yaw"

        for line in carla_tutorial_data.splitlines():
            # if "#####OVERRIDE_WAYPOINT_DEFINITION#####" in line:
                #new_carla_script_arr.append(f'{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}waypoint = self._map.get_waypoint(location=waypoint_location["{town}"]["{intersection_type}"]["location"], project_to_road=True, lane_type=carla.LaneType.Driving){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}waypoint_rotation = waypoint_location["{town}"]["{intersection_type}"]["rotation"]{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}waypoint_loc = waypoint.transform.location{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}transform = carla.Transform(carla.Location(x=waypoint_loc.x, y=waypoint_loc.y, z=0.3), waypoint_rotation){CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}')
            if "#####END_ADD_BEHAVIORS#####" in line:
                new_carla_script_arr.append(carla_code_block_create_parallel_behavior())
                for key, value in context_dict.items():
                    if "My Car.Manuever" in key:
                        if value == "Straight":
                            new_carla_script_arr.append(
                                carla_code_block_create_ego_vehicle_behavior()
                            )
                for key, value in input_dict.items():
                    if "Car" in key:
                        if "yes" in value.lower():
                            new_carla_script_arr.append(
                                carla_code_block_create_vehicle_behavior(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj)
                            )
                    elif "Traffic Light" in key:
                        continue
                    elif "Pedestrian" in key:
                        new_carla_script_arr.append(
                            carla_code_block_create_ego_vehicle_behavior()
                        )
                    elif "Time" in key:
                        # TODO: Account for time of day (sun setting)
                        continue
                    elif "Weather" in key:
                        continue
                    else:
                        print(key)
                        print(value)
                        raise Exception("Object detected not accounted for when creating code block for Carla script")               
            elif "#####END_ADD_OBJECTS#####" in line:

                # Add objects actually in the picture
                f = open( os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}.json' ) )
                json_file = json.load( f )
                for object in json_file['objects']:
                    # Determine location with respect to ego vehicle
                    x_distance_away = str(abs(object['relative_coordinates']['center_x']-.5)*DISTANCE_SCALE)
                    y_distance_away = str((1-object["relative_coordinates"]["center_y"])*DISTANCE_SCALE)

                    forward = "Forward" if (object['relative_coordinates']['center_x']-.5) > 0.45 and (object['relative_coordinates']['center_x']-.5) < 0.55 else None
                    right_or_left = "Right" if (object['relative_coordinates']['center_x']-.5 > 0) else "Left"
                    y_coord_of_obj = generate_directional_reference_coordinate_y(forward, y_distance_away, intersection_type, town)
                    x_coord_of_obj = generate_directional_reference_coordinate_x(right_or_left, x_distance_away, intersection_type, town)

                    yaw_of_obj = "ego_vehicle_rotation.yaw"

                    if object["class_id"] == 1:
                        new_carla_script_arr.append(
                            f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj)}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                        )                

                # Add objects from context tree 
                for key, value in context_dict.items():
                    # Determine location with respect to ego vehicle

                    x_distance_away_in_pixels_of_pic = .3
                    y_distance_away_in_pixels_of_pic = .3
                    x_distance_away = str(abs(x_distance_away_in_pixels_of_pic-.5)*DISTANCE_SCALE)
                    y_distance_away = str((1-y_distance_away_in_pixels_of_pic)*DISTANCE_SCALE)

                    y_coord_of_obj = generate_directional_reference_coordinate_y(key, y_distance_away, intersection_type, town)
                    x_coord_of_obj = generate_directional_reference_coordinate_x(key, x_distance_away, intersection_type, town)

                    yaw_of_obj = "ego_vehicle_rotation.yaw"

                    if "Traffic Light" in key:
                        new_carla_script_arr.append(
                            f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj,'static.prop.trafficwarning')}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                        )
                    elif "Street Sign" in key:
                        new_carla_script_arr.append(
                            f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj, 'static.prop.streetsign')}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                        )
                    elif "Checkpoint Type" in key or "My Car" in key:
                        continue
                    else:
                        print(key)
                        print(value)
                        raise Exception("Object detected not accounted for when creating code block for Carla script")

                # Add objects from input tree
                for key, value in input_dict.items():
                    # Determine location with respect to ego vehicle
                    x_distance_away_in_pixels_of_pic = .5
                    y_distance_away_in_pixels_of_pic = .5
                    x_distance_away = str(abs(.2-x_distance_away_in_pixels_of_pic)*DISTANCE_SCALE)
                    y_distance_away = str((1-y_distance_away_in_pixels_of_pic)*DISTANCE_SCALE)

                    y_coord_of_obj = generate_directional_reference_coordinate_y(key, y_distance_away, intersection_type, town)
                    x_coord_of_obj = generate_directional_reference_coordinate_x(key, x_distance_away, intersection_type, town)

                    yaw_of_obj = "ego_vehicle_rotation.yaw"

                    if "Car" in key:
                        if "yes" in value.lower():
                            new_carla_script_arr.append(
                                f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj)}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                            )
                    elif "Traffic Light" in key:
                        new_carla_script_arr.append(
                            f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj,'static.prop.trafficwarning')}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                        )
                    elif "Pedestrian" in key:
                        new_carla_script_arr.append(
                            f"{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}{carla_code_block_create_agent(x_coord_of_obj, y_coord_of_obj, z_coord_of_obj, 0, 0, yaw_of_obj, 'walker')}{CARLA_SCRIPT_CODE_BLOCK_SEPARATOR}"
                        )
                    elif "Weather" in key:
                        continue                          
                    elif "Time" in key:
                        continue
                    else:
                        print(key)
                        print(value)
                        raise Exception("Object detected not accounted for when creating code block for Carla script")
            new_carla_script_arr.append(line)

        generated_script_file_name = f'{NEW_FILE_PATH}/checkpoint{checkpoint_id}_testcase{testcase_id}_carla_script.py'
        with open(generated_script_file_name, 'w+') as text_file:
            for line in new_carla_script_arr:
                text_file.write(line)
                text_file.write("\n")
        # subprocess.run(["black",generated_script_file_name])
        carla_script_as_html = load_carla_script(generated_script_file_name)
        return render_template('script.html' , carla_script_as_html = carla_script_as_html  )

@checkpoint_api.route('/<int:checkpoint_id>/generate')
def generate_test_case(checkpoint_id):

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        loginToken = login()

        functionName = f'checkpoint{checkpoint.id}'

        attUrl = ( f'djbackend/user/{USERNAME}/projects/trip{checkpoint.trip.id}/functions' )

        res = create_test_cases( attUrl, functionName, loginToken )
        
        if res.status_code >= 200 and res.status_code < 300:

            checkpoint.status = CheckpointStatus.TEST_CASES_GENERATED

            db.session.commit()
                
            time.sleep(1)

    return res.text
    
@checkpoint_api.route('/<int:checkpoint_id>/retrieve')
def retrieve_test_case(checkpoint_id):

    checkpoint = Checkpoint.query.get( checkpoint_id )

    if checkpoint:

        loginToken = login()

        functionName = f'checkpoint{checkpoint.id}'

        attUrl = ( f'djbackend/user/{USERNAME}/projects/trip{checkpoint.trip.id}/functions' )

        res = get_test_cases( attUrl, functionName, loginToken )
            
        if res.status_code >= 200 and res.status_code < 300:
        
            json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_testcases.json' )

            with open( json_path, 'w') as file:

                file.write( res.text )

            checkpoint.status = CheckpointStatus.TEST_CASES_RETRIEVED

            db.session.commit()
                
            time.sleep(1)

            return 'True'
            
    return 'False'

def allowed_file(filename):

    full_name = os.path.splitext(filename)

    return full_name[1] in ALLOWED_EXTENSIONS

def check_object( name ):

    pattern = r"--(.*?)--"
    
    traffic_obj = [ "traffic light" ]

    lanes = [ "direct", "alternative" ]

    new_name = None

    if name in traffic_obj:

        new_name = name.lower().replace(" ", "-")

    elif name.startswith("regulatory"):

        new_name = re.findall( pattern, name )
    
    elif name in lanes:

        new_name = name

    return new_name

def context_object():

    objects = {}

    # Traffic light
    objects["front traffic light"] = "Traffic light"
    objects["right traffic light"] = "Traffic light"
    objects["left traffic light"] = "Traffic light"
    objects["traffic light"] = "Traffic light"

    objects["car"] = "Vehicle"

    # Crosswalk
    objects["front crosswalk"] = "Crosswalk"
    objects["right crosswalk"] = "Crosswalk"
    objects["left crosswalk"] = "Crosswalk"

    # Traffic sign
    objects["no right turn"] = "Traffic sign"
    objects["no left turn"] = "Traffic sign"
    objects["no u turn"] = "Traffic sign"

    return objects

def input_object():

    objects = {"pedestrian", "car", "bike", "animal"}

    return objects

def dict_to_json( dictionary ):

    output = []

    for key, value in dictionary.items():

        child_dict = { "name" : key, "children" : [] }

        if isinstance( value, dict ):

            child_dict["children"] = dict_to_json( value )

        else:

            for item in value:

                child_dict["children"].append( { "name" : item } )

        output.append( child_dict )

    return output

def login():

    print("Login at AI Testing Tool")

    res = requests.post(

        url = f'{AI_TESTING_TOOL_API_URL}/{AI_TESTING_TOOL_LOGIN_URI}',
        data = { 'username' : USERNAME, 'password' : PASSWORD },

    )

    return res.json()["token"]

def create_test_cases( attUrl, functionName, loginToken ):

    AI_TESTING_TOOL_TESTCASES = f'{attUrl}/{functionName}/testcase.generate'
    
    print("Send request to generate test cases in AI Testing Tool")

    res = requests.post(

        url = f'{AI_TESTING_TOOL_API_URL}/{AI_TESTING_TOOL_TESTCASES}',
        headers = { 'Authorization' : f'Bearer {loginToken}'},
    )
    
    print("Test cases generated in AI Testing Tool")
    
    return res

def get_test_cases( attUrl, functionName, loginToken):

    AI_TESTING_TOOL_TESTCASES = f'{attUrl}/{functionName}/testcase'

    print("Send request to get test cases from AI Testing Tool")

    res = requests.get(

        url = f'{AI_TESTING_TOOL_API_URL}/{AI_TESTING_TOOL_TESTCASES}',
        headers={ 'Authorization' : f'Bearer {loginToken}'},

    )

    return res

def save_tree_json( checkpoint_id, name, data ):

    json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_{name}.json' )

    with open( json_path, 'w') as file:

        file.write( json.dumps( { "name" : name, "children" : data }, separators = ( ",", ":" ), ensure_ascii = False ) )

def load_tree( checkpoint_id, name ):

    json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_{name}.json' )

    json_download = f'/static/data/checkpoint{checkpoint_id}_{name}.json'

    if os.path.exists( json_path ):

        with open( json_path ) as file:

            data = json.load( file )

        return json_download, render_dict_as_div( data['children'] )
    
    return 

def load_carla_script_xml(checkpoint_id, name):
    json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_{name}.json' )

    json_download = f'/static/data/checkpoint{checkpoint_id}_{name}.json'

    with open( json_path ) as file:

        data = json.load( file )

    return json_download, render_file_as_html( data['children'] )

def load_carla_script(generated_script_file_name):
    with open( generated_script_file_name, 'r') as file:
        data = file.read()

    return data

def load_testcases( checkpoint_id ):

    data = None

    json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_testcases.json' )

    if os.path.exists( json_path ):

        with open( json_path ) as file:

            try:

                data = json.load( file )

                if 'testcases' in data:

                    data = data['testcases']

                    for aux in data:
                        
                        aux = filter_dict( aux )
            
                # Process the JSON data
            except json.JSONDecodeError:

                data = None

    return data

def load_testcases_for_carla_script( checkpoint_id ):

    data = None
    final_data = {}
    json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_testcases.json' )

    if os.path.exists( json_path ):

        with open( json_path ) as file:

            try:

                data = json.load( file )

                if 'testcases' in data:

                    data = data['testcases']

                    for aux in data:
                        
                        aux = filter_dict( aux )
                        final_data[aux['subkey']] = {}
                        final_data[aux['subkey']]['contextDict'] = aux['contextDict']
                        final_data[aux['subkey']]['inputDict'] = aux['inputDict']

                # Process the JSON data
            except json.JSONDecodeError:

                final_data = None

    return final_data

def filter_dict(dictionary):

    keys_to_keep = ["inputDict", "contextDict", "subkey"]

    for key in list( dictionary.keys() ):

        if key not in keys_to_keep:

            del dictionary[key]
    
    dictionary['subkey'] = dictionary['subkey'] .split("#")[0]

    return dictionary

@checkpoint_api.route('<int:checkpoint_id>/test', methods=['GET'])
def get_checkpoint_test( checkpoint_id ):

    # json_path = os.path.join( NEW_FILE_PATH, f'checkpoint{checkpoint_id}_output.json' )

    # with open( json_path ) as file:

    #     data = json.load( file )

    # return render_dict_as_div( data['children'] )

    return load_testcases( checkpoint_id )

def render_dict_as_div( data ):

    html = '<div class="branch">'
    
    for value in data:

        if len( data ) > 1:
        
            html += f'<div class="entry"><span class="label">{value["name"]}</span>'

        else:

            html += f'<div class="entry sole"><span class="label">{value["name"]}</span>'

        if 'children' in value:
    
            html += render_dict_as_div( value["children"] )
    
        html += '</div>'
    
    html += '</div>'
    
    return html


def get_html():

    get_checkpoint_test( 4 )

def get_traffic_sign():
    
    return [ 
            'dual-lanes-go-straight-on-left',
            'dual-lanes-go-straight-on-right',
            'dual-lanes-turn-left-no-u-turn',
            'dual-lanes-turn-left-or-straight',
            'dual-lanes-turn-right-or-straight',
            'go-straight',
            'go-straight-or-turn-left',
            'go-straight-or-turn-left--g2',
            'go-straight-or-turn-right',
            'keep-left',
            'keep-right',
            'left-turn-yield-on-green',
            'no-entry',
            'no-left-turn',
            'no-parking',
            'no-parking-or-no-stopping',
            'no-right-turn',
            'no-stopping',
            'no-straight-through',
            'no-turn-on-red',
            'no-turns',
            'no-u-turn',
            'one-way-left',
            'one-way-right',
            'one-way-straight',
            'road-closed',
            'road-closed-to-vehicles',
            'stop',
            'stop-here-on-red-or-flashing-light',
            'stop-signals',
            'turn-left',
            'turn-right',
            'turning-vehicles-yield-to-pedestrians',
            'u-turn',
            'wrong-way',
            'yield'

        ]
