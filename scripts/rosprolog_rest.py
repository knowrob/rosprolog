#!/usr/bin/env python3
import rospy
import os
from gevent.pywsgi import WSGIServer  # Web Server
from flask import Flask
from flask_restplus import Api, Resource, fields
from RosprologRestClient import RosprologRestClient

# Set KnowRob version and KnowRob Port from environment variables
KNOWROB_VERSION = os.getenv('KNOWROB_VERSION')
if KNOWROB_VERSION is None:
    KNOWROB_VERSION = '1.0'
else:
    KNOWROB_VERSION = str(KNOWROB_VERSION)

KNOWROB_PORT = os.getenv('KNOWROB_PORT')
if KNOWROB_PORT is None:
    KNOWROB_PORT = 62226
else:
    KNOWROB_PORT = int(KNOWROB_PORT)

app = Flask(__name__)
app.config['RESTPLUS_MASK_SWAGGER'] = False

# API titel
api = Api(app,
          version=KNOWROB_VERSION,
          title='KnowRob API',
          description='KnowRob API reference',
          )

# Query interface
query = api.model('Query', {
    'query': fields.String(required=True, description='The query string'),
    'maxSolutionCount': fields.Integer(required=True, default=100, description='The maximal number of solutions'),
    'response': fields.List(fields.Raw, readonly=True, description='The response list')
})

# Endpoint
ns = api.namespace('knowrob/api/' + KNOWROB_VERSION,
                description='Operations related to KnowRob')

# ROS Client for prolog
rosrest = RosprologRestClient()

# Query interface implementation
@ns.route("/query")
class Query(Resource):
    @ns.expect(query) # input model
    @ns.marshal_with(query)
    def post(self):
        rosrest.post_query(api.payload['query'])
        api.payload['response'] = rosrest.get_solutions(
            api.payload['maxSolutionCount'])
        return api.payload

if __name__ == '__main__':
    rospy.init_node('rosprolog_rest', anonymous=True)
    
    http_server = WSGIServer(('', KNOWROB_PORT), app)
    http_server.serve_forever()
