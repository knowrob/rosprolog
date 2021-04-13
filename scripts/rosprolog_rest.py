#!/usr/bin/env python3
import rospy
from gevent.pywsgi import WSGIServer  # Web Server
from flask import Flask
from flask_restplus import Api, Resource, fields
from RosprologRestClient import RosprologRestClient

app = Flask(__name__)
app.config['RESTPLUS_MASK_SWAGGER'] = False

# API titel
api = Api(app,
          version='1.0',
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
ns = api.namespace('knowrob/api/v1.0',
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
    http_server = WSGIServer(('', 62226), app)
    http_server.serve_forever()
